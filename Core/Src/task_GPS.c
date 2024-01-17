#include "minmea-master/minmea.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common_task_defs.h"
#include "FreeRTOS.h"
#include <stdarg.h>
#include "system_time.h"

enum command_results
{
    UNKNOWN_CMD_ERROR = -1,
    INVALID_COMMAND = 0,
    UNSUPPORTED_COMMAND = 1,
    VALID_COMMAND_ACTION_FAIL = 2,
    VALID_COMMAND_ACTION_SUCCEEDED = 3,
};

enum en_gps_events
{
    ev_data_available = BIT(0)
};
#define SECTION_LEN 1024
#define N_SECTIONS 5

#define MAX_GPS_CMD_LEN 128
#define CMD_RETRIES 10
#define ACK_RETRIES 2
#define ACK_TIMEOUT 100

#define KNOTS_TO_MS 0.51444444444
osEventFlagsId_t GPS_events;

extern UART_HandleTypeDef GPS_UART;

size_t get_len_of_cmd(const char *str)
{

    uint8_t i = 0;
    while (i < 128 - 6)
    {
        switch (str[i])
        {
        case '\0':
        case '\n':
        case '\r':
        case '*':
        case ' ':
            return i;
        }
        i++;
    }

    return 0; // Invalid command
}

size_t get_line_len(char *str)
{
    size_t i = 0;
    while (str[i] != '\0' && str[i] != '\n')
    {
        i++;
        if (i > MAX_GPS_CMD_LEN)
        {
            return 0;
        }
    }
    return i;
}

osMessageQueueId_t rcv_queue;

void process_uart_package(void)
{
    static uint8_t first_time_flag = 1;
    static uint8_t sections[N_SECTIONS][SECTION_LEN];
    static uint8_t ctr = 0;

    uint8_t *sectionptr = sections[ctr];

    if (!first_time_flag)
    {
        osStatus_t ret = osMessageQueuePut(rcv_queue, &sectionptr, 0, 0);
        if (ret != osOK)
        {
            DEBUG_PRINT("GPS Recieve Overrun detected!\n");
        }
        ctr = (ctr >= N_SECTIONS - 1) ? 0 : ctr + 1;
    }
    first_time_flag = 0;

    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&GPS_UART, sections[ctr], SECTION_LEN);
    if (ret != HAL_OK)
    {
        HAL_UART_Abort(&GPS_UART);
        HAL_UARTEx_ReceiveToIdle_DMA(&GPS_UART, sections[ctr], SECTION_LEN);
    }
}

void rcv_gps_uart_irq_handler(void)
{
    uint32_t isrflags = READ_REG(GPS_UART.Instance->ISR);
    if (isrflags & USART_ISR_IDLE)
    {
        process_uart_package();
    }
}

int vgps_build_cmd(char *buffer, size_t maxlen, uint32_t cmd, const char *fmt, va_list args)
{
    if (!buffer || !maxlen)
    {
        return -1;
    }

    size_t allwritten = 0;

    int n_written = snprintf(buffer, maxlen, "$PMTK%03lu", cmd);
    if (n_written < 1 || n_written >= maxlen)
    {
        buffer[0] = '\0';
        return -1;
    }

    allwritten += n_written;
    buffer += n_written;
    maxlen -= n_written;

    if (fmt != NULL)
    {

        if (maxlen < 1)
        {
            buffer[0] = '\0';
            return -1;
        }
        buffer[0] = ',';
        allwritten++;
        buffer++;
        maxlen--;

        n_written = vsnprintf(buffer, maxlen, fmt, args);

        if (n_written < 1 || n_written >= maxlen)
        {
            buffer[0] = '\0';
            return -1;
        }

        allwritten += n_written;
        buffer += n_written;
        maxlen -= n_written;
    }

    uint8_t chck = minmea_checksum(buffer - allwritten);
    n_written = snprintf(buffer, maxlen, "*%02X\r\n", chck);

    if (n_written < 1 || n_written >= maxlen)
    {
        buffer[0] = '\0';
        return -1;
    }
    allwritten += n_written;

    return allwritten;
}
int gps_build_cmd(char *buffer, size_t maxlen, uint32_t cmd, const char *fmt, ...)
{
    int ret;
    va_list args;
    va_start(args, fmt);
    ret = vgps_build_cmd(buffer, maxlen, cmd, fmt, args);
    va_end(args);
    return ret;
}

int gps_parse_ack(int incmd, char *rcv)
{

    char type[10] = {};
    int rcvcmd;
    int status;

    if (strncmp(rcv, "$PMTK001", 8) != 0)
    {
        return -1;
    }

    if (!minmea_scan(rcv, "tii", type, &rcvcmd, &status))
    {
        return -1;
    }
    if (rcvcmd != incmd)
    {
        return -1;
    }

    return status;
}

int gps_find_and_parse_ack(int incmd, char *rcv)
{

    size_t j = 0;
    size_t end = strlen(rcv);
    int status = -1;

    while (status < 0 && j <= end && rcv[j] != '\0')
    {
        if (rcv[j] == '$')
        {
            uint32_t linelen = get_line_len(&rcv[j]);
            rcv[j + linelen] = '\0';
            status = gps_parse_ack(incmd, &rcv[j]);
            if (status != -1)
            {
                break;
            }
            j += linelen + 1;
        }
        else
        {
            j++;
        }
    }
    return status;
}

int parse_NMEA(char *rcv, gps_data_t *dat)
{
    switch (minmea_sentence_id(rcv, 1))
    {
    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga sentence;
        bool ret = minmea_parse_gga(&sentence, rcv);
        if (ret)
        {
            dat->altitude = sentence.altitude.value / (double)sentence.altitude.scale;
            dat->fix_quality = sentence.fix_quality;
        }
        break;
    }
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc sentence;
        bool ret = minmea_parse_rmc(&sentence, rcv);
        if (ret)
        {

            struct tm time = {};
            time.tm_hour = sentence.time.hours;
            time.tm_min = sentence.time.minutes;
            time.tm_sec = sentence.time.seconds;

            time.tm_year = sentence.date.year + 100;
            time.tm_mon = sentence.date.month - 1;
            time.tm_mday = sentence.date.day;
            uint32_t ms = sentence.time.microseconds / 1000;
            set_time(&time, &ms);

            dat->latitude = minmea_tocoord(&sentence.latitude);
            dat->longitude = minmea_tocoord(&sentence.longitude);
            dat->ground_speed = KNOTS_TO_MS * sentence.speed.value / (double)sentence.speed.scale;
        }
        break;
    }
    }
}

int gps_find_and_parse_NMEA(char *rcv, gps_data_t *dat)
{
    memset(dat, -1, sizeof(*dat));
    size_t j = 0;
    size_t end = strlen(rcv);
    int status = -1;
    while (status < 0 && j <= end && rcv[j] != '\0')
    {
        if (rcv[j] == '$')
        {
            uint32_t linelen = get_line_len(&rcv[j]);
            rcv[j + linelen] = '\0';
            parse_NMEA(&rcv[j], dat);
            j += linelen;
        }
        j++;
    }
}
int gps_gps_send_cmd(uint32_t cmd, char *line, uint32_t timeout)
{
    char *rcv;

    HAL_StatusTypeDef hal_st = HAL_UART_Transmit(&GPS_UART, (uint8_t *)line, strlen(line), timeout);
    if (hal_st != HAL_OK)
    {
        DEBUG_PRINT("Failed to send: %s %d\n", line, hal_st);
        return -1;
    }

    int ret = -1;
    for (int i = 0; i < ACK_RETRIES; i++)
    {
        osStatus_t status = osMessageQueueGet(rcv_queue, &rcv, 0, ACK_TIMEOUT);
        if (status == osErrorTimeout)
        {
            break;
        }

        ret = gps_find_and_parse_ack(cmd, rcv);
        if (ret != -1)
        {
            break;
        }
    }
    return ret;
}
int gps_send_cmd(uint32_t cmd, uint32_t timeout, const char *fmt, ...)
{
    int ret = 0;
    char line[MAX_GPS_CMD_LEN];

    va_list args;
    va_start(args, fmt);
    ret = vgps_build_cmd(line, sizeof(line), cmd, fmt, args);
    va_end(args);

    if (ret <= 0)
    {
        printf("build failed\n");
        return -1;
    }

    ret = -1;
    for (int i = 0; i < CMD_RETRIES; i++)
    {
        ret = gps_gps_send_cmd(cmd, line, timeout);
        if (ret == VALID_COMMAND_ACTION_SUCCEEDED || ret == UNSUPPORTED_COMMAND)
        {
            break;
        }
        if (ret == VALID_COMMAND_ACTION_FAIL)
        {
            osDelay(100);
        }
    }
    return ret;
}

int gps_set_mode(uint32_t mode, uint32_t timeout)
{
    gps_send_cmd(225, timeout, "%u", mode);
}

int confirm_comm()
{
    return (gps_send_cmd(0, 1000, NULL) == 3) ? 1 : 0;
}

int set_uart_baud(uint32_t baudrate)
{
    HAL_UART_Abort(&GPS_UART);

    GPS_UART.Init.BaudRate = baudrate;
    HAL_StatusTypeDef ret = HAL_UART_Init(&GPS_UART);
    if (ret != HAL_OK)
    {
        printf("GPS Uart change failed\n");
        return -1;
    }
    process_uart_package();

    return 0;
}

int gps_set_base_interval(uint32_t interval)
{
    return gps_send_cmd(220, 100, "%u", interval);
}

int gps_set_message_interval_scaler(uint32_t gll_int, uint32_t rmc_int, uint32_t vtg_int, uint32_t gga_int, uint32_t gsa_int, uint32_t gsv_int)
{
    return gps_send_cmd(314, 100, "%u,%u,%u,%u,%u,%u,0,0,0,0,0,0,0,0,0,0,0,0,0", gll_int, rmc_int, vtg_int, gga_int, gsa_int, gsv_int);
}

int gps_set_baudrate(uint32_t baudrate)
{
    char line[MAX_GPS_CMD_LEN];
    gps_build_cmd(line, sizeof(line), 251, "%u", baudrate);
    HAL_UART_Transmit(&GPS_UART, (uint8_t *)line, strlen(line), 100);
    set_uart_baud(baudrate);

    return confirm_comm();
}

void GPS_task(void *pvParameters)
{

    rcv_queue = osMessageQueueNew(N_SECTIONS, sizeof(void *), NULL);
    set_uart_baud(19200);
    if (!confirm_comm())
    {
        DEBUG_PRINT("failed to establish GPS connection on 19200, trying 9600\n");
        set_uart_baud(9600);
        if (!confirm_comm())
        {
            DEBUG_PRINT("failed to establish connection with GPS on 9600, restarting!\n");

            HAL_NVIC_SystemReset();
        }
    }

    // gps_set_mode(4, 900);
    int ret = gps_set_baudrate(19200);

    if (!ret)
    {
        DEBUG_PRINT(" GPS  failed baud change, restarting \n");
        HAL_NVIC_SystemReset();
    }

    ret = gps_set_base_interval(1000 / OUTPUT_RATE); // This is the slowest signal so the whole system will sync to this intervall
    if (ret != VALID_COMMAND_ACTION_SUCCEEDED)
    {
        DEBUG_PRINT(" GPS failed base interval change %d, restarting\n", ret);
        HAL_NVIC_SystemReset();
    }

    if (gps_set_message_interval_scaler(0, 1, 0, 1, 0, 0) != VALID_COMMAND_ACTION_SUCCEEDED)
    {
        DEBUG_PRINT(" GPS failed individual message interval change\n");
    }

    //   if (gps_send_cmd(250, 100, "%u", 115200) < 0) // set dgps baud
    //   {
    //       DEBUG_PRINT("failed interval change\n");
    //   }
    //    if (gps_send_cmd(301, 100, "%u", 1) < 0) // set dgps to rtcm
    //   {
    //        DEBUG_PRINT("failed interval change\n");
    //    }
    //
    //   if (gps_send_cmd(313, 100, "%u", 1) < 0) // enable SBAS
    //   {
    //       DEBUG_PRINT("failed interval change\n");
    //   }

    osEventFlagsSet(init_events, ev_init_gps);

    gps_data_t gps_data = {0};

    while (1)
    {
        uint8_t *buf;
        osStatus_t osstat = osMessageQueueGet(rcv_queue, &buf, 0, (1000 / OUTPUT_RATE) * 1.1);
        if (osstat != osOK)
        {
            DEBUG_PRINT(" GPS Something went wrong with Communication...\n");
            process_uart_package();
            osMessageQueueReset(rcv_queue);
            continue;
        }

        gps_find_and_parse_NMEA(buf, &gps_data);
        osMessageQueuePut(gps_data_queue_handle, &gps_data, 0, 0);
    }
}
