

#include "minmea-master/minmea.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common_task_defs.h"
#include "FreeRTOS.h"
#include <stdarg.h>

enum en_gps_events
{
    ev_data_available = BIT(0)
};

#define MAX_GPS_CMD_LEN 128
#define CMD_RETRIES 10

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

void rcv_gps_uart_irq_handler(void)
{
    osEventFlagsSet(GPS_events, ev_data_available);
}

HAL_StatusTypeDef gps_transmit_recieve(const char *cmd, char *buffer, uint16_t length, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    if (buffer != NULL)
    {
        osEventFlagsClear(GPS_events, ev_data_available);
        ret |= HAL_UARTEx_ReceiveToIdle_DMA(&GPS_UART, (uint8_t *)buffer, length);
    }
    if (cmd != NULL && ret == HAL_OK)
    {
        ret |= HAL_UART_Transmit(&GPS_UART, cmd, strlen(cmd), timeout);
    }
    if (buffer != NULL)
    {
        uint32_t eventret = osEventFlagsWait(GPS_events, ev_data_available, NULL, timeout);
        if (*(int32_t *)&eventret < 0)
        {
            buffer[0] = '\0';
            ret = HAL_TIMEOUT;
        }
        else
        {
            buffer[length - GPS_UART.RxXferCount] = '\0';
        }
    }
    HAL_UART_AbortReceive(&GPS_UART);
    return ret;
}

HAL_StatusTypeDef gps_recieve(char *buffer, uint16_t length, uint32_t timeout)
{
    return gps_transmit_recieve(NULL, buffer, length, timeout);
}

HAL_StatusTypeDef gps_send(const char *cmd, uint32_t timeout)
{
    return gps_transmit_recieve(cmd, NULL, 0, timeout);
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
        }
        break;
    }
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc sentence;
        bool ret = minmea_parse_rmc(&sentence, rcv);
        if (ret)
        {
            dat->time = sentence.time;
            dat->date = sentence.date;
            dat->ticks = osKernelGetTickCount();
            dat->latitude = sentence.latitude.value / (double)sentence.latitude.scale;
            dat->longitude = sentence.longitude.value / (double)sentence.longitude.scale;
            dat->ground_speed = KNOTS_TO_MS * sentence.speed.value / (double)sentence.speed.scale;
        }
        break;
    }
    }
}

int gps_find_and_parse_NMEA(char *rcv, gps_data_t *dat)
{
    memset(dat, -1, sizeof(*dat));
    DEBUG_PRINT("Parsing NMEA: \n%s\n", rcv);
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

int gps_send_cmd(uint32_t cmd, uint32_t timeout, const char *fmt, ...)
{
    int ret = 0;
    char rcv[2048] = {};
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

        DEBUG_PRINT("---------------Sent: %s", line);

        if (gps_transmit_recieve(line, rcv, sizeof(rcv), timeout) == HAL_OK)
        {
            DEBUG_PRINT("Answer: \n%s------------\n", rcv)

            int status;
            while (1)
            {
                status = gps_find_and_parse_ack(cmd, rcv);
                if (status != -1)
                {
                    break;
                }
                if (gps_recieve(rcv, sizeof(rcv), timeout) != HAL_OK)
                {
                    break;
                }
                DEBUG_PRINT("Answer secondary: \n%s------------\n", rcv)
            }

            if (status == 3)
            {
                ret = 0;
                break;
            }
            if (status >= 0)
            {
                printf("GPS command ack: %d\n", status);
            }
        }

        osDelay(20);
    }
    return ret;
}

int gps_set_mode(uint32_t mode, uint32_t timeout)
{
    gps_send_cmd(225, timeout, "%u", mode);
}

int confirm_comm()
{
    return gps_send_cmd(0, 100, NULL);
}

int set_uart_baud(uint32_t baudrate)
{
    GPS_UART.Init.BaudRate = baudrate;
    HAL_StatusTypeDef ret = HAL_UART_Init(&GPS_UART);
    if (ret != HAL_OK)
    {
        printf("GPS Uart change failed\n");
        return -1;
    }
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
    uint32_t baud_b4 = GPS_UART.Init.BaudRate;

    char line[MAX_GPS_CMD_LEN];
    gps_build_cmd(line, sizeof(line), 251, "%u", baudrate);
    gps_send(line, 100);

    set_uart_baud(baudrate);

    return confirm_comm();
}

void GPS_task(void *pvParameters)
{
    char inbuf[2048] = {0};
    char outbuf[MAX_GPS_CMD_LEN] = {0};

    gps_data_t gps_data = {0};
    gps_data_t disc;

    GPS_events = osEventFlagsNew(NULL);

    HAL_StatusTypeDef ret = HAL_OK;

    DEBUG_PRINT("---------------------------------------------------------------\nGPS task started\n");
    set_uart_baud(9600);
    if (confirm_comm() < 0)
    {
        DEBUG_PRINT("failed to establish connection on 9600, trying 57100\n");
        set_uart_baud(115200);
        if (confirm_comm() < 0)
        {
            DEBUG_PRINT("failed to establish connection on 57100, trying 115200\n");
            set_uart_baud(115200);
            if (confirm_comm() < 0)
            {
                DEBUG_PRINT("failed to establish connection with GPS on 115200, returning\n");
                return;
            }
        }
    }

    // gps_set_mode(4, 900);

    if (gps_set_baudrate(115200) < 0)
    {
        DEBUG_PRINT("failed baud change\n");
    }

    if (gps_set_base_interval(1000) < 0)
    {
        DEBUG_PRINT("failed interval change\n");
    }
    if (gps_set_message_interval_scaler(1, 1, 1, 1, 1, 1) < 0)
    {
        DEBUG_PRINT("failed individual message interval change\n");
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
    // gps_set_mode(0, 1000); // GPS doesn't seem to confirm the first attempt so has to fail

    while (1)
    {
        gps_recieve(inbuf, sizeof(inbuf), 1100);
        printf("------------------------\n%s\n----------------", inbuf);
        gps_find_and_parse_NMEA(inbuf, &gps_data);
        printf("\n - \n");
        osDelay(20);
    }
}
