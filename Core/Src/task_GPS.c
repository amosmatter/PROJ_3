

#include "minmea-master/minmea.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common_task_defs.h"
#include "FreeRTOS.h"

enum events
{
    ev_line_rcv = BIT(0)
};

osEventFlagsId_t GPS_events;

extern UART_HandleTypeDef GPS_UART;

static HAL_StatusTypeDef gps_rcv_line(char *buffer, uint16_t length, uint32_t timeout)
{
    size_t i = 0;
    HAL_StatusTypeDef ret;

    for (i = 0; i < length; i++)
    {
        ret = HAL_UART_Receive(&GPS_UART, (uint8_t *)&buffer[i], 1, timeout);
        if (buffer[i] == '\n' || ret != HAL_OK)
        {
            break;
        }
    }
    buffer[i + 1] = '\0';
    return ret;
}

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
static HAL_StatusTypeDef gps_send_line(const char *buffer, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;

    size_t cmdlen = get_len_of_cmd(buffer);
    const line_len = cmdlen + 6; // Buffer + '*'+ Checksum (2 Hex Chars) + CR + LF + null
    char *outbuf = (char *)malloc(line_len);
    if (outbuf == NULL)
    {
        return HAL_ERROR;
    }

    memcpy(outbuf, buffer, cmdlen);
    outbuf[cmdlen] = '\0';

    uint8_t chck = minmea_checksum(outbuf);
    sprintf(outbuf + cmdlen, "*%02X\n\r", chck);
    if (minmea_check(outbuf, true))
    {
        ret = HAL_UART_Transmit(&GPS_UART, outbuf, strlen(outbuf), timeout);
    }
    else
    {
        ret = HAL_ERROR;
    }

    free(outbuf);

    return ret;
}

HAL_StatusTypeDef set_normal_mode(void)
{
    return HAL_UART_Transmit(&GPS_UART, "$PGKC105,0*37\r\n", 15, 1000);
}
HAL_StatusTypeDef get_software_version(void)
{
    return HAL_UART_Transmit(&GPS_UART, "$PGKC462*2F\r\n", 13, HAL_MAX_DELAY);
}

HAL_StatusTypeDef get_nmea_intervall(void)
{
    char line[128] = {};

    HAL_StatusTypeDef ret = HAL_UART_Transmit(&GPS_UART, "$PGKC201*2C\r\n", 17, 1000);

    if (ret != HAL_OK)
    {
        printf("req failed\n");
        return HAL_ERROR;
    }

    ret = gps_rcv_line(line, sizeof(line), 10);

    if (ret != HAL_OK)
    {
        printf("rec failed\n");
        return HAL_ERROR;
    }

    if (!minmea_check(line, true))

    {
        printf("check failed\n");
        return HAL_ERROR;
    }

    if (!strncmp(line, "$PGKC202", 8) == 0)
    {
        printf("Wrong response\n");
        return HAL_ERROR;
    }
    printf("%s\n", line);
    return HAL_OK;
}

void rcv_gps_uart_irq_handler(void)
{
    osEventFlagsSet(GPS_events, ev_line_rcv);
}

#define RCVD_GGA_FLAG BIT(0)
#define RCVD_VTG_FLAG BIT(1)

void GPS_task(void *pvParameters)
{
    char line[2048] = {};
    gps_data_t gps_data = {0};
    gps_data_t disc;

    GPS_events = osEventFlagsNew(NULL);

    HAL_StatusTypeDef ret = HAL_OK;

    uint8_t ctr = 0;
    while (1)
    {

        HAL_UARTEx_ReceiveToIdle_DMA(&GPS_UART, line, sizeof(line));
        osEventFlagsWait(GPS_events, ev_line_rcv, NULL, osWaitForever);
        gps_data.ticks = osKernelGetTickCount();
        osEventFlagsClear(GPS_events, ev_line_rcv);
        
        char *ptr = line;
        uint32_t flag = 0;
        while (*ptr != '\0')
        {

            if (!strncmp((char *)ptr + 3, "GGA", 3))
            {
                struct minmea_sentence_gga sentence;
                bool ret = minmea_parse_gga(&sentence, line);
                if (ret)
                {

                    gps_data.time = sentence.time.hours * 3600* 1000 + sentence.time.minutes * 60 * 1000 + sentence.time.seconds * 1000 + sentence.time.microseconds / 1000; 
                    gps_data.altitude = sentence.altitude.value / (double)sentence.altitude.scale;
                    gps_data.latitude = sentence.latitude.value / (double)sentence.latitude.scale;
                    gps_data.longitude = sentence.longitude.value / (double)sentence.longitude.scale;
                    flag |= RCVD_GGA_FLAG;
                }
            }

            if (!strncmp((char *)ptr + 3, "VTG", 3))
            {
                struct minmea_sentence_vtg sentence;
                bool ret = minmea_parse_vtg(&sentence, line);
                if (ret)
                {
                    gps_data.ground_speed = (1 / 3.6) * sentence.speed_kph.value / (double)sentence.speed_kph.scale;
                    flag |= RCVD_VTG_FLAG;
                }
            }

            while (*ptr != '\n' && *ptr)
            {
                ptr++;
            }
            if (!*ptr)
            {
                break;
            }
            ptr++;
        }

        if (flag & RCVD_GGA_FLAG && flag & RCVD_VTG_FLAG)
        {
            while (osMessageQueueGetCount(gps_data_queue_handle) != 0)
            {
                osMessageQueueGet(gps_data_queue_handle, &disc, 0, 0);
            }
            osMessageQueuePut(gps_data_queue_handle, &gps_data, 0, 0);
        }
    }
}
