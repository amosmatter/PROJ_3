

#include "minmea-master/minmea.h"

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

void GPS_task(void *pvParameters)
{
    char line[128] = {};
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    HAL_StatusTypeDef ret = HAL_OK;
    //set_normal_mode();

    {
        const char *aline = "$PGKC146,3,3,115200*07\r\n";
        HAL_UART_Transmit(&GPS_UART, aline, strlen(aline), HAL_MAX_DELAY);
    }
    {
        const char *aline = "$PGKC147,115200*06\r\n";
        HAL_UART_Transmit(&GPS_UART, aline, strlen(aline), HAL_MAX_DELAY);
    }
    // char  cmd []  = "$PGKC463,GOKE9501_1.3_17101100*22\r\n";
    // HAL_UART_Transmit(&GPS_UART, cmd, sizeof(cmd),HAL_MAX_DELAY);

    // ret =gps_send_line("$PGKC105,0");
    //
    // ret =gps_send_line("$PGKC242,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0"); // set rcv Interval to 200ms
    // if (ret != HAL_OK)
    // {
    // 	printf("line1 failed\n");
    // }

    // ret =  gps_send_line("$PGKC161,2,50,100"); // set rcv Interval to 200ms
    //  if (ret != HAL_OK)
    //  {
    //  	printf("line2 failed\n");
    //  }
    //  ret =  gps_send_line("$PGKC161,2,50,100"); // set rcv Interval to 200ms
    //   if (ret != HAL_OK)
    //   {
    //   	printf("line2 failed\n");
    // ret = HAL_UART_Transmit(&GPS_UART, "$PGKC030,3,1*2E\r\n", 17, 1000);
    // if (ret != HAL_OK)
    // {
    //     printf("restart failed\n");
    // }

    // ret = HAL_UART_Transmit(&GPS_UART, "$PGKC115,1,0,0,0*2B\r\n", 21, 1000);
    // if (ret != HAL_OK)
    // {
    //     printf("setting GPS failed\n");
    // }

    // ret = HAL_UART_Transmit(&GPS_UART, "$PGKC105,0*37\r\n", 15, 1000);
    // if (ret != HAL_OK)
    // {
    //     printf("line1 failed\n");
    // }

    // ret = HAL_UART_Transmit(&GPS_UART, "$PGKC030,3,1*2E\r\n", 17, 1000);
    // if (ret != HAL_OK)
    // {
    //     printf("line2 failed\n");
    // }

    //   }
    uint8_t ctr = 0;
    while (1)
    {

        delay_ms(1);
        size_t i = 0;
        HAL_StatusTypeDef ret;

        ret = gps_rcv_line(line, sizeof(line), 300);

        if (ret != HAL_OK)
        {
            printf("Trying to  get software version:\n");
            get_software_version();

            // ret = gps_send_line("$PGKC463",1000);
            // ret = get_nmea_intervall();
            continue;
            // ret = gps_rcv_line(line, sizeof(line), 1);
            // printf("%s\n", line);
        }
        printf("%s\n", line);

        if (strncmp((char *)line + 3, "TXT", 3) == 0)
        {
            printf("\n\n\n\r");
        }

        continue;
        if (strncmp((char *)line + 3, "GGA", 3) == 0)
        {

            struct minmea_sentence_gga sentence;
            minmea_parse_gga(&sentence, line);
            bool valid = minmea_check(line, false); 
            if (valid)
            {
                printf("Latitude: %d\n", sentence.latitude.value);
            }
        }
    }
}
