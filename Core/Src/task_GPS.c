

#include "minmea-master/minmea.h"

#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef GPS_UART;

HAL_StatusTypeDef rcv_str(uint8_t *buffer, uint16_t length)
{
    size_t i = 0;
    HAL_StatusTypeDef ret;

    for (i = 0; i < length; i++)
    {
        ret = HAL_UART_Receive(&GPS_UART, (uint8_t *)&buffer[i], 1, HAL_MAX_DELAY);

        if (buffer[i] == '\n' || ret != HAL_OK)
        {
            break;
        }
    }
    buffer[i + 1] = '\0';
    return ret;
}

void GPS_task(void *pvParameters)
{
    uint8_t line[500] = {};

    while (1)
    {

        delay_ms(1);
        size_t i = 0;
        HAL_StatusTypeDef ret;

        ret = rcv_str(line, sizeof(line));

        while (ret != HAL_OK)
        {
            ret = rcv_str(line, sizeof(line));
        }
        if (strncmp((char *)line + 3, "GGA", 3) == 0)
        {
            printf("\n\n");
        }


        if (strncmp((char *)line + 3, "GSV", 3) == 0)
        {
            continue;
        }
        if (strncmp((char *)line + 1, "GPTXT", 5) == 0)
        {
            continue;
        }
        if (strncmp((char *)line + 3, "GLL", 3) == 0)
        {
            continue;
        }
                if (strncmp((char *)line + 3, "RMC", 3) == 0)
        {
            continue;
        }

        printf("%s\n", line);

        if (strncmp((char *)line + 3, "GGA", 3) == 0)
        {

          struct minmea_sentence_gga sentence;
		  minmea_parse_gga(&sentence, line);
        }

    }
}
