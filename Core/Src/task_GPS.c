

#include "minmea-master/minmea.h"

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef GPS_UART;

static HAL_StatusTypeDef gps_rcv_line(char *buffer, uint16_t length)
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

size_t get_len_of_cmd(const char *str)
{

	uint8_t i = 0;
    	while(i < 128 - 6)
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

static HAL_StatusTypeDef gps_send_line(const char *buffer)
{
	HAL_StatusTypeDef ret = HAL_OK;

	size_t cmdlen = get_len_of_cmd(buffer);
	const line_len = cmdlen + 6;// Buffer + '*'+ Checksum (2 Hex Chars) + CR + LF + null
	char *outbuf = (char *)malloc(line_len);
	if (outbuf == NULL)
	{
		return HAL_ERROR;
	}

	memcpy(outbuf, buffer, cmdlen);
	outbuf[cmdlen] = '\0';

	uint8_t chck = minmea_checksum(outbuf);
	sprintf(outbuf + cmdlen, "*%02X\n\r",chck );
    if (minmea_check(outbuf, true))
    {
        ret = HAL_UART_Transmit(&GPS_UART, outbuf, strlen(outbuf),HAL_MAX_DELAY);
    }
    else
    {
    	ret = HAL_ERROR;
    }

    free(outbuf);

    return ret;
}

void GPS_task(void *pvParameters)
{
    char line[500] = {};
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    HAL_StatusTypeDef ret = HAL_OK;
  // char  cmd []  = "$PGKC463,GOKE9501_1.3_17101100*22\r\n";
  // HAL_UART_Transmit(&GPS_UART, cmd, sizeof(cmd),HAL_MAX_DELAY);


  // ret =gps_send_line("$PGKC105,0"); 
  // 
  // ret =gps_send_line("$PGKC242,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0"); // set rcv Interval to 200ms
  // if (ret != HAL_OK)
  // {
  // 	printf("line1 failed\n");
  // }

  //ret =  gps_send_line("$PGKC161,2,50,100"); // set rcv Interval to 200ms
  // if (ret != HAL_OK)
  // {
  // 	printf("line2 failed\n");
  // }
  // ret =  gps_send_line("$PGKC161,2,50,100"); // set rcv Interval to 200ms
  //  if (ret != HAL_OK)
  //  {
  //  	printf("line2 failed\n");
  //  }

    while (1)
    {
        ret =gps_send_line("$PGKC463");
        ret =gps_send_line("$PGKC463");

        delay_ms(1);
        size_t i = 0;
        HAL_StatusTypeDef ret;

        ret = gps_rcv_line(line, sizeof(line));

        while (ret != HAL_OK)
        {
            ret = gps_rcv_line(line, sizeof(line));
        }

        printf("%s\n", line);



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
