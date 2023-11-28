
#include "main.h"

#include "spi_common.h"

#include <stdio.h>
#include <stdlib.h>

#include "cmsis_os2.h"
#include <string.h>

void delay_ms(uint32_t period)
{
	osDelay(period);
}

void delay_us(uint32_t period)
{
	for (volatile uint64_t i = 0; i < 8 * period; i++)
		;

	return;

	uint32_t clk_cycle_start = DWT->CYCCNT; // TODO: get this working

	/* Go to number of cycles for system */
	period *= (HAL_RCC_GetHCLKFreq() / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - clk_cycle_start) < period)
		;
}

HAL_StatusTypeDef SPI_read_burst_implicit(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t *data, size_t len, GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin)
{
	uint8_t *send_buf = calloc(len + 1, 0);
	send_buf[0] = ((reg_addr & 0x7F) | 0x80);
	uint8_t *rcv_buf = malloc(len + 1);

	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(hspi, send_buf, rcv_buf, len + 1, SPI_TIMEOUT);
	delay_us(2);
	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);

	memcpy(data, &rcv_buf[1], len);

	free(send_buf);
	free(rcv_buf);
	return ret;
}
HAL_StatusTypeDef SPI_write_burst_implicit(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t *data, size_t len, GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin)
{
	uint8_t *send_buf = malloc(len + 1);
	send_buf[0] = (reg_addr & 0x7F);

	memcpy(&send_buf[1], data, len);
	
	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, send_buf, 2 * len, SPI_TIMEOUT);
	delay_us(2);
	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);

	free(send_buf);
	return ret;
}


HAL_StatusTypeDef SPI_write_burst_explicit(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t *data, size_t len, GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin)
{
	if (len == 0)
		return HAL_OK;

	if (len > 127)
		return HAL_ERROR;

	uint8_t *send_buf = malloc(2 * len);

	for (uint32_t i = 0; i < len; i++)
	{
		send_buf[2 * i] = ((reg_addr + i) & 0x7F);
		send_buf[2 * i + 1] = data[i];
	}

	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, send_buf, 2 * len, SPI_TIMEOUT);
	delay_us(2);
	HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);

	free(send_buf);
	return ret;
}

