/*
 * spi_common.h
 *
 *  Created on: Nov 21, 2023
 *      Author: amosm
 */

#ifndef INC_SPI_COMMON_H_
#define INC_SPI_COMMON_H_

#include "main.h"
#define SPI_TIMEOUT 100

HAL_StatusTypeDef SPI_read_burst_implicit( SPI_HandleTypeDef * hspi, uint8_t reg_addr, uint8_t *data, size_t len,GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin );
HAL_StatusTypeDef SPI_write_burst_implicit(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t *data, size_t len, GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin);
HAL_StatusTypeDef SPI_write_burst_explicit( SPI_HandleTypeDef * hspi, uint8_t reg_addr, uint8_t *data, size_t len,GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin );
void delay_us(uint32_t period);
void delay_ms(uint32_t period);

#endif /* INC_SPI_COMMON_H_ */
