/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAVLINK huart1
#define GPS_UART huart2
#define I2C hi2c1
#define SPI hspi1
#define CONSOLE_UART huart3
#define PS_I_MON_Pin GPIO_PIN_14
#define PS_I_MON_GPIO_Port GPIOC
#define PS_nFLT_Pin GPIO_PIN_15
#define PS_nFLT_GPIO_Port GPIOC
#define PS_nFLT_EXTI_IRQn EXTI15_IRQn
#define PPS_Pin GPIO_PIN_0
#define PPS_GPIO_Port GPIOA
#define SD_nCS_Pin GPIO_PIN_1
#define SD_nCS_GPIO_Port GPIOB
#define IMU_FSYNC_Pin GPIO_PIN_13
#define IMU_FSYNC_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_14
#define IMU_INT_GPIO_Port GPIOB
#define IMU_nCS_Pin GPIO_PIN_15
#define IMU_nCS_GPIO_Port GPIOB
#define PTH_nCS_Pin GPIO_PIN_8
#define PTH_nCS_GPIO_Port GPIOA
#define RPI_nCS_Pin GPIO_PIN_11
#define RPI_nCS_GPIO_Port GPIOA
#define LED_READY_Pin GPIO_PIN_5
#define LED_READY_GPIO_Port GPIOB
#define DEBUG_TRIGGER_Pin GPIO_PIN_3
#define DEBUG_TRIGGER_GPIO_Port GPIOH
#define SW_ACTIVE_Pin GPIO_PIN_8
#define SW_ACTIVE_GPIO_Port GPIOB
#define SW_ACTIVE_EXTI_IRQn EXTI8_IRQn

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
