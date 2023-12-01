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
#define GPS_UART huart2
#define MAVLINK huart3
#define SPI hspi3
#define I2C hi2c2
#define BUT_REF_INT_Pin GPIO_PIN_13
#define BUT_REF_INT_GPIO_Port GPIOC
#define BUT_REF_INT_EXTI_IRQn EXTI13_IRQn
#define IMU_FSYNC_Pin GPIO_PIN_3
#define IMU_FSYNC_GPIO_Port GPIOF
#define IMU_INT_Pin GPIO_PIN_5
#define IMU_INT_GPIO_Port GPIOF
#define IMU_INT_EXTI_IRQn EXTI5_IRQn
#define SD_nCS_Pin GPIO_PIN_13
#define SD_nCS_GPIO_Port GPIOF
#define RPI_nCS_Pin GPIO_PIN_14
#define RPI_nCS_GPIO_Port GPIOF
#define DEBUG_TRIGGER_Pin GPIO_PIN_0
#define DEBUG_TRIGGER_GPIO_Port GPIOG
#define PTH_nCS_Pin GPIO_PIN_11
#define PTH_nCS_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define IMU_nCS_Pin GPIO_PIN_2
#define IMU_nCS_GPIO_Port GPIOD
#define LED_READY_Pin GPIO_PIN_7
#define LED_READY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#ifndef DEBUG
#define DEBUG
#endif


#ifdef DEBUG
#define DEBUG_PRINT(...) { printf(__VA_ARGS__);}

#define DEBUG_TRIGGER()                                                            \
  {                                                                                \
    HAL_GPIO_WritePin(DEBUG_TRIGGER_GPIO_Port, DEBUG_TRIGGER_Pin, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(DEBUG_TRIGGER_GPIO_Port, DEBUG_TRIGGER_Pin, GPIO_PIN_RESET); \
  }
#else
#define DEBUG_PRINT(...) {}
#define DEBUG_TRIGGER()

#endif

#define BIT(n) (1UL << (n))


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
