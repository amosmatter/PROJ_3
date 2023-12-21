/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "common_task_defs.h"
#include "task_PTH.h"
#include "task_IMU.h"
#include "task_GPS.h"
#include "task_SD.h"
#include "task_airspeed.h"
#include "task_processing.h"
#include "task_comm_rpi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const osThreadAttr_t PTH_TaskAttributes = {
    .name = "PTHTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32};

const osThreadAttr_t IMU_TaskAttributes = {
    .name = "IMUTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32,
};

const osThreadAttr_t GPS_TaskAttributes = {
    .name = "GPSTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32,
};

const osThreadAttr_t SD_TaskAttributes = {
    .name = "SDTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32,
};

const osThreadAttr_t AS_TaskAttributes = {
    .name = "AirspeedTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32,
};

const osThreadAttr_t proc_TaskAttributes = {
    .name = "Processing Task",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 32,
};

osThreadId_t PTH_TaskHandle;
osThreadId_t IMU_TaskHandle;
osThreadId_t GPS_TaskHandle;
osThreadId_t SD_TaskHandle;
osThreadId_t AS_TaskHandle;
osThreadId_t proc_TaskHandle;
osThreadId_t rpi_comm_TaskHandle;

osMessageQueueId_t imu_data_queue_handle;
osMessageQueueId_t gps_data_queue_handle;
osMessageQueueId_t pth_data_queue_handle;
osMessageQueueId_t airsp_data_queue_handle;
osMessageQueueId_t csv_queue_handle;
osMessageQueueId_t rpi_tx_queue_handle;

osMutexId_t SPI_Lock;

osEventFlagsId_t init_events;
osEventFlagsId_t timing_events;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  SPI_Lock = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */



  init_events = osEventFlagsNew(NULL);
  timing_events = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  csv_queue_handle = osMessageQueueNew(CSV_DUMP_DATA_QUEUE_SIZE, CSV_DUMP_DATA_SIZE, NULL);
  rpi_tx_queue_handle = osMessageQueueNew(RPI_TX_DATA_QUEUE_SIZE, RPI_TX_DATA_SIZE, NULL);
  pth_data_queue_handle = osMessageQueueNew(PTH_DATA_QUEUE_SIZE, PTH_DATA_SIZE, NULL);
  imu_data_queue_handle = osMessageQueueNew(IMU_DATA_QUEUE_SIZE, IMU_DATA_SIZE, NULL);
  airsp_data_queue_handle = osMessageQueueNew(AIRSPEED_DATA_QUEUE_SIZE, AIRSPEED_DATA_SIZE, NULL);

  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //PTH_TaskHandle = osThreadNew(PTH_task, NULL, &PTH_TaskAttributes);
  //proc_TaskHandle = osThreadNew(processing_task, NULL, &PTH_TaskAttributes);
//
  //IMU_TaskHandle = osThreadNew(IMU_task, NULL, &IMU_TaskAttributes);
  //AS_TaskHandle = osThreadNew(airspeed_task, NULL, &AS_TaskAttributes);
  GPS_TaskHandle = osThreadNew(GPS_task, NULL, &GPS_TaskAttributes);
  //rpi_comm_TaskHandle = osThreadNew(comm_rpi_task, NULL, &SD_TaskAttributes);
  //SD_TaskHandle = osThreadNew(SD_task, NULL, &SD_TaskAttributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief Function implementing the defaultTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END defaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
