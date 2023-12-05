/*
 * common_task_defs.h
 *
 *  Created on: Dec 1, 2023
 *      Author: amosm
 */

#ifndef INC_COMMON_TASK_DEFS_H_
#define INC_COMMON_TASK_DEFS_H_

#include "cmsis_os2.h"
#include "task_PTH.h"

extern osThreadId_t PTH_TaskHandle;
extern osThreadId_t IMU_TaskHandle;
extern osThreadId_t GPS_TaskHandle;
extern osThreadId_t SD_TaskHandle;
extern osThreadId_t AS_TaskHandle;

extern osThreadId_t proc_TaskHandle;

extern osMessageQueueId_t imu_data_queue_handle;
extern osMessageQueueId_t gps_data_queue_handle;
extern osMessageQueueId_t pth_data_queue_handle;
extern osMessageQueueId_t airsp_data_queue_handle;

extern osMutexId_t SPI_Lock;


extern osSemaphoreId_t imu_timing_semaphore_handle;
extern osSemaphoreId_t gps_timing_semaphore_handle;
extern osSemaphoreId_t pth_timing_semaphore_handle;
extern osSemaphoreId_t airsp_timing_semaphore_handle;



typedef struct
{
  double yaw;
  double pitch;
  double roll;
} imu_data_t;
#define IMU_DATA_SIZE (sizeof(imu_data_t))
#define IMU_DATA_QUEUE_SIZE (1)

#define GPS_DATA_SIZE (sizeof(gps_data_t))
#define GPS_DATA_QUEUE_SIZE (10)

#define GPS_DATA_TYPE (sizeof(bme280_data))

typedef struct bme280_data PTH_data_t;

#define PTH_DATA_SIZE (sizeof(PTH_data_t))
#define PTH_DATA_QUEUE_SIZE (1)

typedef struct
{
    uint32_t d_pressure;
    uint32_t temp;
} airspeed_data_t;

#define AIRSPEED_DATA_SIZE (sizeof(airspeed_data_t))
#define AIRSPEED_DATA_QUEUE_SIZE (1)

#endif /* INC_COMMON_TASK_DEFS_H_ */
