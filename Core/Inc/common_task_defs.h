/*
 * common_task_defs.h
 *
 *  Created on: Dec 1, 2023
 *      Author: amosm
 */

#ifndef INC_COMMON_TASK_DEFS_H_
#define INC_COMMON_TASK_DEFS_H_

#include "cmsis_os2.h"

extern osThreadId_t PTH_TaskHandle;
extern osThreadId_t IMU_TaskHandle;
extern osThreadId_t GPS_TaskHandle;
extern osThreadId_t SD_TaskHandle;
extern osThreadId_t AS_TaskHandle;

extern osMessageQueueId_t imu_data_queue_handle;
extern osMessageQueueId_t gps_data_queue_handle;
extern osMessageQueueId_t pth_data_queue_handle;
extern osMessageQueueId_t airsp_data_queue_handle;
extern osMessageQueueId_t airsp_data_queue_handle;

extern osMutexId_t SPI_Lock;

typedef struct
{

} imu_data_t;
 
typedef struct
{

} gps_data_t;

#define IMU_DATA_SIZE (sizeof(imu_data_t))
#define IMU_DATA_QUEUE_SIZE (10)

#define GPS_DATA_SIZE (sizeof(gps_data_t))
#define GPS_DATA_QUEUE_SIZE (10)


#endif /* INC_COMMON_TASK_DEFS_H_ */
