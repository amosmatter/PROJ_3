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
#include <stdint.h>

extern osThreadId_t PTH_TaskHandle;
extern osThreadId_t IMU_TaskHandle;
extern osThreadId_t GPS_TaskHandle;
extern osThreadId_t SD_TaskHandle;
extern osThreadId_t AS_TaskHandle;
extern osThreadId_t rpi_comm_TaskHandle;

extern osThreadId_t proc_TaskHandle;

extern osMessageQueueId_t imu_data_queue_handle;
extern osMessageQueueId_t gps_data_queue_handle;
extern osMessageQueueId_t pth_data_queue_handle;
extern osMessageQueueId_t airsp_data_queue_handle;
extern osMessageQueueId_t rpi_tx_queue_handle;
extern osMessageQueueId_t csv_queue_handle;

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


typedef struct
{
  uint32_t time;
  uint32_t ticks;
  double altitude;
  double latitude;
  double longitude;
  double ground_speed;
} gps_data_t;


#define GPS_DATA_SIZE (sizeof(gps_data_t))
#define GPS_DATA_QUEUE_SIZE (1)

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


typedef struct 
{
  uint32_t time_ms;     
  double hum;      // humidity in 0.002 %
  double v_ground; // ground speed in 0.005 m/s
  double v_air;     // air speed in 0.01 m/s
  double temp;    // temperature in 0.005 C
  double press;   // pressure  in 5 Pa
  double longt;
  double lat;
  double alt_rel_start; // altitude relative to start of flight in 0.125 m
  double roll;          // roll in 0.0002 rad
  double pitch;         // pitch in 0.0002 rad
  double yaw;           // yaw in 0.0002 rad
  double energy;      // energy in 0.25 m
} rpi_tx_data_t;

#define RPI_TX_DATA_SIZE (sizeof(rpi_tx_data_t))
#define RPI_TX_DATA_QUEUE_SIZE (1)

typedef struct 
{
  uint32_t time_ms;     
  double hum;      // humidity in 0.002 %
  double v_ground; // ground speed in 0.005 m/s
  double v_air;     // air speed in 0.01 m/s
  double temp;    // temperature in 0.005 C
  double press;   // pressure  in 5 Pa
  double longt;
  double lat;
  double alt_rel_start; // altitude relative to start of flight in 0.125 m
  double roll;          // roll in 0.0002 rad
  double pitch;         // pitch in 0.0002 rad
  double yaw;           // yaw in 0.0002 rad
  double energy;      // energy in 0.25 m
} csv_dump_data_t;

#define CSV_DUMP_DATA_SIZE (sizeof(csv_dump_data_t))
#define CSV_DUMP_DATA_QUEUE_SIZE (10)



#endif /* INC_COMMON_TASK_DEFS_H_ */
