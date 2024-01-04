#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdio.h>

#include "common_task_defs.h"
#include "task_PTH.h"
#include "task_IMU.h"
#include "task_GPS.h"
#include "task_SD.h"
#include "task_airspeed.h"
#include "task_processing.h"
#include "task_comm_rpi.h"
#include "system_time.h"

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
osEventFlagsId_t general_events;

void buttonpressed(void)
{
    osEventFlagsSet(general_events, ev_button_pressed);
    NVIC_SystemReset();
}

void main_task(void *pvParameters)
{
    init_time();
    SPI_Lock = osMutexNew(NULL);

    init_events = osEventFlagsNew(NULL);
    timing_events = osEventFlagsNew(NULL);
    general_events = osEventFlagsNew(NULL);

    csv_queue_handle = osMessageQueueNew(CSV_DUMP_DATA_QUEUE_SIZE, CSV_DUMP_DATA_SIZE, NULL);
    rpi_tx_queue_handle = osMessageQueueNew(RPI_TX_DATA_QUEUE_SIZE, RPI_TX_DATA_SIZE, NULL);
    pth_data_queue_handle = osMessageQueueNew(PTH_DATA_QUEUE_SIZE, PTH_DATA_SIZE, NULL);
    imu_data_queue_handle = osMessageQueueNew(IMU_DATA_QUEUE_SIZE, IMU_DATA_SIZE, NULL);
    airsp_data_queue_handle = osMessageQueueNew(AIRSPEED_DATA_QUEUE_SIZE, AIRSPEED_DATA_SIZE, NULL);
    gps_data_queue_handle = osMessageQueueNew(GPS_DATA_QUEUE_SIZE, GPS_DATA_SIZE, NULL);

    // PTH_TaskHandle = osThreadNew(PTH_task, NULL, &PTH_TaskAttributes);
    // proc_TaskHandle = osThreadNew(processing_task, NULL, &proc_TaskAttributes);
    // IMU_TaskHandle = osThreadNew(IMU_task, NULL, &IMU_TaskAttributes);
    // AS_TaskHandle = osThreadNew(airspeed_task, NULL, &AS_TaskAttributes);
    // GPS_TaskHandle = osThreadNew(GPS_task, NULL, &GPS_TaskAttributes);
    // rpi_comm_TaskHandle = osThreadNew(comm_rpi_task, NULL, &SD_TaskAttributes);
    SD_TaskHandle = osThreadNew(SD_task, NULL, &SD_TaskAttributes);

    for (;;)
    {
        osDelay(1000);
        HAL_GPIO_TogglePin(LED_READY_GPIO_Port, LED_READY_Pin);
        printf("Hello world\r\n");

        if (!(BIT(31) & osEventFlagsWait(general_events, ev_button_pressed, osFlagsWaitAny, 0)))
        {
            printf("Button pressed\r\n");
        }
    }
}
