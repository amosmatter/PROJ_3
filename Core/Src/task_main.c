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
osMutexId_t SPI_Task_Mutex;

osEventFlagsId_t init_events;
osEventFlagsId_t timing_events;
osEventFlagsId_t general_events;

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PS_nFLT_Pin)
    {
        osEventFlagsSet(general_events, ev_power_ok);
        osEventFlagsClear(general_events, ev_fault_detected);
        DEBUG_PRINT("Power ok%d\n", HAL_GPIO_ReadPin(PS_nFLT_GPIO_Port, PS_nFLT_Pin));
    }
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PS_nFLT_Pin)
    {
        osEventFlagsSet(general_events, ev_fault_detected);
        osEventFlagsClear(general_events, ev_power_ok);

        osThreadSuspend(GPS_TaskHandle);
        osThreadSuspend(AS_TaskHandle);
        osThreadSuspend(proc_TaskHandle);
        //DEBUG_PRINT("Fault detected%d\n", HAL_GPIO_ReadPin(PS_nFLT_GPIO_Port, PS_nFLT_Pin));
    }
    else if (GPIO_Pin == SW_ACTIVE_Pin)
    {
        osEventFlagsSet(general_events, ev_button_pressed);
        HAL_NVIC_SystemReset(); // TODO remove
    }
}

void main_task(void *pvParameters)
{
    general_events = osEventFlagsNew(NULL);
    if (HAL_GPIO_ReadPin(PS_nFLT_GPIO_Port, PS_nFLT_Pin) == GPIO_PIN_SET)
    {
        osEventFlagsSet(general_events, ev_power_ok);
    }
    osEventFlagsWait(general_events, ev_power_ok, osFlagsNoClear, osWaitForever);

    init_time();
    SPI_Lock = osMutexNew(NULL);
    SPI_Task_Mutex = osMutexNew(NULL);

    init_events = osEventFlagsNew(NULL);
    timing_events = osEventFlagsNew(NULL);

    csv_queue_handle = osMessageQueueNew(CSV_DUMP_DATA_QUEUE_SIZE, CSV_DUMP_DATA_SIZE, NULL);
    rpi_tx_queue_handle = osMessageQueueNew(RPI_TX_DATA_QUEUE_SIZE, RPI_TX_DATA_SIZE, NULL);
    pth_data_queue_handle = osMessageQueueNew(PTH_DATA_QUEUE_SIZE, PTH_DATA_SIZE, NULL);
    imu_data_queue_handle = osMessageQueueNew(IMU_DATA_QUEUE_SIZE, IMU_DATA_SIZE, NULL);
    airsp_data_queue_handle = osMessageQueueNew(AIRSPEED_DATA_QUEUE_SIZE, AIRSPEED_DATA_SIZE, NULL);
    gps_data_queue_handle = osMessageQueueNew(GPS_DATA_QUEUE_SIZE, GPS_DATA_SIZE, NULL);

    PTH_TaskHandle = osThreadNew(PTH_task, NULL, &PTH_TaskAttributes);
    proc_TaskHandle = osThreadNew(processing_task, NULL, &proc_TaskAttributes);
    IMU_TaskHandle = osThreadNew(IMU_task, NULL, &IMU_TaskAttributes);
    GPS_TaskHandle = osThreadNew(GPS_task, NULL, &GPS_TaskAttributes);
    rpi_comm_TaskHandle = osThreadNew(comm_rpi_task, NULL, &SD_TaskAttributes);
    AS_TaskHandle = osThreadNew(airspeed_task, NULL, &AS_TaskAttributes);

    SD_TaskHandle = osThreadNew(SD_task, NULL, &SD_TaskAttributes);

    for (;;)
    {
        uint32_t ret = osEventFlagsWait(init_events, ev_init_all, osFlagsWaitAll | osFlagsNoClear, 1000);
        HAL_GPIO_TogglePin(LED_READY_GPIO_Port, LED_READY_Pin);

        if (  0 >=* (int32_t * ) &ret)
        {
        	break;
        }

    }


    for(;;)
    {

    	HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
    	osDelay(1995);
    	HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 1);
    	osDelay(5);
    }



}
