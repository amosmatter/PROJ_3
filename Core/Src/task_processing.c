/*
 * task_Processing.c
 *
 *  Created on: Dec 5, 2023
 *      Author: amosm
 */

#include "common_task_defs.h"
#include "FreeRTOS.h"

#define QUEUE_WAIT_TIME (1000 / OUTPUT_RATE)

void processing_task(void *pvParameters)
{

	BaseType_t was_delayed;
	PTH_data_t pth_data;
	imu_data_t imu_data;
	osDelay(1000);


	TickType_t ticks = osKernelGetTickCount();
	while(1)
	{
		osSemaphoreRelease(pth_timing_semaphore_handle);
		osSemaphoreRelease(imu_timing_semaphore_handle);

		osMessageQueueGet(pth_data_queue_handle, &pth_data, 0, QUEUE_WAIT_TIME );
		osMessageQueueGet(imu_data_queue_handle, &imu_data, 0, QUEUE_WAIT_TIME );


		was_delayed = xTaskDelayUntil(&ticks,1000 / OUTPUT_RATE / 2 ); // TODO: could go into sleep here
		if (was_delayed == pdFALSE)
		{
			printf("Measurements took too long \n");
		}
		printf("Yaw:        %f\n", imu_data.yaw);
		// printf("Humidity:   %f %%RH\n", comp.humidity);
		printf("Pressure:   %f Pa\n", pth_data.pressure);
                // printf("Temperature: %f °C\n", comp.temperature);

		was_delayed = xTaskDelayUntil(&ticks,1000 / OUTPUT_RATE / 2 );
		if (was_delayed == pdFALSE)
		{
			printf("Processing task had to wait too long \n");
		}
		
	}

}
