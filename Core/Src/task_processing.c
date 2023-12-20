/*
 * task_Processing.c
 *
 *  Created on: Dec 5, 2023
 *      Author: amosm
 */

#include "common_task_defs.h"
#include "FreeRTOS.h"
#include "math.h"
#include "main.h"

#define QUEUE_WAIT_TIME (1000 / OUTPUT_RATE)

double get_energy(double velocity, double height)
{
	const double g = 9.81;
	return (0.5 / g) * velocity * velocity + height;
}

double get_ground_speed(double horizontal_speed, double d_height, double d_time)
{
	double vertical_speed = d_height / d_time; // TODO check wether to use this approach or one based on imu or pressure data
	return sqrt(horizontal_speed * horizontal_speed + vertical_speed * vertical_speed);
}

double get_air_density(double temperature_c, double pressure) // https://en.wikipedia.org/wiki/Ideal_gas_law#Molar_form
{
	const double R = 287.058;
	const double T_0 = 273.15;

	double T = temperature_c + T_0;
	return pressure / (R * T);
}

double get_air_speed(double temperature_c, double abs_pressure, double diff_pressure) // https://en.wikipedia.org/wiki/Bernoulli%27s_principle#Compressible_flow_in_fluid_dynamics
{
	const double gamma = 1.4;
	double rho = get_air_density(temperature_c, abs_pressure);
	return sqrt(2 * (gamma) / (gamma - 1) * diff_pressure / rho);
}

void processing_task(void *pvParameters)
{

	osEventFlagsWait(init_events, ev_init_all, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

	BaseType_t was_delayed = {0};
	PTH_data_t pth_data = {0};
	imu_data_t imu_data = {0};
	gps_data_t gps_data = {0};

	uint32_t lastcalc = osKernelGetTickCount();
	uint32_t time_ms = 0;
	uint32_t last_time_ms = 0;
	double last_altitude = 0.0;
	double last_energy = 0.0;
	double init_altitude = 0.0;

	TickType_t ticks = osKernelGetTickCount();

	printf("all inited \n");
	uint32_t ctr = 0;
	while (1)
	{
		// osSemaphoreRelease(pth_timing_semaphore_handle);
		// osSemaphoreRelease(imu_timing_semaphore_handle);

		osEventFlagsSet(timing_events, ev_rcv_pth | ev_rcv_imu);

		osMessageQueueGet(pth_data_queue_handle, &pth_data, 0, OUTPUT_RATE / 2);
		osMessageQueueGet(imu_data_queue_handle, &imu_data, 0, 0);
		osMessageQueueGet(gps_data_queue_handle, &gps_data, 0, 0);
		was_delayed = xTaskDelayUntil(&ticks, 1000 / OUTPUT_RATE / 2); // TODO: could go into sleep here

		if (was_delayed == pdFALSE)
		{
			printf("Measurements took too long \n");
		}

		time_ms = gps_data.time + (osKernelGetTickCount() - gps_data.ticks);
		if (last_time_ms == 0)
		{
			last_time_ms = time_ms;
			osDelay(5);
			continue;
		}

		double d_time = (time_ms - last_time_ms) / 1000.0;
		double d_altitude = gps_data.altitude - last_altitude;
		double ground_speed = get_ground_speed(gps_data.ground_speed, d_altitude, d_time);
		double air_speed = get_air_speed(pth_data.temperature, pth_data.pressure, 50);
		double energy = get_energy(ground_speed, gps_data.altitude - init_altitude);
		double d_energy = energy - last_energy;
		if (last_time_ms != 0)
		{
			rpi_tx_data_t rpi_data =
				{
					.time_ms = time_ms,
					.hum = pth_data.humidity,
					.v_ground = ground_speed,
					.v_air = air_speed,
					.temp = pth_data.temperature,
					.press = pth_data.pressure,
					.longt = gps_data.longitude,
					.lat = gps_data.latitude,
					.energy = d_energy,
					.yaw = imu_data.yaw,
					.pitch = imu_data.pitch,
					.roll = imu_data.roll

				};
			printf("ctr %d \n", ctr++);
			osMessageQueuePut(rpi_tx_queue_handle, &rpi_data, 0, QUEUE_WAIT_TIME);
			csv_dump_data_t csv_data =
				{
					.time_ms = time_ms,// (ctr >= 10000) ? (uint32_t)-1 : time_ms, TODO remove
					.hum = pth_data.humidity,
					.v_ground = ground_speed,
					.v_air = air_speed,
					.temp = pth_data.temperature,
					.press = pth_data.pressure,
					.longt = gps_data.longitude,
					.lat = gps_data.latitude,
					.energy = d_energy,
					.yaw = imu_data.yaw,
					.pitch = imu_data.pitch,
					.roll = imu_data.roll};
			osMessageQueuePut(csv_queue_handle, &csv_data, 0, 0);
		}
		last_energy = energy;
		last_altitude = gps_data.altitude;
		last_time_ms = time_ms;

		was_delayed = xTaskDelayUntil(&ticks, 1000 / OUTPUT_RATE / 2);
		if (was_delayed == pdFALSE)
		{
			printf("Processing task had to wait too long \n");
		}
	}
}
