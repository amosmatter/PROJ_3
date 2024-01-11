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
#include "system_time.h"

#define OUTPUT_PERIOD (1000 / OUTPUT_RATE)
#define SENSOR_OVERHEAD (60)

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
	BaseType_t was_delayed = {0};
	PTH_data_t pth_data = {0};
	imu_data_t imu_data = {0};
	gps_data_t gps_data = {0};
	rpi_tx_data_t rpi_data = {0};
	airspeed_data_t airspeed_data = {0};

	csv_dump_data_t csv_data = {0};

	osEventFlagsWait(init_events, ev_init_all, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

	uint32_t lastcalc = osKernelGetTickCount();
	uint32_t time_ms = 0;
	uint32_t last_time_ms = 0;
	double last_altitude = 0.0;
	double last_energy = 0.0;
	double init_altitude = 0.0;

	TickType_t ticks = osKernelGetTickCount();
	double d_time = 1000.0 / OUTPUT_RATE;

	uint32_t ctr = 0;

	uint32_t initialized = 0;
	osMessageQueueReset(gps_data_queue_handle);
	while (1)
	{
		osEventFlagsSet(timing_events, ev_rcv_all);

		uint32_t premature = osMessageQueueGet(gps_data_queue_handle, &gps_data, 0, 0);

		if (*(int32_t * ) &premature >= 0)
		{
			DEBUG_PRINT("The GPS sent while processing task was waiting; Increase Sensor Overhead!\n");
		}


		osMessageQueueGet(gps_data_queue_handle, &gps_data, 0, SENSOR_OVERHEAD); // while waiting here, the sensors should be sending data
		osMessageQueueGet(pth_data_queue_handle, &pth_data, 0, 0);
		osMessageQueueGet(imu_data_queue_handle, &imu_data, 0, 0);
		osMessageQueueGet(airsp_data_queue_handle, &airspeed_data, 0, 0);



		if (!initialized)
		{
			last_altitude = gps_data.altitude;
			init_altitude = gps_data.altitude;
		}
		double d_altitude = gps_data.altitude - last_altitude;
		double ground_speed = get_ground_speed(gps_data.ground_speed, d_altitude, d_time);
		double air_speed = get_air_speed(airspeed_data.temp, pth_data.pressure, airspeed_data.d_pressure);
		double energy = get_energy(ground_speed, gps_data.altitude - init_altitude);

		if (!initialized)
		{
			last_energy = energy;
			initialized = 1;
		}

		double d_energy = energy - last_energy;

		rpi_data.hum = pth_data.humidity;
		rpi_data.v_ground = ground_speed;
		rpi_data.v_air = air_speed;
		rpi_data.temp = pth_data.temperature;
		rpi_data.press = pth_data.pressure;
		rpi_data.longt = gps_data.longitude;
		rpi_data.lat = gps_data.latitude;
		rpi_data.energy = d_energy;
		rpi_data.yaw = imu_data.yaw;
		rpi_data.pitch = imu_data.pitch;
		rpi_data.roll = imu_data.roll;
		osMessageQueuePut(rpi_tx_queue_handle, &rpi_data, 0, 0);

		csv_data.hum = pth_data.humidity;
		csv_data.v_ground = ground_speed;
		csv_data.v_air = air_speed;
		csv_data.temp = pth_data.temperature;
		csv_data.press = pth_data.pressure;
		csv_data.longt = gps_data.longitude;
		csv_data.lat = gps_data.latitude;
		csv_data.energy = d_energy;
		csv_data.yaw = imu_data.yaw;
		csv_data.pitch = imu_data.pitch;
		csv_data.roll = imu_data.roll;
		osMessageQueuePut(csv_queue_handle, &csv_data, 0, 0);

		last_energy = energy;
		last_altitude = gps_data.altitude;

		osDelay(OUTPUT_PERIOD - SENSOR_OVERHEAD);

	}
}
