#ifndef ICM_20948_USE_DMP
#define ICM_20948_USE_DMP
#endif

#ifndef __BSD_VISIBLE
#define __BSD_VISIBLE 1
#endif
#include <math.h>

#include "stm32u575xx.h"
#include "ICM_20948/src/util/ICM_20948_C.h"

#include "spi_common.h"
#include "common_task_defs.h"
#include "main.h"

#include "task_IMU.h"
#include <stdint-gcc.h>
#include <stdio.h>

extern SPI_HandleTypeDef SPI; // from main.c

enum en_imu_events
{
	ev_data_available = BIT(0),
};
osEventFlagsId_t imu_events;

ICM_20948_Status_e imu_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e imu_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

ICM_20948_Device_t myICM;

#define SERIAL_PORT Serial

ICM_20948_Status_e imu_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	HAL_StatusTypeDef ret = SPI_write_burst_implicit(user, reg, data, len, IMU_nCS_GPIO_Port, IMU_nCS_Pin);
	return (ret == HAL_OK) ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

ICM_20948_Status_e imu_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	HAL_StatusTypeDef ret = SPI_read_burst_implicit(user, reg, buff, len, IMU_nCS_GPIO_Port, IMU_nCS_Pin);
	return (ret == HAL_OK) ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

void ValuePrint(const char *label, double value) // From ChatGPT
{
	// Extract integer and fractional parts
	int intValue = (int)value;
	int fractionalPart = (int)((value - intValue) * 1000); // Multiply by 1000 to get 3 decimal places

	// Calculate label length
	int labelLength = 0;
	while (label[labelLength] != '\0')
	{
		labelLength++;
	}

	// Calculate padding for right alignment
	int padding = labelLength >= 10 ? 0 : 10 - labelLength;

	// Print with right-aligned label and 3 decimal places
	printf("%*s: %d.%03d\n", padding + labelLength, label, intValue, fractionalPart);
}

typedef struct
{
	int32_t Q1;
	int32_t Q2;
	int32_t Q3;
	int16_t Accuracy;
} imu_raw_data_t;

double validateAndSet(double num, double lowerBound, double upperBound, double defaultValue)
{ // from chatgpt
	// Check if the number is a valid number (not NaN or infinity)
	if (num != num || num * 0 != 0)
	{
		return defaultValue;
	}

	if (num < lowerBound || num > upperBound)
	{
		return defaultValue;
	}
	return num;
}

void quat_to_ypr(imu_raw_data_t *quats, imu_data_t *data)
{
	double q1 = ((double)quats->Q1) / 1073741824.0; // Convert to double. Divide by 2^30
	double q2 = ((double)quats->Q2) / 1073741824.0; // Convert to double. Divide by 2^30
	double q3 = ((double)quats->Q3) / 1073741824.0; // Convert to double. Divide by 2^30
	double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

	double q2sqr = q2 * q2;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q0 * q1 + q2 * q3);
	double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
	data->roll = validateAndSet(atan2(t0, t1), -M_PI, M_PI, M_PI);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q0 * q2 - q3 * q1);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	data->pitch = validateAndSet(asin(t2), -M_PI / 2.0, M_PI / 2.0, 0.0);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q0 * q3 + q1 * q2);
	double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
	data->yaw = validateAndSet(atan2(t3, t4), -M_PI, M_PI, 0.0);
}

void imu_data_available_irq_handler(void)
{
	osEventFlagsSet(imu_events, ev_data_available);
	printf("Data available\n");
}

void IMU_task(void *pvParameters)
{
	ICM_20948_Device_t myICM;
	imu_events = osEventFlagsNew(NULL);

	const ICM_20948_Serif_t mySerif = {
		imu_write_spi, // write
		imu_read_spi,  // read
		&SPI,		   // this pointer is passed into your functions when they are called.
	};

	while (1)
	{
		ICM_20948_Status_e ret = ICM_20948_startup_default(&myICM, &mySerif, 0);
		if (ret != ICM_20948_Stat_Ok)
		{
			printf("default startupfailed. Halting...\n");
			delay_ms(1000);
			continue;
		}

		ret = ICM_20948_initialize_DMP(&myICM);
		// Currently fails while verifying the start of each bank...
		if (ret != ICM_20948_Stat_Ok)
		{
			printf("DMP Initialization failed. Halting...\n");
			delay_ms(1000);
			continue;
		}
		else
		{
			break;
		}
	}
	ICM_20948_low_power(&myICM, false); // Put chip into low power state

	inv_icm20948_enable_dmp_sensor(&myICM, INV_ICM20948_SENSOR_ORIENTATION, 1);

	// Configuring DMP to output data at multiple ODRs:
	// DMP is capable of outputting multiple sensor data at different rates to FIFO.
	// Setting value can be calculated as follows:
	// Value = (DMP running rate / ODR ) - 1
	// E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
	inv_icm20948_set_dmp_sensor_period(&myICM, DMP_ODR_Reg_Quat9, 0);

	ICM_20948_enable_FIFO(&myICM, true);
	ICM_20948_enable_DMP(&myICM, true);
	ICM_20948_reset_DMP(&myICM);
	ICM_20948_reset_FIFO(&myICM);
	osEventFlagsSet(init_events, ev_init_imu);

	while (1)
	{

		osEventFlagsWait(timing_events, ev_rcv_imu, NULL, osWaitForever);

		icm_20948_DMP_data_t fifo_buf;

		ICM_20948_Status_e status = ICM_20948_Stat_FIFOMoreDataAvail;
		while (status != ICM_20948_Stat_Ok)
		{
			status = inv_icm20948_read_dmp_data(&myICM, &fifo_buf);
		}

		if ((status == ICM_20948_Stat_Ok) && (fifo_buf.header & DMP_header_bitmap_Quat9) > 0) // Was valid data available?
		{
			imu_data_t data;
			quat_to_ypr(&fifo_buf.Quat9.Data, &data);
			osMessageQueuePut(imu_data_queue_handle, &data, 0, 0);
		}
		else
		{
			printf("Something failed in DMP\r\n");
		}
	}
}
