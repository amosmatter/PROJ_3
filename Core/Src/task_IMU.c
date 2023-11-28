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

#include "main.h"

#include "task_IMU.h"
#include <stdint-gcc.h>
#include <stdio.h>

extern SPI_HandleTypeDef SPI; // from main.c

ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

ICM_20948_Device_t myICM;

#define SERIAL_PORT Serial

ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	HAL_StatusTypeDef ret = SPI_write_burst_implicit(user, reg, data, len, IMU_nCS_GPIO_Port, IMU_nCS_Pin);

	if (ret != HAL_OK)
	{
		return ICM_20948_Stat_Err;
	}

	return (ret == HAL_OK) ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	HAL_StatusTypeDef ret = SPI_read_burst_implicit(user, reg, buff, len, IMU_nCS_GPIO_Port, IMU_nCS_Pin);

	if (ret != HAL_OK)
	{
		return ICM_20948_Stat_Err;
	}

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

void IMU_task(void *pvParameters)
{
	ICM_20948_Device_t myICM;

	const ICM_20948_Serif_t mySerif = {
		my_write_spi, // write
		my_read_spi,  // read
		&SPI,		  // this pointer is passed into your functions when they are called.
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

	while (1)
	{
		icm_20948_DMP_data_t data;
		ICM_20948_Status_e status = inv_icm20948_read_dmp_data(&myICM, &data);
		if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
		{
			if ((data.header & DMP_header_bitmap_Quat9) > 0)
			{

				double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
				double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
				double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
				double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

				double q2sqr = q2 * q2;

				// roll (x-axis rotation)
				double t0 = +2.0 * (q0 * q1 + q2 * q3);
				double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
				double roll = atan2(t0, t1) * 180.0 / M_PI;

				// pitch (y-axis rotation)
				double t2 = +2.0 * (q0 * q2 - q3 * q1);
				t2 = t2 > 1.0 ? 1.0 : t2;
				t2 = t2 < -1.0 ? -1.0 : t2;
				double pitch = asin(t2) * 180.0 / M_PI;

				// yaw (z-axis rotation)
				double t3 = +2.0 * (q0 * q3 + q1 * q2);
				double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
				double yaw = atan2(t3, t4) * 180.0 / M_PI;

				ValuePrint("Roll", roll);
				ValuePrint("Pitch", pitch);
				ValuePrint("Yaw", yaw);
			}
		}
		if (status != ICM_20948_Stat_FIFOMoreDataAvail)
		{

			delay_ms(100);
		}
	}
}
