/*
 * my_main.c
 *
 *  Created on: Nov 16, 2023
 *      Author: amosm
 */
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os2.h"

#include "main.h"
#include "BME280/bme280.h"

#include "delay.h"
#include "spi_common.h"
extern SPI_HandleTypeDef SPI; // from main.c

#define SAMPLE_COUNT  UINT8_C(50)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;


/**
 * Wrappers for the library
 *
 */
BME280_INTF_RET_TYPE bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	 HAL_StatusTypeDef ret = SPI_read_burst_implicit(intf_ptr, reg_addr,reg_data,length,PTH_nCS_GPIO_Port,PTH_nCS_Pin);
	 return (ret == HAL_OK)? BME280_OK: BME280_E_COMM_FAIL;
}

BME280_INTF_RET_TYPE bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	 HAL_StatusTypeDef ret = SPI_write_burst_explicit(intf_ptr, reg_addr,reg_data,length,PTH_nCS_GPIO_Port,PTH_nCS_Pin);
	 return (ret == HAL_OK)? BME280_OK: BME280_E_COMM_FAIL;
}

void bme280_delay_us(uint32_t period, void *intf_ptr)
{
	delay_us(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
        case BME280_E_NULL_PTR:
            printf("Error [%d] : Null pointer error.", rslt);
            printf(
                "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
            break;

        case BME280_E_COMM_FAIL:
            printf("Error [%d] : Communication failure error.", rslt);
            printf(
                "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
            break;

        case BME280_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BME280_E_INVALID_LEN:
            printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
        }
    }
}

void init_bme280(struct bme280_dev *dev, SPI_HandleTypeDef *hspi)
{

}

static int8_t get_humidity(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;



    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Humidity[%d]:   %lf %%RH\n", idx, comp_data.humidity);
#else
            printf("Humidity[%d]:   %lu %%RH\n", idx, (long unsigned int)comp_data.humidity);
#endif
            idx++;
        }
    }

    return rslt;
}


void PTH_task(void *pvParameters)
{

        int8_t rslt;
        uint32_t period;
        struct bme280_dev dev;
        struct bme280_settings settings;


        dev.read = bme280_spi_read;
        dev.write = bme280_spi_write;
        dev.intf = BME280_SPI_INTF;
        dev.intf_ptr = &SPI;
        dev.delay_us = bme280_delay_us;

        rslt = bme280_init(&dev);
        bme280_error_codes_print_result("bme280_init", rslt);

        /* Always read the current settings before writing, especially when all the configuration is not modified */
        rslt = bme280_get_sensor_settings(&settings, &dev);
        bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

        /* Configuring the over-sampling rate, filter coefficient and standby time */
        /* Overwrite the desired settings */
        settings.filter = BME280_FILTER_COEFF_2;

        /* Over-sampling rate for humidity, temperature and pressure */
        settings.osr_h = BME280_OVERSAMPLING_1X;
        settings.osr_p = BME280_OVERSAMPLING_1X;
        settings.osr_t = BME280_OVERSAMPLING_1X;

        /* Setting the standby time */
        settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

        rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
        bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

        /* Always set the power mode after setting the configuration */
        rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
        bme280_error_codes_print_result("bme280_set_power_mode", rslt);

        /* Calculate measurement time in microseconds */
        rslt = bme280_cal_meas_delay(&period, &settings);
        bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

        printf("\nHumidity calculation (Data displayed are compensated values)\r\n");
        printf("Measurement time : %lu us\r\n\n", (long unsigned int)period);


        rslt = get_humidity(period, &dev);
        bme280_error_codes_print_result("get_humidity", rslt);

        while(1)
        {
    	osDelay(100);
        }


}


