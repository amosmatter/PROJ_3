#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os2.h"

#include "main.h"
#include "BME280/bme280.h"

#include "system_time.h"
#include "spi_common.h"
#include "common_task_defs.h"
#include "task_PTH.h"
extern SPI_HandleTypeDef SPI; // from main.c

/**
 * Wrappers for the library
 *
 */
BME280_INTF_RET_TYPE bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    HAL_StatusTypeDef ret = SPI_read_burst_implicit(intf_ptr, reg_addr, reg_data, length, PTH_nCS_GPIO_Port, PTH_nCS_Pin);
    return (ret == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

BME280_INTF_RET_TYPE bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    HAL_StatusTypeDef ret = SPI_write_burst_explicit(intf_ptr, reg_addr, reg_data, length, PTH_nCS_GPIO_Port, PTH_nCS_Pin);
    return (ret == HAL_OK) ? BME280_OK : BME280_E_COMM_FAIL;
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
#ifdef DEBUG

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
#endif
}

static int8_t get_pth_data(struct bme280_dev *dev, struct bme280_data *comp_data)
{
    int8_t rslt = BME280_E_NULL_PTR;
    uint8_t status_reg;
    while (1)
    {

        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_HUM | BME280_PRESS | BME280_TEMP, comp_data, dev);

            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);
            return rslt;
        }
        delay_ms(1);
    }
}

void PTH_task(void *pvParameters)
{
    int8_t rslt;
    uint32_t period;
    struct bme280_dev dev;
    struct bme280_settings settings;
    struct bme280_data comp;

    dev.read = bme280_spi_read;
    dev.write = bme280_spi_write;
    dev.intf = BME280_SPI_INTF;
    dev.intf_ptr = &SPI;
    dev.delay_us = bme280_delay_us;

	osMutexAcquire(SPI_Task_Mutex,osWaitForever);

    rslt = bme280_init(&dev);
    bme280_error_codes_print_result("bme280_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&settings, &dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    // IIR Filter coefficient and measurement standby period to change power consumption and
    // step response time
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

    DEBUG_PRINT("\nHumidity calculation (Data displayed are compensated values)\r\n");
    DEBUG_PRINT("Measurement time : %lu us\r\n\n", (long unsigned int)period);

    if (period / 1000 + 20 > 1000 / OUTPUT_RATE)
    {
        DEBUG_PRINT("Output Rate too high for PTH settings!!");
        osThreadTerminate(NULL);
        return;
    }

    osEventFlagsSet(init_events, ev_init_pth);
    osMutexRelease(SPI_Task_Mutex);

	osEventFlagsWait(init_events, ev_init_all, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

    while (1)
    {
        osEventFlagsWait(timing_events, ev_rcv_pth, osFlagsWaitAll, osWaitForever);
        
        osMutexAcquire(SPI_Task_Mutex,osWaitForever);
        rslt = get_pth_data(&dev, &comp);
        osMutexRelease(SPI_Task_Mutex);

        if (rslt != BME280_OK)
        {
            bme280_error_codes_print_result("get_pth_data", rslt);
        }
        else
        {
            osMessageQueuePut(pth_data_queue_handle, &comp, 0, 0);
        }
    }
}
