#include "task_airspeed.h"
#include "main.h"
#include "cmsis_os2.h"
#include "common_task_defs.h"
#include <stdio.h>
#include "stm32u5xx_hal_i2c.h"
#define ADDRESS (0x28 << 1)

#define P_MAX (1)
#define P_MIN (-1)

#define P_SCALE ((1 << 14) - 1)
#define P_OFFS (0.1 * P_SCALE)

#define PSI_TO_PASCAL (6894.7572931783)

#define T_MAX (150)
#define T_MIN (-50)

#define T_SCALE ((1 << 11) - 1)
#define T_OFFS (0)

#define AIRSPEED_N_RETRIES 1 // maybe set higher if works properly

#define ERR_NO_SENSOR 5

typedef struct
{
    uint16_t bridge;
    uint16_t temp;
    uint8_t flags;
} raw_airspeed_data_t;

extern I2C_HandleTypeDef I2C;

HAL_StatusTypeDef initiate_measurement(uint16_t add)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Receive(&I2C, add, NULL, 0, 50);
    return ret;
}
HAL_StatusTypeDef initiate_measurement2(uint16_t add)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(&I2C, add, NULL, 0, 50);
    return ret;
}


enum e_i2cEvents
{
		ev_got_data = BIT(0),
		ev_got_trash = BIT(1),
		ev_not_ack = BIT(2),
};
osEventFlagsId_t i2cFlags = NULL;
void airspeed_recieve_handler(void)
{
	uint32_t ISR = I2C.Instance->ISR;

	if (ISR &I2C_ISR_NACKF)
	{
    	osEventFlagsSet(i2cFlags, ev_not_ack);
	}
	else if (ISR & I2C_ISR_TC)
	{
		osEventFlagsSet(i2cFlags,ev_got_data);
	}
	else{
		osEventFlagsSet(i2cFlags,ev_got_trash);

	}
}

HAL_StatusTypeDef get_data(raw_airspeed_data_t *raw_data)
{
    uint8_t buf[4];
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Receive(&I2C, ADDRESS, buf, 4, 100);
    if (ret != HAL_OK)
    {
        raw_data->flags = 3;
        return ret;
    }

    raw_data->bridge = ((uint16_t)(buf[0]& 0x3F ) << 8) | buf[1];
    raw_data->temp = ((uint16_t)(buf[2]) << 8) | (buf[3]  );
    raw_data->flags = buf[0] >> 6;
    return HAL_OK;
}



HAL_StatusTypeDef get_data_it(raw_airspeed_data_t *raw_data) // Haven't tested this version but would arguably be more efficient
{
    uint8_t buf[4];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Receive_IT(&I2C, ADDRESS, buf, 4);
    if (ret != HAL_OK)
    {
        raw_data->flags = 3;
        return ret;
    }
    uint32_t flgs = osEventFlagsWait(i2cFlags, ev_got_data | ev_got_trash | ev_not_ack, 0, 50);
    if (flgs & ev_not_ack)
    {
    	return ERR_NO_SENSOR;
    }

    if (flgs & (BIT(31) | ev_got_trash ))
    {
    	return HAL_ERROR;
    }

    raw_data->bridge = ((uint16_t)(buf[0]& 0x3F ) << 8) | buf[1];
    raw_data->temp = ((uint16_t)(buf[2]) << 8) | (buf[3]  );
    raw_data->flags = buf[0] >> 6;
    return HAL_OK;
}

double get_pressure_in_psi(raw_airspeed_data_t *raw_data)
{
    return (raw_data->bridge - P_OFFS) * ((P_MAX - P_MIN) / (P_SCALE - 2.0 * P_OFFS)) + P_MIN;
}

double get_pressure_pascals(raw_airspeed_data_t *raw_data)
{
    return get_pressure_in_psi(raw_data) * PSI_TO_PASCAL;
}

double get_temperature_c(raw_airspeed_data_t *raw_data)
{
    return ((raw_data->temp - T_OFFS) * ((T_MAX - T_MIN) / (T_SCALE - 2.0 * T_OFFS)) + T_MIN) / 100.0;
}


void airspeed_task(void *pvParameters)
{
    raw_airspeed_data_t raw_data;
    airspeed_data_t data ;
    i2cFlags = osEventFlagsNew(NULL);
    osEventFlagsSet(init_events, ev_init_airsp);

    while (1)
    {
        osEventFlagsWait(timing_events, ev_rcv_airsp, osFlagsWaitAll, osWaitForever);
        HAL_StatusTypeDef ret = get_data(&raw_data);
        if (ret == ERR_NO_SENSOR)// request measurement
        {
        	DEBUG_PRINT("No airspeed sensor connected!\n");
        	continue;
        }
        for (int i = 0; i < AIRSPEED_N_RETRIES; i++)
        {
            osDelay(50);
            ret = get_data(&raw_data); // get measurement
            if (ret != HAL_OK)
            {
                DEBUG_PRINT("AIRSPEED Comm error! \n");
                continue;
            }
            else if (raw_data.flags & 3 == 2)
            {
                DEBUG_PRINT("AIRSPEED Stale Data! \n");
                continue;
            }
            else if (raw_data.flags & 3 == 3)
            {
                DEBUG_PRINT("Fault detected!\n");
                continue;
            }
            data.d_pressure = get_pressure_pascals(&raw_data);
            data.temp = get_temperature_c(&raw_data);
            osMessageQueuePut(airsp_data_queue_handle, &data, 0, 0);
            break;
        }

    }
}
