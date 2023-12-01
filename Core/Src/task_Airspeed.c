#include "task_Airspeed.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdio.h>

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

typedef struct
{
    uint16_t bridge;
    uint16_t temp;
    uint8_t flags;
} airspeed_data_t;

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

HAL_StatusTypeDef get_data(airspeed_data_t *data)
{
    uint8_t buf[4];
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Receive(&I2C, ADDRESS, buf, 4, 100);
    if (ret != HAL_OK)
    {
        data->flags = 3;
        return ret;
    }

    data->bridge = ((uint16_t)buf[0] << 8) | buf[1];
    data->temp = ((uint16_t)(buf[2] & 0x3F) << 8) | buf[3];
    data->flags = buf[2] >> 6;
    return HAL_OK;
}

double get_pressure_in_psi(airspeed_data_t *data)
{
    return (data->bridge - P_OFFS) * ((P_MAX - P_MIN) / (P_SCALE - 2.0 * P_OFFS)) + P_MIN;
}

double get_pressure_pascals(airspeed_data_t *data)
{
    return get_pressure_in_psi(data) * PSI_TO_PASCAL;
}

double get_temperature_c(airspeed_data_t *data)
{
    return ((data->temp - T_OFFS) * ((T_MAX - T_MIN) / (T_SCALE - 2.0 * T_OFFS)) + T_MIN) / 100.0;
}

void airspeed_task(void *pvParameters)
{
    airspeed_data_t data;

    while (1)
    {
        osDelay(1);
        HAL_StatusTypeDef ret =  get_data(&data);
        if (ret != HAL_OK)
        {
            DEBUG_PRINT("Comm error! \n");
            continue;
        }
        else if (data->flags & 3 == 2)
        {
            DEBUG_PRINT("Stale Data! \n");
            continue;
        }
        else if (data->flags & 3 == 3)
        {
            DEBUG_PRINT("Fault detected!\n");
            continue;
        }
        printf("Bridge: %f, Temp: %f\n", get_pressure_pascals(&data), get_temperature_c(&data));
    }
}
