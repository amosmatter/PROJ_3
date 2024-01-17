/*
 * task_comm_rpi.c
 *
 *  Created on: Dec 6, 2023
 *      Author: amosm
 */

#include "common_task_defs.h"
#include "task_comm_rpi.h"
#include "system_time.h"
#include "spi_common.h"
#include <stdio.h>
#include <stdlib.h>
#include "system_time.h"
typedef struct __packed
{
    uint32_t time;     // time in seconds, shifted left by 15
    uint16_t hum;      // humidity in 0.002 %
    uint16_t v_ground; // ground speed in 0.005 m/s
    int16_t v_air;     // air speed in 0.01 m/s
    int16_t temp;      // temperature in 0.005 C
    int16_t press;     // pressure  in 5 Pa
    uint32_t longt;
    uint32_t lat;
    int16_t alt_rel_start; // altitude relative to start of flight in 0.125 m
    int16_t roll;          // roll in 0.0002 rad
    int16_t pitch;         // pitch in 0.0002 rad
    int16_t yaw;           // yaw in 0.0002 rad
    int16_t energy;        // energy in 0.25 m
    uint16_t crc;
} rpi_tx_structured_data_t;

uint32_t convert_coord_to_fmt(double coord)
{
    int32_t scaled = (int32_t)coord;

    double fraction = fabs(coord - (double)scaled);
    uint32_t fraction_scaled = (uint32_t)(fraction * (1 << 20));

    uint32_t ret = 0;
    ret |= scaled << 22;
    ret |= ((fraction_scaled << 2) & (0b111111 << 16UL));
    ret |= (fraction_scaled & 0x3FFFUL);

    return ret;
}

#define CRC_POLY 0b1100010110011001
uint16_t calculate_crc(uint16_t *data, size_t length)// From ChatGPT
{                          
    uint16_t crc = 0; // Initial CRC value

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 16; j++)
        {
            if (crc & 0x1)
            {
                crc = (crc >> 1) ^ CRC_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
void comm_rpi_task(void *pvParameters)
{
    osEventFlagsWait(init_events, ev_init_in, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
    osEventFlagsSet(init_events, ev_init_rpi);
    rpi_tx_data_t rpi_in_data;
    while (1)
    {
        osMessageQueueGet(rpi_tx_queue_handle, &rpi_in_data, 0, osWaitForever);
        struct tm timeinfo;
        uint32_t ms = 0;
        get_time(&timeinfo, &ms);
        uint32_t sec = timeinfo.tm_sec + timeinfo.tm_min * 60 + timeinfo.tm_hour * 3600;
        sec <<= 15;

        // Alternative, if sub second accuracy is needed for example with 10Hz sampling
        // uint32_t  tim = ms + timeinfo.tm_sec * 1000 + timeinfo.tm_min * 60 * 1000 + timeinfo.tm_hour * 3600 * 1000;
        // tim <<= 15;
        // tim /= 1000;

        rpi_tx_structured_data_t rpi_out_data = {
            .time = sec,
            .hum = rpi_in_data.hum / 0.002,
            .v_ground = rpi_in_data.v_ground / 0.005,
            .v_air = rpi_in_data.v_air / 0.01,
            .temp = rpi_in_data.temp / 0.005,
            .press = rpi_in_data.press / 5,
            .longt = convert_coord_to_fmt(rpi_in_data.longt) & 0x7FFFFFFF,
            .lat = convert_coord_to_fmt(rpi_in_data.lat) & 0x3FFFFFFF,
            .alt_rel_start = rpi_in_data.alt_rel_start / 0.125,
            .roll = rpi_in_data.roll / 0.0002,
            .pitch = rpi_in_data.pitch / 0.0002,
            .yaw = rpi_in_data.yaw / 0.0002,
            .energy = rpi_in_data.energy / 0.25};

        rpi_out_data.crc = calculate_crc((void *)&rpi_out_data, (sizeof(rpi_out_data) / 2) - 1);

        osMutexAcquire(SPI_Task_Mutex, osWaitForever);
        SPI_write_bytes(&SPI, &rpi_out_data, sizeof(rpi_out_data), RPI_nCS_GPIO_Port, RPI_nCS_Pin);
        osMutexRelease(SPI_Task_Mutex);
    }
}
