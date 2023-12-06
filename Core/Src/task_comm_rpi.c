/*
 * task_comm_rpi.c
 *
 *  Created on: Dec 6, 2023
 *      Author: amosm
 */

#include "common_task_defs.h"
#include "task_comm_rpi.h"
#include "delay.h"
#include "spi_common.h"
#include <stdio.h>
#include <stdlib.h>


typedef struct __packed
{
    uint32_t time;     // time in seconds, shifted left by 8
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

void comm_rpi_task(void *pvParameters)
{
    rpi_tx_data_t rpi_in_data;
    while (1)
    {
        osMessageQueueGet(rpi_tx_queue_handle, &rpi_in_data, 0, osWaitForever);
        uint64_t time = rpi_in_data.time_ms;
        time <<= 8;
        time /= 1000;

        // TODO convert to dezidegrees
        uint32_t longt = rpi_in_data.longt;
        uint32_t lat = rpi_in_data.lat;

        rpi_tx_structured_data_t rpi_out_data = {
            .time = time,
            .hum = rpi_in_data.hum / 0.002,
            .v_ground = rpi_in_data.v_ground / 0.005,
            .v_air = rpi_in_data.v_air / 0.01,
            .temp = rpi_in_data.temp / 0.005,
            .press = rpi_in_data.press / 5,
            .longt = longt,
            .lat = lat,
            .alt_rel_start = rpi_in_data.alt_rel_start / 0.125,
            .roll = rpi_in_data.roll / 0.0002,
            .pitch = rpi_in_data.pitch / 0.0002,
            .yaw = rpi_in_data.yaw / 0.0002,
            .energy = rpi_in_data.energy / 0.25};
        continue;
        SPI_write_bytes(&SPI, &rpi_out_data, sizeof(rpi_out_data), RPI_nCS_GPIO_Port, RPI_nCS_Pin);
    }
}
