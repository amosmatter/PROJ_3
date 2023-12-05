/*
 * task_PTH.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amosm
 */

#ifndef SRC_TASK_PTH_H_
#define SRC_TASK_PTH_H_

#include "BME280/bme280_defs.h"
#include "main.h"

void PTH_task(void *pvParameters);

typedef struct bme280_data bme280_data_t;

#define PTH_MEAS_RATE (OUTPUT_RATE * 5)

#endif /* SRC_TASK_PTH_H_ */
