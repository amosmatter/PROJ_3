/*
 * task_IMU.h
 *
 *  Created on: Nov 21, 2023
 *      Author: amosm
 */

#ifndef INC_TASK_IMU_H_
#define INC_TASK_IMU_H_

#include "ICM_20948/src/util/ICM_20948_C.h"

ICM_20948_Status_e ICM_20948_initialize_DMP(ICM_20948_Device_t *pdev);
ICM_20948_Status_e ICM_20948_startup_default(ICM_20948_Device_t *pdev, const ICM_20948_Serif_t *pserif, uint32_t minimal);
void IMU_task(void *pvParameters);

#endif /* INC_TASK_IMU_H_ */
