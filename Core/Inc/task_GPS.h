/*
 * task_GPS.h
 *
 *  Created on: Nov 27, 2023
 *      Author: amosm
 */

#ifndef INC_TASK_GPS_H_
#define INC_TASK_GPS_H_

void GPS_task(void *pvParameters);
void rcv_gps_uart_irq_handler(void);
#endif /* INC_TASK_GPS_H_ */
