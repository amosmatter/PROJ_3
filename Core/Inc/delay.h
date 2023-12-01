/*
 * delay.h
 *
 *  Created on: 24.06.2020
 *      Author: zihlmalb
 */


#ifndef TTS_OLD_DELAY_H_
#define TTS_OLD_DELAY_H_

#include <stdint.h>
#include "cmsis_os2.h"

#ifndef FCPU
#warning FCPU not defined. Assuming 8MHz
#define FCPU (48000000)
#endif

// This macro does the conversion from us to the number of
// iterations to do by the busywait-routine
#define DELAY_MICROSECONDS(us) ((us) * (FCPU/(4*1000000))-2)


/**
 * Does a busy-wait during given value val.
 * The wait time is calculated: time/us = (val * 4000000 + 8)/fcpu
 * @param val
 */
void delay_cycles(uint32_t val);


/**
 * Does a busy-wait during given value in us.
 * @param us
 */
static inline void delay_us(uint32_t us){ delay_cycles(DELAY_MICROSECONDS(us));}
osStatus_t delay_ms(uint32_t ms);

#endif /* TTS_OLD_DELAY_H_ */
