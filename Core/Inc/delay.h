/*
 * delay.h
 *
 *  Created on: 24.06.2020
 *      Author: zihlmalb
 */

#ifndef B3A5C413_D3B2_4F9E_BA6F_BB9DB5891870
#define B3A5C413_D3B2_4F9E_BA6F_BB9DB5891870

#ifndef TTS_OLD_DELAY_H_
#define TTS_OLD_DELAY_H_

#include <stdint.h>

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



#endif /* TTS_OLD_DELAY_H_ */


#endif /* B3A5C413_D3B2_4F9E_BA6F_BB9DB5891870 */
