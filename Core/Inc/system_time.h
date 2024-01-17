#ifndef A007AA87_242E_4408_B3AB_83AA440FC08A
#define A007AA87_242E_4408_B3AB_83AA440FC08A
#include "time.h"
#include <stdint.h>
#include "cmsis_os2.h"

void init_time();
void set_time(struct tm *utc_time, uint32_t *ms);
void get_time(struct tm *localtime, uint32_t *ms);
time_t get_swiss_tz_offset(int month, int day, int weekday);



#define FCPU (16000000)

// Next 3 Funcions are from  
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

void sleep_timer_elapsed(void);

void sleep_ms(uint32_t ms);


#endif /* A007AA87_242E_4408_B3AB_83AA440FC08A */
