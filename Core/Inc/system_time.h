#ifndef A007AA87_242E_4408_B3AB_83AA440FC08A
#define A007AA87_242E_4408_B3AB_83AA440FC08A
#include "time.h"
#include "stdint.h"

void init_time();
void set_time(struct tm *utc_time, uint32_t *ms);
void get_time(struct tm *localtime, uint32_t *ms);

#endif /* A007AA87_242E_4408_B3AB_83AA440FC08A */
