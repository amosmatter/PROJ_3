#include "main.h"
#include "common_task_defs.h"
#include "time.h"

osMessageQueueId_t time_q;
typedef struct
{
    time_t time_s;
    uint32_t ms;
    uint32_t ticks;
} sys_time_t;

void init_time()
{
    time_q = osMessageQueueNew(1, sizeof(sys_time_t), NULL);
}

int isSwissDaylightSaving(int month, int day, int weekday)
{
    return (month > 3 && month < 10 ||                       //
            (month == 3 && ((31 - day) < (7 - weekday))) ||  // has the last sunday of  march  passed?
            (month == 10 && ((31 - day) >= (7 - weekday)))); // has the last sunday of  october not  passed?
}

uint32_t get_sys_ms()
{
    return HAL_GetTick() * HAL_GetTickFreq();
}

void set_time(struct tm *utc_time, uint32_t *ms)
{

    sys_time_t sys_time;
    sys_time.ticks = get_sys_ms();
    sys_time.ms = *ms;
    sys_time.time_s = mktime(utc_time) + 3600 * (1 + isSwissDaylightSaving(utc_time->tm_mon, utc_time->tm_mday, utc_time->tm_wday)); // 1 hour offset CET or 2 if DST
    osMessageQueueReset(time_q);
    osMessageQueuePut(time_q, &sys_time, 0, 0);
}

void get_time(struct tm *localtime, uint32_t *ms)
{
    sys_time_t sys_time;

    osMessageQueueGet(time_q, &sys_time, 0, osWaitForever);
    osMessageQueuePut(time_q, &sys_time, 0, 0); // TODO: this is a hack, switch to freertos version

    uint32_t d_ticks = get_sys_ms() - sys_time.ticks;

    sys_time.ms += d_ticks;
    sys_time.time_s += (sys_time.ms / 1000);
    if (ms != NULL)
    {
        *ms = (sys_time.ms % 1000);
    }

    localtime_r(&sys_time.time_s, localtime);
}
