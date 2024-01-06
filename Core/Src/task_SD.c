
#include <stdio.h>
#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "FATFS/diskio.h"
#include "common_task_defs.h"
#include <stdarg.h>
#include "math.h"
#include "system_time.h"

#define HEADER "Time [s]; Humidity [percent]; Groundspeed [m/s]; Airspeed [m/s]; Temperature [deg C]; Airpressure [kPa]; Longitude; Latitude; Flight Height [m]; Roll [deg]; Pitch [deg]; Yaw [deg]; Energy [m];Energieableitung[m/s];"
#define DBL_FORMATTING "%.4+e;\t"

#define RETRIES 99
#define RETRY_PERIOD 20

int write_dbl(FIL *buffer, double val)
{
    return f_printf(buffer, DBL_FORMATTING, val);
}
int write_str(FIL *buffer, const char *val)
{
    return f_printf(buffer, "%s", val);
}

void write_line(FIL *buffer, csv_dump_data_t *data)
{
    write_dbl(buffer, data->pitch / M_PI * 180);
    write_dbl(buffer, data->roll / M_PI * 180);
    write_dbl(buffer, data->yaw / M_PI * 180);

    write_str(buffer, "\r\n");
}

void SD_task(void *pvParameters)
{
    csv_dump_data_t data_in;

    FATFS fs;
    FIL file;

    // imu and pth share the spi bus so allow them to go first
    osEventFlagsWait(init_events, ev_init_imu | ev_init_pth, osFlagsNoClear | osFlagsWaitAll, osWaitForever);
    uint32_t ctr = 0;

    while (1)
    {
        FRESULT res = f_mount(&fs, "", 1); // Mounts the default drive
        if (res == FR_OK)
        {
            break;
        }

        if (ctr++ >= RETRIES)
        {
            printf("Giving up on SD Card! Error %d\n", res);
            osDelay(-1); // Go to sleep
        }

        DEBUG_PRINT("SD failed to mount. Error: %d\n", res);
        osDelay(RETRY_PERIOD);
    }

    struct tm timeinfo;
    get_time(&timeinfo, NULL);

    char folderName[64];
    snprintf(folderName, 64, "0:/logs/%04d_%02d_%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

    char fileName[96];
    snprintf(fileName, 96, "%s/%04d_%02d_%02d___%02d_%02d_%02d.csv",folderName,timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    DEBUG_PRINT("Folder name: %s\n", folderName);
    DEBUG_PRINT("File name: %s\n", fileName);


    while (1)
    {
        FRESULT res = f_mkdir("logs");
        if (res == FR_OK)
        {
            break;
        }
        if (res == FR_EXIST)
        {
            break;
        }
        if (ctr++ > RETRIES)
        {
            printf("Giving up on SD Card! Error %d\n", res);
            osDelay(-1); // Go to sleep
        }

        DEBUG_PRINT("Failed with logs folder. Error code: %d\n", res);
        osDelay(RETRY_PERIOD);
    }

    while (1)
    {
        FRESULT res = f_mkdir(folderName);
        if (res == FR_OK)
        {
            break;
        }
        if (res == FR_EXIST)
        {
            break;
        }

        if (ctr++ > RETRIES)
        {
            printf("Giving up on SD Card! Error %d\n", res);
            osDelay(-1); // Go to sleep
        }
        DEBUG_PRINT("Failed to create daily folder. Error code: %d\n", res);
        osDelay(RETRY_PERIOD);
    }

    while (1)
    {

        FRESULT res = f_open(&file, fileName, FA_WRITE | FA_CREATE_NEW);
        if (res == FR_OK)
        {
            break;
        }

        if (ctr++ > RETRIES)
        {
            printf("Giving up on SD Card! Error %d\n", res);
            osDelay(-1); // Go to sleep
        }

		if (res == FR_EXIST)
		{
			snprintf(fileName, 96, "%s/%04d_%02d_%02d___%02d_%02d_%02d__%03lu.csv",folderName,timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ctr);
			continue;
		}

        DEBUG_PRINT("Failed to create file! Error %d\n", res);

    osDelay(RETRY_PERIOD);
    }

    DEBUG_PRINT("File opened: %s\n", fileName);
    ctr = 0;
    f_lseek(&file, 0);


    while (1)
    {
        int ret = f_printf(&file, "%s\n", HEADER);
        if (ret > 0)
        {
            break;
        }
        if (ctr > RETRIES)
        {
            printf("Giving up on SD Card! Error %d\n", ret);
            osDelay(-1);
        }
        DEBUG_PRINT("Failed to write Header");
        osDelay(RETRY_PERIOD);
    }

    osEventFlagsSet(init_events, ev_init_csv);
	osEventFlagsWait(init_events, ev_init_all, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

    while (1)
    {
        osMessageQueueGet(csv_queue_handle, &data_in, 0, osWaitForever);
        if (data_in.time_ms == -1)
        {
            break;
        }
        write_line(&file, &data_in);
    }
    printf("Closing file\n");
    // Close the file
    f_close(&file);
    osDelay(-1);
}
