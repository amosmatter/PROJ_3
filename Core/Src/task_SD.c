
#include <stdio.h>
#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "FATFS/diskio.h"
#include "common_task_defs.h"
#include <stdarg.h>
#include "math.h"

#define HEADER "Time [s]; Humidity [%]; Groundspeed [m/s]; Airspeed [m/s]; Temperature [deg C]; Airpressure [kPa]; Longitude; Latitude; Flight Height [m]; Roll [deg]; Pitch [deg]; Yaw [deg]; Energy [m];Energieableitung[m/s];"
#define DBL_FORMATTING "%.4+e;\t"
#define DBL_FORMATTING_SZ (1 + 2 + 4 + 4 + 1 + 1) // sign +  2 before frac +  frac + e + exp sign + 2 exp num + ; + tab

char filebuffer[2048];

char *write_val(char *buffer, size_t *maxlen, size_t expected_sz, const char *fmt, ...)
{
    if (!buffer || !maxlen || !fmt)
    {
        return NULL;
    }

    int ret = 0;
    if (*maxlen >= expected_sz)
    {
        va_list args;
        va_start(args, fmt);
        ret = vsnprintf(buffer, *maxlen, fmt, args);
        va_end(args);
    }

    if (ret != expected_sz)
    {
        ret = snprintf(buffer, *maxlen, ";");
        if (ret != 1)
        {
            return NULL;
        }
    }

    *maxlen -= ret;
    buffer += ret;
    return buffer;
}

char *write_timestamp(char *buffer, size_t *maxlen, double val)
{
    return write_val(buffer, maxlen, DBL_FORMATTING_SZ, DBL_FORMATTING, val);
}
char *write_dbl(char *buffer, size_t *maxlen, double val)
{
    return write_val(buffer, maxlen, DBL_FORMATTING_SZ, DBL_FORMATTING, val);
}

char *write_str(char *buffer, size_t *maxlen, char *val)
{
    return write_val(buffer, maxlen, strlen(val), "%s", val);
}

char *write_line(char *buffer, size_t *maxlen, csv_dump_data_t *data)
{
    buffer = write_dbl(buffer, &maxlen, data->pitch / M_PI * 180);
    buffer = write_dbl(buffer, &maxlen, data->roll / M_PI * 180);
    buffer = write_dbl(buffer, &maxlen, data->yaw / M_PI * 180);

    buffer = write_str(buffer, &maxlen, "\r\n");
    return buffer;
}

void SD_task(void *pvParameters)
{
    osDelay(2000);
    csv_dump_data_t data_in;

    FATFS fs;
    FATFS *fs_ptr = &fs;
    FRESULT res;
    DIR dir;
    FILINFO fileInfo;
    FIL file;

    res = f_mount(fs_ptr, "", 1); // Mounts the default drive
    while (res != FR_OK)
    {

        printf("SD failed to mount. Error: %d\n", res);
        osDelay(100);
        res = f_mount(&fs, "", 1); // Mounts the default drive
    }

    // Open the directory
    res = f_opendir(&dir, "/"); // Replace "/" with your desired directory path
    if (res != FR_OK)
    {
        printf("Failed to open directory. Error code: %d\n", res);
        osDelay(-1);
    }

    printf("Listing contents of the directory:\n");
    while (1)
    {
        res = f_readdir(&dir, &fileInfo);
        if (res != FR_OK || fileInfo.fname[0] == 0)
            break; // Break on error or end of directory

        // Print the file/folder name
        printf("%s\n", fileInfo.fname);
    }

    /* res = f_mkdir("/test");
     if (res != FR_OK)
     {
         printf("Failed to make directory. Error code: %d\n", res);
         osDelay(-1);
     }*/

    res = f_opendir(&dir, "/test"); // Replace "/" with your desired directory path
    if (res != FR_OK)
    {
        printf("Failed to open directory. Error code: %d\n", res);
        osDelay(-1);
    }

    // Open or create a file named "data.csv"
    res = f_open(&file, "data.csv", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        printf("Failed to open/create file. Error code: %d\n", res);
        osDelay(-1);
    }

    // Write header to the file
    sprintf(filebuffer, "%s\n", HEADER);
    UINT bytes_written;
    res = f_write(&file, filebuffer, strlen(filebuffer), &bytes_written);
    if (res != FR_OK)
    {
        printf("Failed to write to file. Error code: %d\n", res);
        f_close(&file);
        osDelay(-1);
    }

    while (1)
    {
        osMessageQueueGet(csv_queue_handle, &data_in, 0, osWaitForever);
        if (data_in.time_ms == -1)
        {
            break;
        }
        write_line(filebuffer, sizeof(filebuffer) - 1, &data_in);
        printf("%s", filebuffer);
        res = f_write(&file, filebuffer, strlen(filebuffer), &bytes_written);
        if (res != FR_OK)
        {
            printf("Failed to write to file. Error code: %d\n", res);
            break;
        }
    }
    // Close the file
    f_close(&file);
    osDelay(-1);
}
