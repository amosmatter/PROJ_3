
#include <stdio.h>
#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"
#include "FATFS/diskio.h"

char filebuffer[2048];

void SD_task(void *pvParameters)
{
    FATFS fs;
	FATFS* fs_ptr = &fs;
    FRESULT res;
    DIR dir;
    FILINFO fileInfo;
    FIL file;

	printf("\n\n\nentered SD task\n");
    res = f_mount(fs_ptr, "", 1); // Mounts the default drive
    while (res != FR_OK)
    {

        printf("SD failed to mount. Error: %d\n", res);
        osDelay(100);
        res = f_mount(&fs, "", 1); // Mounts the default drive

    }



    // Open the directory
    res = f_opendir(&dir, "/"); // Replace "/" with your desired directory path
    if (res != FR_OK) {
        printf("Failed to open directory. Error code: %d\n", res);
        return -1;
    }



    printf("Listing contents of the directory:\n");
    while (1) {
        res = f_readdir(&dir, &fileInfo);
        if (res != FR_OK || fileInfo.fname[0] == 0) break; // Break on error or end of directory

        // Print the file/folder name
        printf("%s\n", fileInfo.fname);
    }

    res = f_mkdir("/test");
    if (res != FR_OK)
    {
        printf("Failed to make directory. Error code: %d\n", res);
        return -1;
    }

    res = f_opendir(&dir, "/test"); // Replace "/" with your desired directory path
    if (res != FR_OK) {
        printf("Failed to open directory. Error code: %d\n", res);
        return -1;
    }

    // Open or create a file named "data.csv"
    res = f_open(&file, "data.csv", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Failed to open/create file. Error code: %d\n", res);
        return -1;
    }

    // Write header to the file
    sprintf(filebuffer, "test, test1, test2\n");
    UINT bytes_written;
    res = f_write(&file, filebuffer, strlen(filebuffer), &bytes_written);
    if (res != FR_OK) {
        printf("Failed to write to file. Error code: %d\n", res);
        f_close(&file);
        return -1;
    }

    // Write some example data to the file
    sprintf(filebuffer, "value1, value2, value3\n");
    res = f_write(&file, filebuffer, strlen(filebuffer), &bytes_written);
    if (res != FR_OK) {
        printf("Failed to write to file. Error code: %d\n", res);
        f_close(&file);
        return -1;
    }

    // Close the file
    f_close(&file);


    while (1)
    {
        printf("sd_task\n");
        osDelay(100);
    }
}
