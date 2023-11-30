
#include <stdio.h>
#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"
#include "FATFS/diskio.h"
void SD_task(void *pvParameters)
{
    FATFS fs;
	FATFS* fs_ptr = &fs;
    FRESULT res;
    DIR dir;
    FILINFO fileInfo;

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

    while (1)
    {
        printf("sd_task\n");
        osDelay(100);
    }
}
