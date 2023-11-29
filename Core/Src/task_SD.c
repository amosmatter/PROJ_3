
#include <stdio.h>
#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"
#include "FATFS/diskio.h"
void SD_task(void *pvParameters)
{
    FATFS fs;
    FRESULT res;
	FATFS* fs_ptr = &fs;
	printf("entered SD task\n");
    res = f_mount(fs_ptr, "", 1); // Mounts the default drive
    while (res != FR_OK)
    {

        printf("SD failed to mount. Error: %d\n", res);
        osDelay(100);
        res = f_mount(&fs, "", 1); // Mounts the default drive

    }



    uint32_t freeClust;

	res = f_getfree("", &freeClust, &fs_ptr); // Warning! This fills fs.n_fatent and fs.csize!
	while(res != FR_OK) {
        osDelay(100);
		printf("f_getfree() failed, res = %d\r\n", res);
		res = f_getfree("", &freeClust, &fs_ptr); // Warning! This fills fs.n_fatent and fs.csize!
	}





    FILINFO fileInfo;
    FRESULT statResult;

    statResult = f_stat("/", &fileInfo);
    while (statResult != FR_OK)
    {
        printf("SD failed to stat. Error: %d\n", statResult);
        // Handle error
        osDelay(100);

        statResult = f_stat("/", &fileInfo);

    }

     printf("SD stat result: %s\n", fileInfo.fname);
        // Drive is running, proceed to access its contents


    while (1)
    {
        printf("sd_task\n");
        osDelay(100);
    }
}
