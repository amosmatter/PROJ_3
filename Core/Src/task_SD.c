#include "main.h"
#include "FATFS/ff.h"
#include "cmsis_os2.h"

char workBuffer[512]; // Work area (should be aligned to the sector size)

void SD_task(void *pvParameters)
{
    FRESULT mountResult;
    FATFS fs;

    mountResult = f_mount(&fs, "", 1); // Mounts the default drive
    if (mountResult != FR_OK)
    {
        printf("SD failed to mount. Error: %d\n", mountResult);
        // Handle mount failure
        if (mountResult == FR_NO_FILESYSTEM)
        {

            FRESULT formatResult = f_mkfs("", NULL, workBuffer, sizeof(workBuffer));
            if (formatResult != FR_OK)
            {
                // Handle formatting error
                printf("SD failed to format. Error: %d\n", formatResult);
            }
        }
    }

    FILINFO fileInfo;
    FRESULT statResult;

    statResult = f_stat("/", &fileInfo);
    if (statResult != FR_OK)
    {
        printf("SD failed to stat. Error: %d\n", statResult);
        // Handle error
    }
    else
    {
        printf("SD stat result: %s\n", fileInfo.fname);
        // Drive is running, proceed to access its contents
    }

    char line[128] = {};
    while (1)
    {
        printf("sd_task\n");
        osDelay(100);
    }
}
