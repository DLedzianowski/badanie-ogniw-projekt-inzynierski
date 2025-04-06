// fatfs_sd_defs.c

#include "fatfs_sd_defs.h"
#include "ff.h"

FATFS fs;
FIL fil;

void SDcardInit(char* folder_name) {
    HAL_Delay(500);

    if (f_mount(&fs, "", 0) != FR_OK) {
        printf("Error mounting filesystem!\n");
        return;
    }

    if (f_open(&fil, folder_name, FA_OPEN_ALWAYS | FA_WRITE | FA_READ) != FR_OK) {
        printf("Error opening SDcard file!\n");
        return;
    }

    if (f_size(&fil) == 0) {
        f_puts("--- Nowy pomiar ---\n", &fil);
        f_puts("Temperatura (°C),Ciśnienie (hPa)\n", &fil);
    }
}

void SDcardWriteData(float *temperature, int32_t *pressure) {
    if (f_lseek(&fil, f_size(&fil)) != FR_OK) {
        printf("Error seeking in file!\n");
        return;
    }

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%.2f,%ld\n", *temperature, *pressure);  // Użyj dereferencji wskaźników

    if (f_puts(buffer, &fil) < 0) {
        printf("Error writing to file!\n");
    }

    if (f_sync(&fil) != FR_OK) {
        printf("Error syncing file!\n");
    }
}


void SDcardClose() {
    if (f_close(&fil) != FR_OK) {
        printf("Error closing file!\n");
    }
}
