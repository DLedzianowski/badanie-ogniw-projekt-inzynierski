#ifndef INC_FATFS_SD_DEFS_H_
#define INC_FATFS_SD_DEFS_H_

#include <stdint.h>

void SDcardInit(char* folder_name);
void SDcardWriteData(float *temperature, int32_t *pressure);
void SDcardClose();


#endif /* INC_FATFS_SD_DEFS_H_ */
