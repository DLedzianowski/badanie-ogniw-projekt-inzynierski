/*
 * user_SDcard.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#ifndef SRC_LOGIC_USER_SDCARD_H_
#define SRC_LOGIC_USER_SDCARD_H_

#include "fatfs.h"
#include "sensors/ILI9341_STM32_Driver.h"
#include "sensors/ILI9341_GFX.h"
#include "sensors/fatfs_sd.h"


typedef struct {
	FATFS fs;
	FIL fil;
	FRESULT res;
} SDcard_t;
extern SDcard_t sd;

void SDcardInit(const char *folder_name);
void SDcardWriteData(void);
void SDcardClose(void);

#endif /* SRC_LOGIC_USER_SDCARD_H_ */
