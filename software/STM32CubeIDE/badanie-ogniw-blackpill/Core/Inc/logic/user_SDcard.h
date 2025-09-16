/*
 * user_SDcard.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#ifndef SRC_LOGIC_USER_SDCARD_H_
#define SRC_LOGIC_USER_SDCARD_H_

#include "fatfs.h"
#include "sensors/st7735.h"
#include "sensors/fatfs_sd.h"


typedef struct {
	FATFS fs;
	FIL fil;
	FRESULT res;
} SDcard_t;

void SDcardInit(SDcard_t* sd, const char *folder_name);
void SDcardWriteData(SDcard_t* sd, struct sensors *s);
void SDcardClose(SDcard_t* sd);


#endif /* SRC_LOGIC_USER_SDCARD_H_ */
