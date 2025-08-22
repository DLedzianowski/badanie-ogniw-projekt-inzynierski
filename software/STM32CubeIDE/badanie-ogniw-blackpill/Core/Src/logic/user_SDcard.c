/*
 * user_SDcard.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#include "logic/user_SDcard.h"


void SDcardInit(SDcard_t* sd, char* folder_name) {
	uint8_t retry_count = 1;
	while (retry_count--) {
		sd->res = f_mount(&sd->fs, "", 1);
		if (sd->res == FR_OK) {
			break;
		}
		printf("Error mounting filesystem! (%d). Retrying...\r\n", sd->res);
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		HAL_Delay(RETRY_DELAY_MS);
	}

	retry_count = 1;
	while (retry_count--) {
		sd->res = f_open(&sd->fil, "test.csv", FA_OPEN_ALWAYS | FA_WRITE);
		if (sd->res == FR_OK) {
			break;
		}
		printf( "Error opening SDcard file! (%d). Retrying...\r\n", sd->res);
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		HAL_Delay(RETRY_DELAY_MS);
	}

	sd->res = f_lseek(&sd->fil, f_size(&sd->fil));
	if (sd->res != FR_OK) {
		printf("Error seeking to end of file! (%d)\r\n", sd->res);
		f_close(&sd->fil);
		return;
	}
	if (retry_count == 0) {
		SDcardClose(sd);
	}

	f_puts("\n--- Nowy pomiar ---\n", &sd->fil);
	f_puts("TVOC_ppb,CO2_eq_ppm,Ethanol_signal,H2_signal,Temperatura,Cisnienie,Napiecie_mV,Prad_mA,Moc_mW\n", &sd->fil);

	f_sync(&sd->fil);
}

void SDcardWriteData(SDcard_t* sd, struct sensors *s) {
	// ERROR SDcard -> OLED
	if (f_lseek(&sd->fil, f_size(&sd->fil)) != FR_OK) {
		printf("Error seeking in file!\r\n");
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		return;
	}

	char buffer[200];
	snprintf(buffer, sizeof(buffer), "%u,%u,%.2f,%.2f,%.2f,%ld,%u,%d,%u\n",
			s->tvoc_ppb, s->co2_eq_ppm, s->scaled_ethanol_signal/512.0f, s->scaled_h2_signal/512.0f, s->BMP280temperature[0], s->BMP280pressure[0],s->INA219_Voltage, s->INA219_Current, s->INA219_Power);


	if (f_puts(buffer, &sd->fil) < 0) {
		printf("Error writing to file!\r\n");
	}

	if (f_sync(&sd->fil) != FR_OK) {
		printf("Error syncing file!\r\n");
	}
	f_sync(&sd->fil);
}

void SDcardClose(SDcard_t* sd) {
	if (f_close(&sd->fil) == FR_OK) {
		printf("Closing file!\r\n");
	}
	else {
		printf("Error closing file!\r\n");
	}
}
