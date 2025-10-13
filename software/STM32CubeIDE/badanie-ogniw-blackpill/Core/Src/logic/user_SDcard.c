/*
 * user_SDcard.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#include "logic/user_SDcard.h"

SDcard_t sd;

void SDcardInit(const char *folder_name) {
	uint8_t retry_count = 2;
	while (retry_count--) {
		sd.res = f_mount(&sd.fs, "", 1);
		if (sd.res == FR_OK) {
			break;
		}
		LOG_DEBUG("Error mounting filesystem! (%d). Retrying...\r\n", sd.res);
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		HAL_Delay(RETRY_DELAY_MS);
	}

	retry_count = 2;
	while (retry_count--) {
		sd.res = f_open(&sd.fil, folder_name, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		if (sd.res == FR_OK) {
			break;
		}
		LOG_DEBUG( "Error opening SDcard file! (%d). Retrying...\r\n", sd.res);
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		HAL_Delay(RETRY_DELAY_MS);
	}

	sd.res = f_lseek(&sd.fil, f_size(&sd.fil));
	if (sd.res != FR_OK) {
		LOG_DEBUG("Error seeking to end of file! (%d)\r\n", sd.res);
		SDcardClose();
		return;
	}
	if (retry_count == 0) {
		SDcardClose();
	}

	f_puts("\n--- Nowy pomiar ---\n", &sd.fil);
	f_puts("TVOC_ppb,CO2_eq_ppm,Ethanol_signal,H2_signal,Temperatura,Cisnienie,Napiecie_mV,Prad_mA,Moc_mW\n", &sd.fil);

	f_sync(&sd.fil);
}

void SDcardWriteData(struct sensors *s) {
	// ERROR SDcard -> OLED
	if (f_lseek(&sd.fil, f_size(&sd.fil)) != FR_OK) {
		LOG_DEBUG("Error seeking in file!\r\n");
		ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
		return;
	}

	char buffer[200];
	snprintf(buffer, sizeof(buffer),
			"%u,%u,%.2f,%.2f,"
			"%.2f,%ld,%.2f,%ld,%.2f,%ld,"
			"%u,%d,%u,%i,%i\n",
			s->tvoc_ppb, s->co2_eq_ppm, s->scaled_ethanol_signal/512.0f, s->scaled_h2_signal/512.0f,
			s->BMP280temperature[0], s->BMP280pressure[0], s->BMP280temperature[1], s->BMP280pressure[1], s->BMP280temperature[2], s->BMP280pressure[2],
			s->INA219_Voltage, s->INA219_Current, s->INA219_Power, (int)s->adc_percentage, get_state_int());

	if (f_puts(buffer, &sd.fil) < 0) {
		LOG_DEBUG("Error writing to file!\r\n");
	}

	if (f_sync(&sd.fil) != FR_OK) {
		LOG_DEBUG("Error syncing file!\r\n");
	}
}

void SDcardClose(void) {
	if (f_close(&sd.fil) != FR_OK) {
		LOG_DEBUG("Error closing file!\r\n");
	}

	sd.res = f_mount(NULL, "", 1);
	if (sd.res == FR_OK) {
		LOG_DEBUG("Filesystem unmounted successfully!\r\n");
	} else {
		LOG_DEBUG("Error unmounting filesystem! (%d)\r\n", sd.res);
	}
}
