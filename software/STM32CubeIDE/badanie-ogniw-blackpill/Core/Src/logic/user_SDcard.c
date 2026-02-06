/*
 * user_SDcard.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */
//exFAT

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
		//ILI9341_DrawText(((ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth("Error in file!", FONT1)) / 2), ILI9341_SCREEN_HEIGHT-FONT1h, "Error in file!", FONT1, RED, BLACK);
		ILI9341_DrawRectangle(0, ILI9341_SCREEN_HEIGHT-1, ILI9341_SCREEN_WIDTH, 1, RED);
		HAL_Delay(RETRY_DELAY_MS);
	}

	HAL_Delay(100);
	retry_count = 2;
	while (retry_count--) {
		sd.res = f_open(&sd.fil, folder_name, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		if (sd.res == FR_OK) {
			break;
		}
		LOG_DEBUG( "Error opening SDcard file! (%d). Retrying...\r\n", sd.res);
		HAL_Delay(RETRY_DELAY_MS);
	}

	HAL_Delay(100);
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
	f_puts("TVOC_ppb,CO2_eq_ppm,"
	       "Temperatura1,Temperatura2,Temperatura3,Wilgotnosc,"
	       "Napiecie_V,Prad_A,Zadany_prad_Charge,Zadany_prad_Discharge,"
	       "Stan_baterii,Typ_baterii\n", &sd.fil);

	f_sync(&sd.fil);
}

void SDcardWriteData() {
//	// ERROR SDcard -> OLED
//	if (f_lseek(&sd.fil, f_size(&sd.fil)) != FR_OK) {
//		LOG_DEBUG("Error seeking in file!\r\n");
//		ILI9341_DrawText(((ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth("Error in file!", FONT1)) / 2), ILI9341_SCREEN_HEIGHT-FONT1h, "Error in file!", FONT1, RED, BLACK);
//		return;
//	}

	char buffer[200];
	int32_t len = snprintf(buffer, sizeof(buffer),
			"%u,%u,"
			"%.2f,%.2f,%.2f,%.2f,"
			"%.2f,%.2f,%.2f,%.2f,"
			"%i,%i\n",
			s.tvoc_ppb, s.co2_eq_ppm,
			s.BME280temperature[0], s.BME280temperature[1], s.BME280temperature[2], s.BME280humidity,
			s.voltage, s.current, st.get_current_charge, st.set_current_discharge,
			st.battery_state, st.battery_current);

#if ENABLE_DEBUG
	if (len >= sizeof(buffer)) {
		LOG_DEBUG("SD buffer overflow!\r\n");
		return;
	}
#endif


	// inserting into package
	if (f_puts(buffer, &sd.fil) < 0) {
		LOG_DEBUG("Error writing to file!\r\n");
		//ILI9341_DrawText(((ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth("Error writing to file!", FONT1)) / 2), ILI9341_SCREEN_HEIGHT-FONT1h, "Error writing to file!", FONT1, RED, BLACK);
		ILI9341_DrawRectangle(0, ILI9341_SCREEN_HEIGHT-1, ILI9341_SCREEN_WIDTH, 1, RED);
		return;
	}

	// sending package to SD
	static uint8_t sync_cnt = 0;
	if (++sync_cnt >= 10) {
		if (f_sync(&sd.fil) != FR_OK) {
			LOG_DEBUG("Error sync sd!\r\n");
			//ILI9341_DrawText(((ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth("Error sync sd!", FONT1)) / 2), ILI9341_SCREEN_HEIGHT-FONT1h, "Error sync sd!", FONT1, RED, BLACK);
			ILI9341_DrawRectangle(0, ILI9341_SCREEN_HEIGHT-1, ILI9341_SCREEN_WIDTH, 1, RED);
		}
		sync_cnt = 0;
	}
	// SPI_RxByte() - SD BUSY state max 2s
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
