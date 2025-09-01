/*
 * user_OLED.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#include "logic/user_OLED.h"

SensorsFunc_t sensor_display[] = {
    display_sensor_first,
    display_sensor_second
};

MenuFunc_t menu_display[] = {
	display_menu_main,
	display_menu_start,
	display_menu_battery_type,
	display_menu_adc,
	display_menu_stop
};

MenuFunc_t menu_actions[] = {
	action_menu_main,
	action_menu_start,
	action_menu_battery_type,
	action_menu_adc,
	action_menu_stop
};

void OLED_manage(struct state *st, struct sensors *s) {
	switch (st->current_screen_type) {
	case SCREEN_MENU:

		if (st->is_enc_pressed || st->update_actions){
			menu_actions[st->menu_current](st);
			st->is_enc_pressed = false;
		}
		if (st->screen_clear == true && st->current_screen_type == SCREEN_MENU){
			st->screen_clear = false;
			ST7735_FillScreenFast(ST7735_BLACK);

			menu_display[st->menu_current](st);
		}
	    break;

	case SCREEN_SENSOR:
		if (st->screen_clear == true) {
			st->screen_clear = false;
			ST7735_FillScreenFast(ST7735_BLACK);
		}
		sensor_display[st->sensor_current](s);
		if (st->is_enc_pressed) {
			st->is_enc_pressed = false;
			st->current_screen_type = SCREEN_MENU;
		}
	    break;
	default:
		break;
	}
}

void display_sensor_first(struct sensors *s) {
	char buffer[150];

	// Temperatura
	snprintf(buffer, sizeof(buffer), "Temp: %5.2f C ", s->BMP280temperature[0]);
	ST7735_WriteString(5, 5, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Ciśnienie
	snprintf(buffer, sizeof(buffer), "Prs:  %6.4f bar", (float)s->BMP280pressure[0] / 100000.0f);
	ST7735_WriteString(5,  20, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// TVOC
	snprintf(buffer, sizeof(buffer), "TVOC: %6u ppb", s->tvoc_ppb);
	ST7735_WriteString(5,  35, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// CO2eq
	snprintf(buffer, sizeof(buffer), "CO2:  %6u ppm", s->co2_eq_ppm);
	ST7735_WriteString(5,  50, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Etanol/512.0
	float ethanol = s->scaled_ethanol_signal / 512.0f;
	snprintf(buffer, sizeof(buffer), "EtOH: %6.2f", ethanol);
	ST7735_WriteString(5,  65, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// H2/512.0
	float h2 = s->scaled_h2_signal / 512.0f;
	snprintf(buffer, sizeof(buffer), "H2:   %6.2f", h2);
	ST7735_WriteString(5,  80, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// INA219_Current
	snprintf(buffer, sizeof(buffer), "Current:  %4d mA", s->INA219_Current);
	ST7735_WriteString(5,  95, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// INA219_Voltage
	snprintf(buffer, sizeof(buffer), "Voltage:  %4u mV", s->INA219_Voltage);
	ST7735_WriteString(5,  110, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// INA219_Power
	snprintf(buffer, sizeof(buffer), "Power:  %4u mW", s->INA219_Power);
	ST7735_WriteString(5,  125, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void display_sensor_second(struct sensors *s) {
	char buffer[150];

	// Temperatura 1
	snprintf(buffer, sizeof(buffer), "Temp1: %5.2f C ", s->BMP280temperature[0]);
	ST7735_WriteString(5, 5, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Temperatura 2
	snprintf(buffer, sizeof(buffer), "Temp2: %5.2f C ", s->BMP280temperature[1]);
	ST7735_WriteString(5, 20, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Temperatura 3
	snprintf(buffer, sizeof(buffer), "Temp3: %5.2f C ", s->BMP280temperature[2]);
	ST7735_WriteString(5, 35, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// PWM "ADC: %.2f%%\r\n",
	snprintf(buffer, sizeof(buffer), "PWM: %.2f%% ", s->adc_percentage);
	ST7735_WriteString(5, 50, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void display_menu_main(struct state *st) {
	char buffer[50];
    uint8_t first_visible;
    uint8_t ptr_pose;  // pointer position {up, middle, down}

    if (st->menu_current_ptr == MENU_START) {  // first position
    	first_visible = 0;
    	ptr_pose = 0;
    }
    else if (st->menu_current_ptr == MENU_SCREEN_COUNT - 1) {  // last position
    	first_visible = SCREENS_MENU_NUM - SCREENS_MENU_VISIBLE_ITEMS;
        ptr_pose = SCREENS_MENU_VISIBLE_ITEMS - 1;
    }
    else {  // middle position
    	first_visible = st->menu_current_ptr - 2;
    	ptr_pose = 1;
    }

    for (uint8_t i = 0; i < SCREENS_MENU_VISIBLE_ITEMS; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", menu[first_visible + i]);

    	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
    	uint16_t y_pos = i*(Font_11x18.height+26)+26;
    	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
        // cursor
    	if ( ptr_pose == i ) {
    		ST7735_FillRectangle(x_pos, y_pos + Font_11x18.height + 1, strlen(buffer)*Font_11x18.width, 2, ST7735_WHITE);
    	}
    }

	uint16_t y_pos = ST7735_HEIGHT - Font_7x10.height;
	snprintf(buffer, sizeof(buffer), "%s", batteries[st->battery_current]);
	ST7735_WriteString(5, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	uint16_t x_pos = (strlen(buffer) + 2) * Font_7x10.width;
	snprintf(buffer, sizeof(buffer), "%.1f%%", s.adc_percentage);
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

//	snprintf(buffer, sizeof(buffer), "%s", batteries[st->battery_current]);
//	x_pos = ST7735_WIDTH - (strlen(buffer) * Font_7x10.width) - 5;
//	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	x_pos = ST7735_WIDTH - img_thunder.width - 3;
	y_pos = ST7735_HEIGHT - img_thunder.height - 3;
	ST7735_DrawImage(x_pos, y_pos, img_thunder.width, img_thunder.height, img_thunder.data);

}

void display_menu_start(struct state *st) {
	char buffer[] = "Press to start";

	//snprintf(buffer, sizeof(buffer), "Press to start");
	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_7x10.width)) / 2;
	uint16_t y_pos = (ST7735_WIDTH + Font_7x10.height)/2;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void display_menu_battery_type(struct state *st) {
	char buffer[50];

    for (uint8_t i = 0; i < 2; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", batteries[i]);

    	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
    	uint16_t y_pos = (ST7735_HEIGHT - (2 * Font_11x18.height))/2 + (2*Font_11x18.height*i);
    	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
        // cursor
    	if ( st->battery_ptr == i ) {
    		ST7735_FillRectangle(x_pos, y_pos + Font_11x18.height + 1, strlen(buffer)*Font_11x18.width, 2, ST7735_WHITE);
    	}
    }
}

void display_menu_adc(struct state *st) {
	char buffer[20];

	snprintf(buffer, sizeof(buffer), "PWM: %.1f%%", s.adc_percentage);
	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_7x10.width)) / 2;
	uint16_t y_pos = (ST7735_WIDTH + Font_7x10.height)/2;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	st->update_actions = true;
}

void display_menu_stop(struct state *st) {
	char buffer[] = "Stop";

	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_7x10.width)) / 2;
	uint16_t y_pos = (ST7735_WIDTH + Font_7x10.height)/2;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

/*
 *  click action
 */

void action_menu_main(struct state *st) {
	st->menu_current = st->menu_current_ptr;
	st->screen_clear = true;
}

void action_menu_start(struct state *st) {
	st->current_screen_type = SCREEN_SENSOR;
	st->is_measurements_started = true;

	st->menu_current = MENU_MAIN;
	st->screen_clear = true;
}

void action_menu_battery_type(struct state *st) {
	st->battery_current = st->battery_ptr;

//	if (batteries[st->battery_current] == "Li-Pol") {
//		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_y, GPIO_PIN_SET);
//	}
//	else if (batteries[st->battery_current] == "Li-On") {
//		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_y, GPIO_PIN_RESET);
//	}

	st->current_screen_type = SCREEN_MENU;
	st->menu_current = MENU_MAIN;
	st->screen_clear = true;
}

void action_menu_adc(struct state *st) {
	if (st->update_actions == true) {
		// update adc every 1s
	    static uint32_t last_update = 0;
	    uint32_t now = HAL_GetTick();

	    if (now - last_update >= 1000) {
	        last_update = now;
			get_adc_percentage();

			// refresh adc val
			st->screen_clear = true;
			st->update_actions = false;
	    }
	}
	if (st->is_enc_pressed == true) {
		st->update_actions = false;
		st->menu_current = MENU_MAIN;
		st->current_screen_type = SCREEN_MENU;
		st->screen_clear = true;
	}
}

void action_menu_stop(struct state *st) {
	st->current_screen_type = SCREEN_MENU;
	st->is_measurements_started = false;

	st->menu_current = MENU_MAIN;
	st->screen_clear = true;

	SDClose();
}

