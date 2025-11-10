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
	display_menu_current,
	display_menu_state,
	display_menu_auto_mode,
	display_menu_stop
};

MenuFunc_t menu_actions[] = {
	action_menu_main,
	action_menu_start,
	action_menu_battery_type,
	action_menu_current,
	action_menu_state,
	action_menu_auto_mode,
	action_menu_stop
};

void OLED_manage(void) {
	switch (st.current_screen_type) {
	case SCREEN_MENU:
		if (st.is_enc_pressed) {
			menu_actions[st.menu_current]();
			st.enc_offset = __HAL_TIM_GET_COUNTER(&htim1) / 4;
			st.is_enc_pressed = false;
			st.screen_clear = true;
		}
		if (st.screen_clear == true && st.current_screen_type == SCREEN_MENU) {
			st.screen_clear = false;
			ST7735_FillScreenFast(ST7735_BLACK);

			menu_display[st.menu_current]();
		}
	    break;

	case SCREEN_SENSOR:
		if (st.screen_clear == true) {
			st.screen_clear = false;
			ST7735_FillScreenFast(ST7735_BLACK);
		}

		sensor_display[st.sensor_current]();
		if (st.is_enc_pressed) {
			st.is_enc_pressed = false;
			st.current_screen_type = SCREEN_MENU;
			st.screen_clear = true;
		}
	    break;
	default:
		break;
	}
}

void display_sensor_first(void) {
	char buffer[150];

	// Temperatura 1
	snprintf(buffer, sizeof(buffer), "Temp1: %5.2f C ", s.BMP280temperature[0]);
	ST7735_WriteString(5, 5, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Temperatura 2
	snprintf(buffer, sizeof(buffer), "Temp2: %5.2f C ", s.BMP280temperature[1]);
	ST7735_WriteString(5, 20, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Temperatura 3
	snprintf(buffer, sizeof(buffer), "Temp3: %5.2f C ", s.BMP280temperature[2]);
	ST7735_WriteString(5, 35, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Ciśnienie
	snprintf(buffer, sizeof(buffer), "Prs:  %6.4f bar", (float)s.BMP280pressure[0] / 100000.0f);
	ST7735_WriteString(5,  50, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// TVOC
	snprintf(buffer, sizeof(buffer), "TVOC: %6u ppb", s.tvoc_ppb);
	ST7735_WriteString(5,  65, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// CO2eq
	snprintf(buffer, sizeof(buffer), "CO2:  %6u ppm", s.co2_eq_ppm);
	ST7735_WriteString(5,  80, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// Etanol/512.0
	float ethanol = s.scaled_ethanol_signal / 512.0f;
	snprintf(buffer, sizeof(buffer), "EtOH: %6.2f", ethanol);
	ST7735_WriteString(5,  95, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// H2/512.0
	float h2 = s.scaled_h2_signal / 512.0f;
	snprintf(buffer, sizeof(buffer), "H2:   %6.2f", h2);
	ST7735_WriteString(5,  110, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

}

void display_sensor_second(void) {
	char buffer[150];

	// ACS712_current
	snprintf(buffer, sizeof(buffer), "Current: %.3f A ", s.current);
	ST7735_WriteString(5,  5, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// INA333_voltage
	snprintf(buffer, sizeof(buffer), "Voltage: %.3f V ", s.voltage);
	ST7735_WriteString(5,  20, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// PWM "ADC: %.2f%%\r\n",
	snprintf(buffer, sizeof(buffer), "Set Current: %.1fA", s.set_current);
	ST7735_WriteString(5, 35, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	// state
	snprintf(buffer, sizeof(buffer), "state: %s", status[st.battery_state]);
	ST7735_WriteString(5,  50, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void display_menu_main(void) {
	char buffer[50];
    uint8_t first_visible;
    uint8_t ptr_pose;  // pointer position {up, middle, down}

    if (st.menu_current_ptr == MENU_START) {  // first position
    	first_visible = 0;
    	ptr_pose = 0;
    }
    else if (st.menu_current_ptr == MENU_SCREEN_COUNT - 1) {  // last position
    	first_visible = SCREENS_MENU_NUM - SCREENS_MENU_VISIBLE_ITEMS;
        ptr_pose = SCREENS_MENU_VISIBLE_ITEMS - 1;
    }
    else {  // middle position
    	first_visible = st.menu_current_ptr - 2;
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

    /*
     * bottom bar
     */

	uint16_t y_pos = ST7735_HEIGHT - Font_7x10.height;
	snprintf(buffer, sizeof(buffer), "%s", batteries[st.battery_current]);
	ST7735_WriteString(5, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	uint16_t x_pos = (strlen(buffer) + 2) * Font_7x10.width;
	snprintf(buffer, sizeof(buffer), "%.1fA", s.set_current);
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	x_pos = ST7735_WIDTH - img_batery.width - 3;
	y_pos = ST7735_HEIGHT - img_batery.height - 3;
	ST7735_DrawImage(x_pos, y_pos, img_batery.width, img_batery.height, img_batery.data);

	if (st.battery_state == BATTERY_DISCHARGING) {
		x_pos += 2;
		y_pos += 4;
		ST7735_DrawImage(x_pos, y_pos, img_thunder.width, img_thunder.height, img_thunder.data);
	}
	else if (st.battery_state == BATTERY_CHARGING) {
		x_pos += 3;
		y_pos += 4;
		ST7735_DrawImage(x_pos, y_pos, img_charging.width, img_charging.height, img_charging.data);
	}
	else {
		x_pos += 3;
		y_pos += img_batery.height - img_bar.height - 3;
		for (uint8_t i = 0; i < 4; ++i) {
			//ST7735_DrawImage(x_pos, y_pos - (i * 4), img_bar.width, img_bar.height, img_bar.data);
			ST7735_FillRectangleFast(x_pos, y_pos - (i * 4), img_bar.width, img_bar.height, ST7735_WHITE);
		}
	}
}

void display_menu_start(void) {
//	char buffer[] = "Press to start";
//	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_7x10.width)) / 2;
//	uint16_t y_pos = (ST7735_WIDTH + Font_7x10.height)/2;
//	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	st.is_enc_pressed = true;

}

void display_menu_battery_type(void) {
	char buffer[50];

    for (uint8_t i = 0; i < BATERYS_NUM; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", batteries[i]);

    	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
    	uint16_t y_pos = (ST7735_HEIGHT - (2 * Font_11x18.height))/2 + (2*Font_11x18.height*i);
    	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
        // cursor
    	if ( st.battery_ptr == i ) {
    		ST7735_FillRectangle(x_pos, y_pos + Font_11x18.height + 1, strlen(buffer)*Font_11x18.width, 2, ST7735_WHITE);
    	}
    }
}

void display_menu_current(void) {
	char buffer[20];

    static uint16_t prev_enc_count = 0;
    float delta = (float)(st.enc_count - prev_enc_count);
    st.set_current_prev += delta * 0.1f;
    st.set_current_prev = fminf(fmaxf(st.set_current_prev, 0.0f), 3.0f);
    prev_enc_count = st.enc_count;

	snprintf(buffer, sizeof(buffer), "%s", "Prad");
	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
	uint16_t y_pos = (ST7735_HEIGHT - (2 * Font_11x18.height))/2;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);

	snprintf(buffer, sizeof(buffer), "%s", "rozladowyw.");
	x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
	y_pos += Font_11x18.height + 2;;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);

	snprintf(buffer, sizeof(buffer), "%.1fA", st.set_current_prev);
	x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
	y_pos = (ST7735_HEIGHT - (2 * Font_11x18.height))/2 + (2*Font_11x18.height);
	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_YELLOW, ST7735_BLACK);

}

void display_menu_state(void) {
	char buffer[50];

    for (uint8_t i = 0; i < STATUS_NUM; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", status[i]);

    	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
    	uint16_t y_pos = i*(Font_11x18.height+26)+26;
    	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
        // cursor
    	if ( st.status_ptr == i ) {
    		ST7735_FillRectangle(x_pos, y_pos + Font_11x18.height + 1, strlen(buffer)*Font_11x18.width, 2, ST7735_WHITE);
    	}
    }
}

void display_menu_auto_mode(void) {
	char buffer[50];

    for (uint8_t i = 0; i < AUTO_MODE_NUM; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", auto_mode[i]);

    	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_11x18.width)) / 2;
    	uint16_t y_pos = (ST7735_HEIGHT - (2 * Font_11x18.height))/2 + (2*Font_11x18.height*i);
    	ST7735_WriteString(x_pos, y_pos, buffer, Font_11x18, ST7735_WHITE, ST7735_BLACK);
        // cursor
    	if ( st.auto_mode_ptr == i ) {
    		ST7735_FillRectangle(x_pos, y_pos + Font_11x18.height + 1, strlen(buffer)*Font_11x18.width, 2, ST7735_WHITE);
    	}
    }
}


void display_menu_stop(void) {
	char buffer[] = "Stop";

	uint16_t x_pos = (ST7735_WIDTH - (strlen(buffer) * Font_7x10.width)) / 2;
	uint16_t y_pos = (ST7735_WIDTH + Font_7x10.height)/2;
	ST7735_WriteString(x_pos, y_pos, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	st.battery_state = BATTERY_IDLE;
	handle_battery_state();
}

/*
 *  click action
 */

void action_menu_main(void) {
	st.menu_current = st.menu_current_ptr;
}

void action_menu_start(void) {
	ST7735_FillScreenFast(ST7735_BLACK);

	// SD
	if (st.is_measurements_started == false) {
		SDcardInit("test.csv");
	}

	st.is_measurements_started = true;

	st.current_screen_type = SCREEN_SENSOR;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_battery_type(void) {
	st.battery_current = st.battery_ptr;

	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_current(void) {
	s.set_current = st.set_current_prev;
	current_filter_reset();

	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_state(void) {
	st.status_current = st.status_ptr;
	st.battery_state = (enum BatteryStatus)(st.status_ptr);

	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_auto_mode(void) {
	st.auto_mode_current = (enum AutoMode)(st.auto_mode_ptr);

	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_stop(void) {
	st.is_measurements_started = false;

	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;

	SDcardClose();
}

