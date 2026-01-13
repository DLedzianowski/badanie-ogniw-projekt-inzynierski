/*
 * user_OLED.c
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#include "logic/user_OLED.h"

uint8_t set_current_discharge_first_enter = true;
uint8_t set_current_charge_first_enter = true;
uint8_t charge_end_current_first_enter = true;
uint8_t discharge_cutoff_voltage_first_enter = true;

MenuFunc_t OLED_display_init[] = {
	// display_menu_current_charge,
	display_menu_min_charge_current,
	display_menu_min_discharge_voltage,
	display_menu_current_discharge,
	display_menu_state,
	display_menu_auto_mode
};
#define OLED_OPTIONS_INIT_COUNT	(sizeof(OLED_display_init) / sizeof(OLED_display_init[0]))

MenuFunc_t OLED_action_init[] = {
	// 	action_menu_current_charge,
	action_menu_min_charge_current,
	action_menu_min_discharge_voltage,
	action_menu_current_discharge,
	action_menu_state,
	action_menu_auto_mode
};


SensorsFunc_t sensor_display[] = {
    display_sensor_first,
    display_sensor_second
};

MenuFunc_t menu_display[] = {
	display_menu_main,
	display_menu_start,
	//display_menu_battery_type,
	//display_menu_current_charge,
	display_menu_min_charge_current,
	display_menu_min_discharge_voltage,
	display_menu_current_discharge,
	display_menu_state,
	display_menu_auto_mode,
	display_menu_stop
};

MenuFunc_t menu_actions[] = {
	action_menu_main,
	action_menu_start,
	//action_menu_battery_type,
	//action_menu_current_charge,
	action_menu_min_charge_current,
	action_menu_min_discharge_voltage,
	action_menu_current_discharge,
	action_menu_state,
	action_menu_auto_mode,
	action_menu_stop
};

void OLED_manage(void) {
	switch (st.current_screen_type) {
	case SCREEN_MENU:
		if (st.is_enc_pressed) {
			menu_actions[st.menu_current]();
			//st.enc_offset = __HAL_TIM_GET_COUNTER(&htim1) / 4;
			st.is_enc_pressed = false;
			st.screen_clear = true;
		}
		if (st.screen_clear == true && st.current_screen_type == SCREEN_MENU) {
			st.screen_clear = false;
			ILI9341_FillScreen(BLACK);

			menu_display[st.menu_current]();
		}
	    break;

	case SCREEN_SENSOR:
		if (st.screen_clear == true) {
			st.screen_clear = false;
			ILI9341_FillScreen(BLACK);

			int16_t idx = st.enc_count % SENSOR_SCREEN_COUNT;
			if (idx < 0)
				idx += SENSOR_SCREEN_COUNT;
			st.sensor_current = (enum SensorScreen)idx;
		}

		sensor_display[st.sensor_current]();

		if (st.is_enc_pressed) {
			ENC_SetPosition(0);
			st.is_enc_pressed = false;
			st.current_screen_type = SCREEN_MENU;
			st.screen_clear = true;
		}
	    break;
	default:
		break;
	}
}

void OLED_options_init(void) {
    st.screen_clear = true;
	for(uint8_t i = 0; i < OLED_OPTIONS_INIT_COUNT; i++) {
		ENC_SetPosition(0);
		st.menu_current_ptr = i;
		st.screen_clear = 1;

		while(1) {
			if(st.screen_clear) {
				st.screen_clear = 0;
				ILI9341_FillScreen(BLACK);
				OLED_display_init[st.menu_current_ptr]();
			}

			if(st.is_enc_pressed) {
				st.is_enc_pressed = 0;
				OLED_action_init[st.menu_current_ptr]();
				break;
			}
			HAL_Delay(50);
		}
	}
	ENC_SetPosition(0);
	st.screen_clear = true;
	OLED_manage();
}

void display_sensor_first(void) {
	char buffer[150];
	uint8_t x;

	// Temperatura 1
	snprintf(buffer, sizeof(buffer), "Temp. ogniwa1: %5.2f C ", s.BME280temperature[0]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 5, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5, 5, buffer, FONT4, WHITE, BLACK);

	// Temperatura 2
	snprintf(buffer, sizeof(buffer), "Temp. ogniwa2: %5.1f C ", s.BME280temperature[1]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 30, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5, 30, buffer, FONT4, WHITE, BLACK);

	// Temperatura 3
	snprintf(buffer, sizeof(buffer), "Temp. tla: %5.1f C ", s.BME280temperature[2]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 60, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5, 60, buffer, FONT4, WHITE, BLACK);

	// Wilgotność
	snprintf(buffer, sizeof(buffer), "Wilgotnosc:  %4.1f %%", s.BME280humidity);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 90, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  90, buffer, FONT4, WHITE, BLACK);

	// TVOC
	snprintf(buffer, sizeof(buffer), "TVOC: %6u ppb", s.tvoc_ppb);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 120, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  120, buffer, FONT4, WHITE, BLACK);

	// CO2eq
	snprintf(buffer, sizeof(buffer), "CO2:  %6u ppm", s.co2_eq_ppm);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 150, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  150, buffer, FONT4, WHITE, BLACK);

	// Etanol/512.0
//	float ethanol = s.scaled_ethanol_signal / 512.0f;
//	snprintf(buffer, sizeof(buffer), "EtOH: %6.2f", ethanol);
//	ILI9341_DrawText(5,  95, buffer, FONT4, WHITE, BLACK);

	// H2/512.0
//	float h2 = s.scaled_h2_signal / 512.0f;
//	snprintf(buffer, sizeof(buffer), "H2:   %6.2f", h2);
//	ILI9341_DrawText(5,  110, buffer, FONT4, WHITE, BLACK);
	// ACS712_current
	snprintf(buffer, sizeof(buffer), "Prad: %.2f A ", s.current);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 180, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  180, buffer, FONT4, WHITE, BLACK);

	// INA333_voltage
	snprintf(buffer, sizeof(buffer), "Napiecie ogniwa: %.2f V ", s.voltage);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 210, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  210, buffer, FONT4, WHITE, BLACK);

	// PWM "ADC: %.2f%%\r\n",
	snprintf(buffer, sizeof(buffer), "Prad ladowania: %.1fA", st.set_current_charge);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 240, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5, 240, buffer, FONT4, WHITE, BLACK);

	// PWM "ADC: %.2f%%\r\n",
	snprintf(buffer, sizeof(buffer), "Prad rozladowania: %4.1f%%", st.get_current_charge);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 270, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5, 270, buffer, FONT4, WHITE, BLACK);



}

void display_sensor_second(void) {
	char buffer[150];
	uint8_t x;

	// state
	snprintf(buffer, sizeof(buffer), "Aktualny stan: %s", status[st.battery_state]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 5, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  5, buffer, FONT4, WHITE, BLACK);

	// Battery current
	snprintf(buffer, sizeof(buffer), "Aktualne ogniwo: %s", (st.battery_current == 0) ? "Brak" : batteries[st.battery_current]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 30, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  30, buffer, FONT4, WHITE, BLACK);

	// Current auto mode
	snprintf(buffer, sizeof(buffer), "Aktualny tryb:%s", auto_mode[st.auto_mode_current]);
	x = ILI9341_GetTextWidth(buffer, FONT4);
	ILI9341_DrawRectangle(x+5, 60, ILI9341_SCREEN_WIDTH-x-5, FONT4h, BLACK);
	ILI9341_DrawText(5,  60, buffer, FONT4, WHITE, BLACK);
}

#define MENU_SCROLLBAR_X	(ILI9341_SCREEN_WIDTH - 10)
#define MENU_DOT_STEP		15

void display_menu_main(void) {
	char buffer[50];
    uint8_t first_visible;
    uint8_t ptr_pose;  // pointer position {up, middle, down}

    int16_t idx = st.enc_count % SCREENS_MENU_NUM;
    if (idx < 0)
        idx += SCREENS_MENU_NUM;

    st.menu_current_ptr = (enum MenuScreen)(MENU_START + idx);
    uint8_t menu_index = st.menu_current_ptr - MENU_START;


	if (menu_index <= 2) {
		first_visible = 0;
		ptr_pose = menu_index;
	}
	else if (menu_index == 3) {
		first_visible = 1;
		ptr_pose = 2;
	}
	else {
		first_visible = SCREENS_MENU_NUM - SCREENS_MENU_VISIBLE_ITEMS;
		ptr_pose = menu_index - first_visible;

		if (ptr_pose >= SCREENS_MENU_VISIBLE_ITEMS) {
			ptr_pose = SCREENS_MENU_VISIBLE_ITEMS - 1;
		}
	}

	uint16_t slot_h = (ILI9341_SCREEN_HEIGHT-50) / SCREENS_MENU_VISIBLE_ITEMS;
	uint16_t y_offset = (slot_h - FONT4h) / 2;

	for (uint8_t i = 0; i < SCREENS_MENU_VISIBLE_ITEMS; i++) {
		snprintf(buffer, sizeof(buffer), "%s", menu[first_visible + i]);


		uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
		uint16_t y_pos = (i * slot_h) + y_offset;

		ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

		if (ptr_pose == i) {
			ILI9341_DrawRectangle(x_pos, y_pos + FONT4h + 1, ILI9341_GetTextWidth(buffer, FONT4), 2, WHITE);
		}
	}

	uint16_t dots_block_h = (SCREENS_MENU_NUM - 1) * MENU_DOT_STEP;
	uint16_t dots_start_y = ((ILI9341_SCREEN_HEIGHT-40) - dots_block_h) / 2;

	for (uint8_t i = 0; i < SCREENS_MENU_NUM; i++) {
		uint16_t dot_y = dots_start_y + (i * MENU_DOT_STEP);

		if (i == menu_index) {
			ILI9341_DrawFilledCircle(MENU_SCROLLBAR_X, dot_y, 3, WHITE);
		}
		else {
			ILI9341_DrawHollowCircle(MENU_SCROLLBAR_X, dot_y, 2, WHITE);
		}
	}
    /*
     * bottom bar
     */
	display_bottom_bar();
}

void display_menu_start(void) {
//	char buffer[] = "Press to start";
//	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
//	uint16_t y_pos = (ILI9341_SCREEN_WIDTH + FONT4h)/2;
//	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	st.is_enc_pressed = true;

}

//void display_menu_battery_type(void) {
//	char buffer[50];
//
//
//  int16_t idx = st.enc_count % BATERYS_NUM;
//	if (idx < 0)
//	    idx += BATERYS_NUM;
//	st.battery_ptr = (uint8_t)idx;
//
//  for (uint8_t i = 0; i < BATERYS_NUM; i++) {
//		snprintf(buffer, sizeof(buffer), "%s", batteries[i]);
//
//    	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT3)) / 2;
//    	uint16_t y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT3h))/2 + (2*FONT3h*i);
//    	ILI9341_DrawText(x_pos, y_pos, buffer, FONT3, WHITE, BLACK);
//        // cursor
//    	if ( st.battery_ptr == i ) {
//    		ILI9341_DrawRectangle(x_pos, y_pos + FONT3h + 1, ILI9341_GetTextWidth(buffer, FONT3), 2, WHITE);
//    	}
//    }
//}

void display_menu_current_charge(void) {
	char buffer[20];

    static int16_t prev_enc_count = 0;
    float delta = (float)(st.enc_count - prev_enc_count);
    if (set_current_charge_first_enter) {
    	delta = 0;
    	set_current_charge_first_enter = false;
    }
    st.set_current_charge_prev += delta * 0.1f;
    st.set_current_charge_prev = fminf(fmaxf(st.set_current_charge_prev, 0.0f), 3.0f);
    prev_enc_count = st.enc_count;

	snprintf(buffer, sizeof(buffer), "%s", "Prad");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%s", "adowania");
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos += FONT4h + 2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%.1f %%", st.set_current_charge);
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2 + (2*FONT4h);
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, YELLOW, BLACK);
}

void display_menu_min_charge_current(void) {
	char buffer[20];
    static int16_t prev_enc_count = 0;
    float delta = (float)(st.enc_count - prev_enc_count);

    if (charge_end_current_first_enter) {
    	delta = 0;
    	charge_end_current_first_enter = false;
    }
    st.charge_end_current_prev += delta * 0.01f;
    st.charge_end_current_prev = fminf(fmaxf(st.charge_end_current_prev, 0.01f), 0.5f);
    prev_enc_count = st.enc_count;

	snprintf(buffer, sizeof(buffer), "%s", "Minimalny prad");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%s", "adowania");
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos += FONT4h + 2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%.2fA", st.charge_end_current_prev);
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2 + (2*FONT4h);
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, YELLOW, BLACK);

}
void display_menu_min_discharge_voltage(void) {
	char buffer[20];
    static int16_t prev_enc_count = 0;
    float delta = (float)(st.enc_count - prev_enc_count);

    if (discharge_cutoff_voltage_first_enter) {
    	delta = 0;
    	discharge_cutoff_voltage_first_enter = false;
    }
    st.discharge_cutoff_voltage_prev += delta * 0.1f;
    st.discharge_cutoff_voltage_prev = fminf(fmaxf(st.discharge_cutoff_voltage_prev, 2.9f), 4.2f);
    prev_enc_count = st.enc_count;

	snprintf(buffer, sizeof(buffer), "%s", "Minimalne napiecie");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%s", "rozadowania");
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos += FONT4h + 2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%.1fV", st.discharge_cutoff_voltage_prev);
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2 + (2*FONT4h);
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, YELLOW, BLACK);
}

void display_menu_current_discharge(void) {
	char buffer[20];

    static int16_t prev_enc_count = 0;

    float delta = (float)(st.enc_count - prev_enc_count);

    if (set_current_discharge_first_enter) {
    	delta = 0;
    	set_current_discharge_first_enter = false;
    }

    st.set_current_discharge_prev += delta * 0.1f;
    st.set_current_discharge_prev = fminf(fmaxf(st.set_current_discharge_prev, 0.0f), 3.0f);
    prev_enc_count = st.enc_count;

	snprintf(buffer, sizeof(buffer), "%s", "Prad");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%s", "rozadowania");
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos += FONT4h + 2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	snprintf(buffer, sizeof(buffer), "%.1fA", st.set_current_discharge_prev);
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	y_pos = (ILI9341_SCREEN_HEIGHT - (2 * FONT4h))/2 + (2*FONT4h);
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, YELLOW, BLACK);
}

void display_menu_state(void) {
	char buffer[50];

    int16_t idx = st.enc_count % STATUS_NUM;
    if (idx < 0)
        idx += STATUS_NUM;
    st.status_ptr = (uint8_t)idx;

	snprintf(buffer, sizeof(buffer), "Aktualny tryb:");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = 26;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

    for (uint8_t i = 0; i < STATUS_NUM; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", status[i]);

    	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
    	y_pos = i*(FONT4h+25)+40*2;
    	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);
        // cursor
    	if ( st.status_ptr == i ) {
    		ILI9341_DrawRectangle(x_pos, y_pos + FONT4h + 1, ILI9341_GetTextWidth(buffer, FONT4), 2, WHITE);
    	}
    }
}

void display_menu_auto_mode(void) {
	char buffer[50];

    int16_t idx = st.enc_count % AUTO_MODE_NUM;
    if (idx < 0)
        idx += AUTO_MODE_NUM;
    st.auto_mode_ptr = (uint8_t)idx;

	snprintf(buffer, sizeof(buffer), "Sterowanie cyklami:");
	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = 26;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

    for (uint8_t i = 0; i < AUTO_MODE_NUM; i++) {
    	snprintf(buffer, sizeof(buffer), "%s", auto_mode[i]);

    	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
    	y_pos = i*(FONT4h+25)+40*2;
    	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);
        // cursor
    	if ( st.auto_mode_ptr == i ) {
    		ILI9341_DrawRectangle(x_pos, y_pos + FONT4h + 1, ILI9341_GetTextWidth(buffer, FONT4), 2, WHITE);
    	}
    }
}


void display_menu_stop(void) {
	char buffer[] = "Stop";

	uint16_t x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	uint16_t y_pos = (ILI9341_SCREEN_WIDTH + FONT4h)/2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	st.battery_state = BATTERY_IDLE;
	handle_battery_state();
}

/*
 *  click action
 */

void action_menu_main(void) {
	ENC_SetPosition(0);
	st.menu_current = st.menu_current_ptr;
}

void action_menu_start(void) {
	ILI9341_FillScreen(BLACK);

	// SD
	if (st.is_measurements_started == false) {
		SDcardInit("test.csv");
	}

	st.is_measurements_started = true;

	ENC_SetPosition(0);
	st.current_screen_type = SCREEN_SENSOR;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

//void action_menu_battery_type(void) {
//	st.battery_current = st.battery_ptr;
//
//	ENC_SetPosition(1);
//	st.current_screen_type = SCREEN_MENU;
//	st.menu_current_ptr = MENU_BATTERY_TYPE;
//	st.menu_current = MENU_BATTERY_TYPE;
//}

void action_menu_current_charge(void) {
	st.set_current_charge = st.set_current_charge_prev;
	set_current_charge_first_enter = true;
	current_filter_reset();

	ENC_SetPosition(0);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_SET_CURRENT_CHARGE;
	st.menu_current = MENU_MAIN;
}

void action_menu_min_charge_current(void) {
	st.charge_end_current = st.charge_end_current_prev;
	charge_end_current_first_enter = true;

	ENC_SetPosition(1);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_SET_CURRENT_DISCHARGE;
	st.menu_current = MENU_MAIN;

}
void action_menu_min_discharge_voltage(void) {
	st.discharge_cutoff_voltage = st.discharge_cutoff_voltage_prev;
	discharge_cutoff_voltage_first_enter = true;

	ENC_SetPosition(2);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_SET_CURRENT_DISCHARGE;
	st.menu_current = MENU_MAIN;

}

void action_menu_current_discharge(void) {
	st.set_current_discharge = st.set_current_discharge_prev;
	set_current_discharge_first_enter = true;
	current_filter_reset();

	ENC_SetPosition(3);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_SET_CURRENT_DISCHARGE;
	st.menu_current = MENU_MAIN;
}


void action_menu_state(void) {
	st.status_current = st.status_ptr;
	st.battery_state = (enum BatteryStatus)(st.status_ptr);

	ENC_SetPosition(4);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_auto_mode(void) {
	st.auto_mode_current = (enum AutoMode)(st.auto_mode_ptr);

	ENC_SetPosition(5);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;
}

void action_menu_stop(void) {
	st.is_measurements_started = false;

	ENC_SetPosition(0);
	st.current_screen_type = SCREEN_MENU;
	st.menu_current_ptr = MENU_START;
	st.menu_current = MENU_MAIN;

	SDcardClose();
}

void display_bottom_bar(void) {
	char buffer[20];
	uint16_t y_pos = ILI9341_SCREEN_HEIGHT - FONT4h - 2;

	// charge potentiometer
	get_current_charge_val();

	// Arrow up current_charging
	uint16_t x_pos = 5;
	ILI9341_DrawImageWH(x_pos, y_pos, &img_arrowup);
	x_pos += img_arrowup.width + 4;
	snprintf(buffer, sizeof(buffer), "%.1f%%", st.get_current_charge);
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	// Current battery
	snprintf(buffer, sizeof(buffer), "%s", batteries[st.battery_current]);
	x_pos = (ILI9341_SCREEN_WIDTH - ILI9341_GetTextWidth(buffer, FONT4)) / 2;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);

	// Arrow down current_discharging
	snprintf(buffer, sizeof(buffer), "%.1fA", st.set_current_discharge);
	x_pos = ILI9341_SCREEN_WIDTH - (img_arrowdown.width + ILI9341_GetTextWidth(buffer, FONT4) + 4);
	ILI9341_DrawImageWH(x_pos, y_pos, &img_arrowdown);
	x_pos += img_arrowdown.width + 4;
	ILI9341_DrawText(x_pos, y_pos, buffer, FONT4, WHITE, BLACK);
}
