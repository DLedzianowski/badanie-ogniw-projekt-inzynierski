/*
 * user_OLED.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#ifndef SRC_LOGIC_USER_OLED_H_
#define SRC_LOGIC_USER_OLED_H_

#include "main.h"
#include "sensors/ILI9341_STM32_Driver.h"
#include "sensors/ILI9341_GFX.h"
#include "logic/user_fnc.h"
#include "logic/user_SDcard.h"

typedef void (*MenuFunc_t)(void);

void OLED_manage(void);
void OLED_options_init(void);
extern MenuFunc_t OLED_display_init[];
extern MenuFunc_t OLED_action_init[];


typedef void (*SensorsFunc_t)(void);
extern SensorsFunc_t sensor_display[];
void display_sensor_first(void);
void display_sensor_second(void);

extern MenuFunc_t menu_display[];
void display_menu_main(void);
void display_menu_start(void);
void display_menu_battery_type(void);
void display_menu_current_discharge(void);
void display_menu_current_charge(void);
void display_menu_state(void);
void display_menu_auto_mode(void);
void display_menu_stop(void);

extern MenuFunc_t menu_actions[];
void action_menu_main(void);
void action_menu_start(void);
void action_menu_battery_type(void);
void action_menu_current_discharge(void);
void action_menu_current_charge(void);
void action_menu_state(void);
void action_menu_auto_mode(void);
void action_menu_stop(void);


void display_bottom_bar(void);

#endif /* SRC_LOGIC_USER_OLED_H_ */
