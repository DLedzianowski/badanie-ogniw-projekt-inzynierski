/*
 * user_OLED.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Dominik
 */

#ifndef SRC_LOGIC_USER_OLED_H_
#define SRC_LOGIC_USER_OLED_H_

#include "main.h"
#include "sensors/st7735.h"
#include "logic/user_fnc.h"
#include "logic/user_SDcard.h"

void OLED_manage(void);


typedef void (*MenuFunc_t)(void);
extern MenuFunc_t menu_display[];
void display_menu_main(void);
void display_menu_start(void);
void display_menu_battery_type(void);
void display_menu_current(void);
void display_menu_state(void);
void display_menu_auto_mode(void);
void display_menu_stop(void);

extern MenuFunc_t menu_actions[];
void action_menu_main(void);
void action_menu_start(void);
void action_menu_battery_type(void);
void action_menu_current(void);
void action_menu_state(void);
void action_menu_auto_mode(void);
void action_menu_stop(void);

typedef void (*SensorsFunc_t)(void);
extern SensorsFunc_t sensor_display[];
void display_sensor_first(void);
void display_sensor_second(void);


#endif /* SRC_LOGIC_USER_OLED_H_ */
