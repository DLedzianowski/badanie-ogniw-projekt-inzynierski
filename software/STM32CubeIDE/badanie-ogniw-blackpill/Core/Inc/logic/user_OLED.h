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


void OLED_manage(struct state *st, struct sensors *s);


typedef void (*MenuFunc_t)(struct state *st);
extern MenuFunc_t menu_display[];
void display_menu_main(struct state *st);
void display_menu_start(struct state *st);
void display_menu_battery_type(struct state *st);
void display_menu_adc(struct state *st);
void display_menu_stop(struct state *st);

extern MenuFunc_t menu_actions[];
void action_menu_main(struct state *st);
void action_menu_start(struct state *st);
void action_menu_battery_type(struct state *st);
void action_menu_adc(struct state *st);
void action_menu_stop(struct state *st);

typedef void (*SensorsFunc_t)(struct sensors *s);
extern SensorsFunc_t sensor_display[];
void display_sensor_first(struct sensors *s);
void display_sensor_second(struct sensors *s);


#endif /* SRC_LOGIC_USER_OLED_H_ */
