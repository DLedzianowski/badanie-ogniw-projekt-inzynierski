/*
 * user_fnc.h
 *
 *  Created on: Sep 28, 2025
 *      Author: Dominik
 */

#ifndef INC_LOGIC_USER_FNC_H_
#define INC_LOGIC_USER_FNC_H_

#include "main.h"
#include "adc.h"
#include "tim.h"

#include "sensors/BMPXX80.h"
#include "sensors/sgp30.h"
#include "sensors/INA219.h"

extern INA219_t myina219;

void get_adc_percentage(void);
void set_adc_percentage(float val);
void read_sensors_data(void);
void control_battery_state(struct state *st, uint16_t *INA219_Voltage);
void handle_battery_state(struct state *st);

#endif /* INC_LOGIC_USER_FNC_H_ */
