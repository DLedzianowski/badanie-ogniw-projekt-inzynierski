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

float current_filtered_read(void);
float voltage_read(void);
void set_current_filtering(float val);
void get_current_charge_val(void);
void current_filter_reset(void);
void read_sensors_data(void);
void control_battery_state(float *voltage, float *current, float *temperature);
void handle_battery_state(void);
void ENC_SetPosition(int8_t pos);
void BATT_state(void);
float absolute_humidity_calc(float RH, float T);
#endif /* INC_LOGIC_USER_FNC_H_ */
