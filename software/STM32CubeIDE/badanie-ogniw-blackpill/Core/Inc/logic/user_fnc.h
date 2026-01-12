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

#define N_SAMPLES 16

extern float adc_buffer[N_SAMPLES];
extern uint8_t sample_idx;

float current_filtered_read(void);
void get_current_charge_val();
void current_filter_reset(void);
void read_sensors_data(void);
void control_battery_state(float *voltage, float *current);
void handle_battery_state(void);
void ENC_SetPosition(int8_t pos);


#endif /* INC_LOGIC_USER_FNC_H_ */
