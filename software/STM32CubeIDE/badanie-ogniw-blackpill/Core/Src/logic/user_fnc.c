/*
 * user_fnc.c
 *
 *  Created on: Sep 28, 2025
 *      Author: Dominik
 */

#include "logic/user_fnc.h"
#include <stdlib.h>

static float current_filtered = 0.0f;

float current_filtered_read(void) {
	/* IN9
	 * ACS712
	 * restart - set_current_filtering(val)
	 */
	#define CURRENT_A			(-0.0124104129f)
	#define CURRENT_B			(25.1413510889f)
	#define CURRENT_ALPHA		(0.02f)

	float adc = (float)ADC_Convert_Channel(ADC_CHANNEL_9);

	float current = CURRENT_A * adc + CURRENT_B;

	/* IIR filtering */
	current_filtered += CURRENT_ALPHA *
					   (current - current_filtered);

	return current_filtered;
}

float voltage_read(void) {
	#define ADC_3V   1950.0f
	#define ADC_4V2  2630.0f
	#define V_3V     3.0f
	#define V_4V2    4.2f
	#define V_a  ((V_4V2 - V_3V) / (ADC_4V2 - ADC_3V))
	#define V_b  (V_3V - V_a * ADC_3V)

	return (V_a * (float)ADC_Convert_Channel(ADC_CHANNEL_8) + V_b);
}

void set_current_filtering(float val) {
	current_filtered = -val;
}

void get_current_charge_val(void)
{
	float adc = ADC_Convert_Channel(ADC_CHANNEL_0);
	float a;
	float b;
	if (st.battery_current == 1) {	// li-ion
		//50, 1400, 1653, 1818, 2120, 2400, 2615, 2900, 3162, 3377, 3648
		//1.3, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3 ,0.2, 0.1
		a =  -0.0003495841f;
		b =  1.4172358277f;
	} else {	// li-pol
		//1750.0f, 2100.0f, 2320.0f, 2560.0f, 2800.0f, 3050.0f, 3290.0f, 3523.0f, 3750.0f
		//0.9f, 0.8f, 0.7f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f, 0.1f
		a = -0.0004074038f;
		b =  1.6381505139f;
	}

	float current = a * adc + b;

	st.get_current_charge = current;
}


void read_sensors_data(void) {
	/* IN8
	 * INA333
	 * 3.3636V -> adc 3.44
	 */
	// ADC
	s.voltage = voltage_read();				// battery voltage
	//2025.5 -- 0.0A
	//2019.5 -- 0.1A
	//2008.5 -- 0.2A
	//1999.0 -- 0.3A
	//1994.5 -- 0.4A
	//1985.5 -- 0.5A
	//1980.5 -- 0.6A
	//1968.0 -- 0.7A
	s.current = -current_filtered_read();	// moving average filter

	// charge potentiometer
	get_current_charge_val();

	// lipol or liion
	BATT_state();


	// BMP
	for (uint8_t index = 0; index < BME_SENSOR_COUNT; ++index) {
		if (index == 0){
			BME280_ReadTemperatureAndPressureAndHuminidity(&s.BME280temperature[index], &s.BME280pressure[index], &s.BME280humidity, index);
		}
		else {
			BME280_ReadTemperatureAndPressure(&s.BME280temperature[index], &s.BME280pressure[index], index);
		}

	}

	// SGP
	sgp_set_absolute_humidity((uint32_t)absolute_humidity_calc(s.BME280humidity, s.BME280temperature[0]));
	sgp_measure_iaq_blocking_read(&s.tvoc_ppb, &s.co2_eq_ppm);
	sgp_measure_signals_blocking_read(&s.scaled_ethanol_signal, &s.scaled_h2_signal);

	// PWM discharge
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((st.set_current_discharge / 3.3f) * (float)htim3.Init.Period));


	// PWM charge
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)((st.set_current_charge / 3.3f) * (float)htim4.Init.Period));

}

#define MAXtemperature 50.0
void control_battery_state(float *voltage, float *current, float *temperature) {
	static uint8_t out_of_range_counter = 0;

	switch (st.battery_state) {
		case BATTERY_IDLE:
			break;

		case BATTERY_CHARGING:
			if (fabsf(*current) < st.charge_end_current) { // current condition
				if (out_of_range_counter >= 3) {
					out_of_range_counter = 0;
					st.battery_state = BATTERY_DISCHARGING;
				}
				else {
					out_of_range_counter++;
				}
			}
			else if(*temperature > MAXtemperature){ // temperature condition
				out_of_range_counter = 0;
				st.battery_state = BATTERY_IDLE;
			}
			else if (*voltage < 2.0f) { // no battery detected
				out_of_range_counter = 0;
				st.battery_state = BATTERY_IDLE;
			}
			else {
				out_of_range_counter = 0;
			}
			break;

		case BATTERY_DISCHARGING:
			if (*voltage < st.discharge_cutoff_voltage) { // voltage condition
				if (out_of_range_counter >= 3) {
					out_of_range_counter = 0;
					if(st.auto_mode_current == MANUAL_MODE) {
						st.battery_state = BATTERY_IDLE;
					}
					else {
						st.battery_state = BATTERY_CHARGING;
					}
				}
				else {
					out_of_range_counter++;
				}
			}
			else if(*temperature > MAXtemperature){ // temperature condition
				out_of_range_counter = 0;
				st.battery_state = BATTERY_IDLE;
			}
			else {
				out_of_range_counter = 0;
			}
			break;
	}
}


void handle_battery_state_dead_time(uint8_t *active_state, uint8_t *pending_state, uint8_t *switch_delay) {
	(*switch_delay)--;

	st.battery_state = BATTERY_IDLE;
	st.charge_relay = false;
	st.discharge_relay = false;
	set_current_filtering(0.0f);
	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, false);
	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, false);

	if (*switch_delay == 0) {
		*active_state = *pending_state;
		st.battery_state = *active_state;

		if (*active_state == BATTERY_CHARGING) {
			set_current_filtering(st.get_current_charge);
		} else if (*active_state == BATTERY_DISCHARGING) {
			set_current_filtering(-st.set_current_discharge);
		} else {
			set_current_filtering(0.0f);
		}
	}
}

bool handle_battery_state_decision(uint8_t *active_state, uint8_t *pending_state, uint8_t *switch_delay) {
	uint8_t requested_state = st.battery_state;

	if (requested_state != *active_state) {
		if ((*active_state == BATTERY_CHARGING && requested_state == BATTERY_DISCHARGING) ||
			(*active_state == BATTERY_DISCHARGING && requested_state == BATTERY_CHARGING)) {			*pending_state = requested_state;	// CHARGE <-> DISCHARGE : dead-time
			*switch_delay = 10;
			return false;

		} else {
			*active_state = requested_state;	//IDLE <-> CHARGE|DISCHARGE

			if (requested_state == BATTERY_CHARGING) {
				set_current_filtering(st.get_current_charge);
			} else if (requested_state == BATTERY_DISCHARGING) {
				set_current_filtering(-st.set_current_discharge);
			} else {
				set_current_filtering(0.0f);
			}
		}
	}
	return true;
}

void handle_battery_state(void) {
	static uint8_t active_state = BATTERY_IDLE;
	static uint8_t pending_state = BATTERY_IDLE;
	static uint8_t switch_delay = 0;

	/* ---------------- DEAD TIME ---------------- */

	if (switch_delay > 0) {
		handle_battery_state_dead_time(&active_state, &pending_state, &switch_delay);
		return;
	}

	/* ---------------- DECISION ---------------- */

	control_battery_state(&s.voltage, &s.current, &s.BME280temperature[0]);
	if (!handle_battery_state_decision(&active_state, &pending_state, &switch_delay)) {
		return;
	}

	/* ---------------- APPLY ---------------- */

	st.battery_state = active_state;
	switch (active_state) {
		case BATTERY_IDLE:
			st.charge_relay = false;
			st.discharge_relay = false;
			break;

		case BATTERY_CHARGING:
			st.charge_relay = true;
			st.discharge_relay = false;
			break;

		case BATTERY_DISCHARGING:
			st.charge_relay = false;
			st.discharge_relay = true;
			break;

		default:
			st.charge_relay = false;
			st.discharge_relay = false;
			active_state = BATTERY_IDLE;
			st.battery_state = BATTERY_IDLE;
			break;
	}

	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, st.charge_relay);
	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, st.discharge_relay);
}


void ENC_SetPosition(int8_t pos) {
    __HAL_TIM_SET_COUNTER(&htim1, (int16_t)(pos << 2));
    st.enc_count = pos;
}

void BATT_state(void) {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        st.battery_current = 1;
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
        st.battery_current = 2;
    else
        st.battery_current = 0;
}

float absolute_humidity_calc(float RH, float T) {
	return 216.78f*((RH/100.0f)*6.112f*expf((17.62f*T)/(243.12f+T)))/(273.15f+T);
}
