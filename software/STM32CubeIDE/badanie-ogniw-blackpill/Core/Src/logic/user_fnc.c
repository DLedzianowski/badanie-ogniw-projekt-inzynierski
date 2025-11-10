/*
 * user_fnc.c
 *
 *  Created on: Sep 28, 2025
 *      Author: Dominik
 */

#include "logic/user_fnc.h"

float adc_buffer[N_SAMPLES] = {0};
uint8_t sample_idx = 0;

float current_filtered_read(void) {
	/* IN8
	 * ACS712
	 * 5%      10%     15%
	 * 0.1553A 0.3106A 0.4718A
	 * 2113   2120   2155
	 */

	float adc_val = (float)ADC_Convert_Channel(ADC_CHANNEL_8);
	float current = 0.007536f * adc_val - 15.76f; // przelicz na prąd
	adc_buffer[sample_idx] = current;
	sample_idx = (sample_idx + 1) % N_SAMPLES;

	float sum = 0;
	for (uint8_t i = 0; i < N_SAMPLES; i++)
		sum += adc_buffer[i];

	return sum / (float)N_SAMPLES;
}

void current_filter_reset(void) {
	for (uint8_t i = 0; i < N_SAMPLES; i++) {
		adc_buffer[i] = 0;
	}
	sample_idx = 0;
}
void read_sensors_data(void) {
	// ADC
	/* ina333 IN9
	 * 4.195V -> raw adc 3337
	 * 3.075V -> raw adc 2514
	 */
	s.voltage = 0.001361f * ADC_Convert_Channel(ADC_CHANNEL_9) - 0.346f;

	// moving average filter
	s.current = current_filtered_read();

	// BMP
	for (uint8_t index = 0; index < BMP_SENSOR_COUNT; ++index) {
		BMP280_ReadTemperatureAndPressure(&s.BMP280temperature[index], &s.BMP280pressure[index], index);
	}

	// SGP
	sgp_measure_iaq_blocking_read(&s.tvoc_ppb, &s.co2_eq_ppm);
	sgp_measure_signals_blocking_read(&s.scaled_ethanol_signal, &s.scaled_h2_signal);
	//sgp_set_absolute_humidity();

	// PWM
	/*
	 * 0.1A   0.2A   0.3A   0.4A   0.5A
	 * 0.1052 0.2050 0.3050 0.4047 0.5040
	 */
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((s.set_current / 3.3f) * (float)htim3.Init.Period));

}

void control_battery_state(float *voltage) {
	if(st.auto_mode_current == MANUAL_MODE) {
		return;
	}

	static uint8_t out_of_range_counter = 0;

	switch (st.battery_state) {
		case BATTERY_IDLE:
			break;

		case BATTERY_CHARGING:
			if (*voltage > 4.2f || *voltage < 2.8f) {
				// if event ocurced
				if (out_of_range_counter > 2) {
					out_of_range_counter = 0;
					if (*voltage > 4.2f) {
						st.battery_state = BATTERY_DISCHARGING;
					}
					if (*voltage < 2.8f) {
						st.battery_state = BATTERY_IDLE;
					}
				}
				else {
					out_of_range_counter++;
				}
			}
			else {
				out_of_range_counter = 0;
			}
			break;

		case BATTERY_DISCHARGING:
			if (*voltage < 3.0f || *voltage > 4.4f) {
				if (out_of_range_counter > 2) {
					out_of_range_counter = 0;
					if (*voltage < 3.0f) {
						st.battery_state = BATTERY_CHARGING;
					}
					if (*voltage > 4.4f) {
						st.battery_state = BATTERY_IDLE;
					}
				}
				else {
					out_of_range_counter++;
				}
			}
			else {
				out_of_range_counter = 0;
			}
			break;
	}
}

void handle_battery_state() {
	switch (st.battery_state) {
	case BATTERY_IDLE:
		st.discharging_relay = false;
		st.charging_relay = false;
		break;

	case BATTERY_CHARGING:
		st.discharging_relay = false;
		st.charging_relay = true;
		break;

	case BATTERY_DISCHARGING:
		st.discharging_relay = true;
		st.charging_relay = false;
		break;
	}
	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, !st.charging_relay);
	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, !st.discharging_relay);
}
