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
	 */

	float adc_val = (float)ADC_Convert_Channel(ADC_CHANNEL_8);
	//float current = 0.007536f * adc_val - 15.76f;4095.0f
	float voltage = adc_val * 3.3f / 4095.0f; // adc * Vref / 2^12
	float current = (voltage - 2.5f) / 0.185f; // (V - Voff) / ACS_Sensitivity

	// Current filtering
	adc_buffer[sample_idx] = current;
	sample_idx = (sample_idx + 1) % N_SAMPLES;
	float sum = 0;
	for (uint8_t i = 0; i < N_SAMPLES; i++)
		sum += adc_buffer[i];

	return sum / (float)N_SAMPLES;
}

void get_current_charge_val() {
	st.get_current_charge = (ADC_Convert_Channel(ADC_CHANNEL_1) * 100.0f) / 4095.0f;
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
	//s.voltage = 0.001361f * ADC_Convert_Channel(ADC_CHANNEL_9) - 0.346f;
	s.voltage = 2.0f * ADC_Convert_Channel(ADC_CHANNEL_9) * 3.3f / 4095.0f;

	// moving average filter
	s.current = current_filtered_read();

	// charge potentiometer
	get_current_charge_val();


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
	sgp_measure_iaq_blocking_read(&s.tvoc_ppb, &s.co2_eq_ppm);
	sgp_measure_signals_blocking_read(&s.scaled_ethanol_signal, &s.scaled_h2_signal);
	sgp_set_absolute_humidity((uint32_t)s.BME280humidity);

	// PWM discharge
	/*
	 * 0.1A   0.2A   0.3A   0.4A   0.5A
	 * 0.1052 0.2050 0.3050 0.4047 0.5040
	 */
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((st.set_current_discharge / 3.3f) * (float)htim3.Init.Period));


	// PWM charge
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)((st.set_current_charge / 3.3f) * (float)htim4.Init.Period));

}

#define MAXtemperature 60.0
void control_battery_state(float *voltage, float *current, float *temperature) {
	if(st.auto_mode_current == MANUAL_MODE) {
		return;
	}

	static uint8_t out_of_range_counter = 0;

	switch (st.battery_state) {
		case BATTERY_IDLE:
			break;

		case BATTERY_CHARGING:
			if (abs(*current) < st.charge_end_current) { // current condition
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
					st.battery_state = BATTERY_CHARGING;
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

void handle_battery_state() {
	switch (st.battery_state) {
	case BATTERY_IDLE:
		st.discharge_relay = false;
		st.charge_relay = false;
		break;

	case BATTERY_CHARGING:
		st.discharge_relay = false;
		st.charge_relay = true;
		break;

	case BATTERY_DISCHARGING:
		st.discharge_relay = true;
		st.charge_relay = false;
		break;
	}
	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, !st.charge_relay);
	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, !st.discharge_relay);
}

void ENC_SetPosition(int8_t pos) {
    __HAL_TIM_SET_COUNTER(&htim1, (int16_t)(pos << 2));
    st.enc_count = pos;
}
