/*
 * user_fnc.c
 *
 *  Created on: Sep 28, 2025
 *      Author: Dominik
 */

#include "logic/user_fnc.h"

INA219_t myina219;

void get_adc_percentage(void) {
	uint16_t adc_position;

	adc_position = ADC_Convert_Channel(ADC_CHANNEL_9);  // potentiometer %
	adc_position = ((adc_position - 250.0f) / (3600.0f - 250.0f)) * 100.0f;
	adc_position = adc_position - fmodf(adc_position, 5.0f);
	s.adc_percentage = fminf(fmaxf(adc_position, 0.0f), 100.0f);
}

void set_adc_percentage(float val) {
	s.adc_percentage = val;
}

void read_sensors_data(void) {
	// ADC
//	get_adc_percentage();
	// PWM
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, s.adc_percentage*10);

	// BMP
	for (uint8_t index = 0; index < BMP_SENSOR_COUNT; ++index) {
		BMP280_ReadTemperatureAndPressure(&s.BMP280temperature[index], &s.BMP280pressure[index], index);
	}

	// SGP
	sgp_measure_iaq_blocking_read(&s.tvoc_ppb, &s.co2_eq_ppm);
	sgp_measure_signals_blocking_read(&s.scaled_ethanol_signal, &s.scaled_h2_signal);
	//sgp_set_absolute_humidity()

	// INA219
	s.INA219_Current = INA219_ReadCurrent(&myina219);
	s.INA219_Voltage = INA219_ReadBusVoltage(&myina219);
	s.INA219_Power = INA219_ReadPower(&myina219);
}

void control_battery_state(struct state *st, uint16_t *INA219_Voltage) {
	static uint8_t out_of_range_counter = 0;

	switch (st->battery_state) {
		case BATTERY_IDLE:
			break;

		case BATTERY_CHARGING:
			if (*INA219_Voltage > 4200 || *INA219_Voltage < 2900) {
				// if event ocurced
				if (out_of_range_counter > 2) {
					out_of_range_counter = 0;
					if (*INA219_Voltage > 4200) {
						st->battery_state = BATTERY_DISCHARGING;
					}
					if (*INA219_Voltage < 2900) {
						st->battery_state = BATTERY_IDLE;
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
			if (*INA219_Voltage < 3000 || *INA219_Voltage > 4300) {
				if (out_of_range_counter > 2) {
					out_of_range_counter = 0;
					if (*INA219_Voltage < 3000) {
						st->battery_state = BATTERY_CHARGING;
					}
					if (*INA219_Voltage > 4300) {
						st->battery_state = BATTERY_IDLE;
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

void handle_battery_state(struct state *st) {
	switch (st->battery_state) {
	case BATTERY_IDLE:
		st->discharging_relay = false;
		st->charging_relay = false;
		break;

	case BATTERY_CHARGING:
		st->discharging_relay = false;
		st->charging_relay = true;
		break;

	case BATTERY_DISCHARGING:
		st->discharging_relay = true;
		st->charging_relay = false;
		break;
	}
	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, !st->charging_relay);
	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, !st->discharging_relay);
}
