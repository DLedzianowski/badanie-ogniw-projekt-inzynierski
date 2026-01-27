/*
 * user_fnc.c
 *
 *  Created on: Sep 28, 2025
 *      Author: Dominik
 */

#include "logic/user_fnc.h"
#include <stdlib.h>

float adc_buffer[N_SAMPLES] = {0};
uint8_t sample_idx = 0;

float current_filtered_read(void) {
	/* IN9
	 * ACS712
	 */
	//float current = (float)ADC_Convert_Channel(ADC_CHANNEL_9);
//2042 0A
//2005 0.5A
//2080 -0.5A
	#define ADC_ZERO   2042.0f
	#define ADC_PER_A  (37.0f / 0.5f)

	float adc = (float)ADC_Convert_Channel(ADC_CHANNEL_9);
	float current = (adc - ADC_ZERO) / ADC_PER_A;

	//float adc_val = (float)ADC_Convert_Channel(ADC_CHANNEL_9);
	//float voltage = adc_val * 3.3f / 4095.0f; // adc * Vref / 2^12
	//float current = (voltage - 1.65f) / 0.0625f; // (V - Voff) / ACS_Sensitivity(0.625/10)

	// Current filtering
	adc_buffer[sample_idx] = current;
	sample_idx = (sample_idx + 1) % N_SAMPLES;
	float sum = 0;
	for (uint8_t i = 0; i < N_SAMPLES; i++)
		sum += adc_buffer[i];

	return sum / (float)N_SAMPLES;
}

void get_current_charge_val(void){
	static const float adc_points[8] = {
		40.0f, 200.0f, 430.0f, 660.0f,
		850.0f, 1130.0f, 1600.0f, 3000.0f
	};

	static const float current_points[8] = {
		0.05f, 0.15f, 0.28f, 0.40f,
		0.55f, 0.70f, 0.85f, 0.95f
	};

    float adc = 4095.0f - ADC_Convert_Channel(ADC_CHANNEL_0);
    float current = current_points[0];

    // ograniczenia
    if (adc <= adc_points[0]) {
        current = current_points[0];
    }
    else if (adc >= adc_points[7]) {
        current = current_points[7];
    }
    else {
        for (int i = 0; i < 7; i++) {
            if (adc <= adc_points[i + 1]) {
                float x0 = adc_points[i];
                float x1 = adc_points[i + 1];
                float y0 = current_points[i];
                float y1 = current_points[i + 1];

                current = y0 + (adc - x0) * (y1 - y0) / (x1 - x0);
                break;
            }
        }
    }

    st.get_current_charge = current;
}

void current_filter_reset(void) {
	for (uint8_t i = 0; i < N_SAMPLES; i++) {
		adc_buffer[i] = 0;
	}
	sample_idx = 0;
}

void read_sensors_data(void) {
	/* IN8
	 * INA333
	 * 3.3636V -> adc 3.44
	 */
	// ADC
	#define ADC_3V   1950.0f
	#define ADC_4V2  2550.0f
	#define V_3V     3.0f
	#define V_4V2    4.2f
	#define V_a  ((V_4V2 - V_3V) / (ADC_4V2 - ADC_3V))
	#define V_b  (V_3V - V_a * ADC_3V)

	s.voltage = V_a * (float)ADC_Convert_Channel(ADC_CHANNEL_8) + V_b;
	
	// moving average filter
	s.current = current_filtered_read();

	// charge potentiometer
	get_current_charge_val();

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

void handle_battery_state() {
	static uint8_t prev_state = BATTERY_IDLE;
    if (st.battery_state != prev_state) {
        prev_state = st.battery_state;

    	HAL_GPIO_WritePin(R1IN1_GPIO_Port, R1IN1_Pin, false);
    	HAL_GPIO_WritePin(R1IN2_GPIO_Port, R1IN2_Pin, false);
        return;
    }

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
	default:
		st.discharge_relay = false;
		st.charge_relay = false;
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
