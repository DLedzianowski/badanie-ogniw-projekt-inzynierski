/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum ScreenType {
    SCREEN_MENU,
    SCREEN_SENSOR,
	SCREEN_TYPE_COUNT
};

enum MenuScreen {
    MENU_MAIN,
    MENU_START,
    MENU_BATTERY_TYPE,
    MENU_SET_CURRENT_CHARGE,
	MENU_SET_CURRENT_DISCHARGE,
	MENU_STATUS,
	MENU_AUTO_MODE,
    MENU_STOP,
	MENU_SCREEN_COUNT
};

enum SensorScreen {
    SENSOR_FIRST,
    SENSOR_SECOND,
	SENSOR_SCREEN_COUNT
};

enum BatteryStatus {
	BATTERY_IDLE,
	BATTERY_CHARGING,
	BATTERY_DISCHARGING
};

enum AutoMode {
	AUTO_MODE,
	MANUAL_MODE
};

struct state {
    enum ScreenType current_screen_type;
    enum MenuScreen menu_current;
    enum MenuScreen menu_current_ptr;
    enum SensorScreen sensor_current;
    enum BatteryStatus battery_state;
    enum AutoMode auto_mode_current;

	uint8_t screen_menu_ptr;
	uint8_t screen_menu_current;
	uint8_t battery_ptr;
	uint8_t battery_current;
	uint8_t status_ptr;
	uint8_t status_current;
	uint8_t auto_mode_ptr;
	//uint8_t auto_mode_current;
	int16_t enc_count;
	uint16_t prev_enc_count;
	uint16_t enc_offset;
    float set_current_discharge;
    float set_current_discharge_prev;
    float set_current_charge;
    float set_current_charge_prev;
    float get_current_charge;
    float charge_end_current;
    float charge_end_current_prev;
    float discharge_cutoff_voltage;
    float discharge_cutoff_voltage_prev;

    bool _interrupt_flag;
	bool is_measurements_started;
	bool is_enc_pressed;
	bool is_screen_menu;
	bool screen_clear;
	bool discharge_relay;
	bool charge_relay;
};
extern struct state st;

// Struktura z pomiarami
#define BME_SENSOR_COUNT 3
struct sensors {
    float BME280temperature[BME_SENSOR_COUNT];
    int32_t BME280pressure[BME_SENSOR_COUNT];
    float BME280humidity;

    uint16_t tvoc_ppb;
    uint16_t co2_eq_ppm;
    uint16_t scaled_ethanol_signal;
    uint16_t scaled_h2_signal;

    float voltage;
    float current;
};
extern struct sensors s;

#define BATERYS_NUM 3
extern const char* batteries[BATERYS_NUM];

#define STATUS_NUM 3
extern const char* status[STATUS_NUM];

#define AUTO_MODE_NUM 2
extern const char* auto_mode[AUTO_MODE_NUM];

#define SCREENS_MENU_VISIBLE_ITEMS 5  // number of visible opcions in menu
#define SCREENS_MENU_NUM 7  // numbers of all menu elements
extern const char* menu[SCREENS_MENU_NUM];
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define RETRY_DELAY_MS 1000
#define SCREENS_SENSORS_NUM 2  // number of screens with sensors data

#define ENABLE_DEBUG   1
#define ENABLE_DATA    1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#if ENABLE_DEBUG
	#define ANSI_CYAN    "\033[36m"
	#define LOG_DEBUG(msg, ...) (printf(ANSI_CYAN "[DEBUG] " msg, ##__VA_ARGS__))
#else
	#define LOG_DEBUG(msg, ...)
#endif

#if ENABLE_DATA
	#define ANSI_YELLOW  "\033[33m"
	#define LOG_DATA(msg, ...)  (printf(ANSI_YELLOW "[DATA] " msg, ##__VA_ARGS__))
#else
	#define LOG_DATA(msg, ...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define oled_CS_Pin GPIO_PIN_1
#define oled_CS_GPIO_Port GPIOA
#define oled_RST_Pin GPIO_PIN_2
#define oled_RST_GPIO_Port GPIOA
#define oled_DC_Pin GPIO_PIN_3
#define oled_DC_GPIO_Port GPIOA
#define sd_CS_Pin GPIO_PIN_4
#define sd_CS_GPIO_Port GPIOA
#define R1IN1_Pin GPIO_PIN_12
#define R1IN1_GPIO_Port GPIOB
#define R1IN2_Pin GPIO_PIN_13
#define R1IN2_GPIO_Port GPIOB
#define lipol_detection_Pin GPIO_PIN_14
#define lipol_detection_GPIO_Port GPIOB
#define lipol_detection_EXTI_IRQn EXTI15_10_IRQn
#define liion_detection_Pin GPIO_PIN_15
#define liion_detection_GPIO_Port GPIOB
#define liion_detection_EXTI_IRQn EXTI15_10_IRQn
#define enc_CH1_Pin GPIO_PIN_8
#define enc_CH1_GPIO_Port GPIOA
#define enc_CH2_Pin GPIO_PIN_9
#define enc_CH2_GPIO_Port GPIOA
#define enc_KEY_Pin GPIO_PIN_10
#define enc_KEY_GPIO_Port GPIOA
#define enc_KEY_EXTI_IRQn EXTI15_10_IRQn
#define PWM_Pin GPIO_PIN_15
#define PWM_GPIO_Port GPIOA
#define bmp2_CS_Pin GPIO_PIN_3
#define bmp2_CS_GPIO_Port GPIOB
#define PWM_discharge_Pin GPIO_PIN_4
#define PWM_discharge_GPIO_Port GPIOB
#define bmp1_CS_Pin GPIO_PIN_5
#define bmp1_CS_GPIO_Port GPIOB
#define PWM_charge_Pin GPIO_PIN_7
#define PWM_charge_GPIO_Port GPIOB
#define bmp3_CS_Pin GPIO_PIN_8
#define bmp3_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
