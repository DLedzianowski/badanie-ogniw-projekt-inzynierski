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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Struktura z pomiarami
#define BMP_SENSOR_COUNT 3
struct sensors {
    float BMP280temperature[BMP_SENSOR_COUNT];
    int32_t BMP280pressure[BMP_SENSOR_COUNT];

    uint16_t tvoc_ppb;
    uint16_t co2_eq_ppm;
    uint16_t scaled_ethanol_signal;
    uint16_t scaled_h2_signal;

    uint16_t INA219_Voltage;
    int16_t INA219_Current;
    uint16_t INA219_Power;

    float adc_percentage;
};
extern struct sensors s;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define RETRY_DELAY_MS 2000
#define SCREENS_NUM 2
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn
#define oled_CS_Pin GPIO_PIN_1
#define oled_CS_GPIO_Port GPIOA
#define oled_DC_Pin GPIO_PIN_2
#define oled_DC_GPIO_Port GPIOA
#define oled_RST_Pin GPIO_PIN_3
#define oled_RST_GPIO_Port GPIOA
#define sd_CS_Pin GPIO_PIN_4
#define sd_CS_GPIO_Port GPIOA
#define bmp2_CS_Pin GPIO_PIN_3
#define bmp2_CS_GPIO_Port GPIOB
#define bmp1_CS_Pin GPIO_PIN_5
#define bmp1_CS_GPIO_Port GPIOB
#define bmp3_CS_Pin GPIO_PIN_8
#define bmp3_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
