/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/*
 *  USER BUTTON - przelacznie ekranow
 *  potencjometr - ustawianie prądu rozładowania (PWM mosfeta)
 *
 *
 * 	PINOUT SPI1
 * 	CLK  	PA5
 * 	MISO	PA6
 * 	MOSI 	PB7
 *
 * 	OLED +3.3V
 *	--CS	PA1
 *	--DC	PA2
 *	--RST	PA3
 *
 *	BME1	+3.3V
 *	--CS	PB5
 *
 *	BME2	+3.3V
 *	--CS	PB3
 *
 *	BME3	+3.3V
 *	--CS	PB8
 *
 *	SD card	+3.3V  == exFAT
 *	--CS	PA4
 *
 *	PINOUT I2C
 *	SCL PB6
 *	SDA PB7
 *	SGP30 0x58
 *	INA219 0x40
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "sensors/printf_to_uart.h"
#include "sensors/st7735.h"
#include "sensors/fonts.h"
#include "sensors/testimg.h"
#include "sensors/BMPXX80.h"
#include "sensors/sensirion_common.h"
#include "sensors/sgp30.h"
#include "sensors/INA219.h"
#include "logic/user_SDcard.h"
#include "logic/user_OLED.h"
#include "logic/user_fnc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct state st = {
	.current_screen_type = MENU_MAIN,
	.menu_current_ptr = MENU_START,
	.sensor_current = SENSOR_FIRST,
	.battery_state = BATTERY_IDLE,
	.auto_mode_current = MANUAL_MODE,

	.screen_menu_ptr = 0,
	.screen_menu_current = 0,
	.battery_ptr = 0,
	.battery_current = 0,
	.status_ptr = 0,
	.status_current = 0,
	.auto_mode_ptr = 0,
	.auto_mode_current = 0,
	.enc_count = 0,
	.prev_enc_count = 0,
	.enc_offset = 0,
	.set_current_prev = 0,

	.is_screen_menu = true,
	.screen_clear = true,
};
struct sensors s = {0};

const char* menu[SCREENS_MENU_NUM] = {
	"Start",
	"Typ baterii",
	"Prad",
	"Status",
	"Tryb auto",
	"Stop"
};

const char* batteries[BATERYS_NUM] = {
	"Li-Pol",
	"Li-On"
};

const char* status[STATUS_NUM] = {
	"Bezczynny",
	"Ladowanie",
	"Rozladowyw."
};

const char* auto_mode[AUTO_MODE_NUM] = {
	"Auto.",
	"Reczny"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// OLED
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);

	// BMP x3
	for (uint8_t index = 0; index < 1 ; ++index) {
		if (!BMP280_Init(&hspi1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE, index)) {
			LOG_DEBUG("BMP280 sensor error\r\n");
		}
	}

	// SGP
	if (sgp_probe() != STATUS_OK) {
		LOG_DEBUG("SGP sensor error\r\n");
	}


	// TIMER
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	LOG_DEBUG("Complete peripheral initialization\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// stele probkowanie
		if (st._interrupt_flag == true && st.is_measurements_started == true) {
			read_sensors_data();

			// charging state
			control_battery_state(&s.voltage);
			handle_battery_state();

			// OLED
			OLED_manage();

			// SD
			SDcardWriteData();

			// Transmit over uart
			LOG_DATA("{%u,%u,%.2f,%.2f,"
					"%.2f,%ld,%.2f,%ld,%.2f,%ld,"
					"%f,%f,%.2f,%i}\r\n",
					s.tvoc_ppb, s.co2_eq_ppm, s.scaled_ethanol_signal/512.0f, s.scaled_h2_signal/512.0f,
					s.BMP280temperature[0], s.BMP280pressure[0], s.BMP280temperature[1], s.BMP280pressure[1], s.BMP280temperature[2], s.BMP280pressure[2],
					s.voltage, s.current, s.set_current, st.battery_state);

			st._interrupt_flag = false;
		}
		// menu poczatkowe
		else if (st.is_measurements_started == false || st.screen_clear == true) {
			OLED_manage();
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// encoder
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
    	st.enc_count = ((__HAL_TIM_GET_COUNTER(htim) / 4) - st.enc_offset);
		st.sensor_current = (enum SensorScreen)(st.enc_count % SENSOR_SCREEN_COUNT);
		st.menu_current_ptr = (enum MenuScreen)(1 + (st.enc_count % (MENU_SCREEN_COUNT - 1)));
		st.battery_ptr = st.enc_count % BATERYS_NUM;
		st.status_ptr = st.enc_count % STATUS_NUM;
		st.auto_mode_ptr = st.enc_count % AUTO_MODE_NUM;

		if (st.prev_enc_count != st.enc_count) {
    		st.screen_clear = true;
    	}
    	st.prev_enc_count = st.enc_count;
    }
}

// main loop sensor data
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim4 && st.is_measurements_started == true){
		if (st._interrupt_flag == true){
			LOG_DEBUG("Flaga _interrupt_flag jest juz 1\r\n");
		}
		else{
			st._interrupt_flag = true;
		}
	}
}

// User Button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_press_time = 0;
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == enc_KEY_Pin) {
        // debounce
        if (now - last_press_time >= 500) {
            st.is_enc_pressed = true;
            st.screen_clear = true;
            last_press_time = now;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
