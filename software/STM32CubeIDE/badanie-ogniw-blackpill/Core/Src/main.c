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
 * 	PINOUT SPI1
 * 	CLK  	PA5
 * 	MISO	PA6
 * 	MOSI 	PB7
 *
 * 	OLED +3.3V
 *	--CS	PA1
 *	--RST	PA2
 *	--DC	PA3
 *
 *	BME1	+3.3V
 *	--CS	PB5
 *
 *	BMP2	+3.3V
 *	--CS	PB3
 *
 *	BMP3	+3.3V
 *	--CS	PB8
 *
 *	SD card	+3.3V  == exFAT
 *	--CS	PA4
 *
 *	PINOUT I2C
 *	SGP30 0x58
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
#include "sensors/ILI9341_STM32_Driver.h"
#include "sensors/ILI9341_GFX.h"
#include "sensors/fonts.h"
#include "sensors/BMPXX80.h"
#include "sensors/sensirion_common.h"
#include "sensors/sgp30.h"
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
    .set_current_discharge = 0,
    .set_current_discharge_prev = 0,
    .set_current_charge = 0,
    .set_current_charge_prev = 0,
    .get_current_charge = 0,
    .charge_end_current = 0.05,
    .charge_end_current_prev = 0.05,
    .discharge_cutoff_voltage = 3.0,
    .discharge_cutoff_voltage_prev = 3.0,

	.is_screen_menu = true,
	.screen_clear = true,
};
struct sensors s = {0};

const char* menu[SCREENS_MENU_NUM] = {
	"Start",
	//"Typ baterii",
	//"Prad adowania",
	"Min. prad adowania",
	"Min. napiecie roz.",
	"Prad rozadowania",
	"Status",
	"Tryb auto",
	"Stop"
};

const char* batteries[BATERYS_NUM] = {
	"Brak ogniwa",
	"Li-Pol",
	"Li-Ion"
};

const char* status[STATUS_NUM] = {
	"Bezczynny",
	"Ladowanie",
	"Rozladowywanie"
};
const char* auto_mode[AUTO_MODE_NUM] = {
	"Automatyczny",
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
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	// OLED
	ILI9341_Init();
	ILI9341_FillScreen(WHITE);

	// BMP x3
	for (uint8_t index = 0; index < BME_SENSOR_COUNT ; ++index) {
		if (!BME280_Init(&hspi1, BME280_TEMPERATURE_16BIT, BME280_PRESSURE_ULTRALOWPOWER, BME280_HUMINIDITY_ULTRAHIGH, BME280_FORCEDMODE, index)) {
			LOG_DEBUG("BMP280 sensor error\r\n");
		}
	}

	// SGP
	if (sgp_probe() != STATUS_OK) {
		LOG_DEBUG("SGP sensor error\r\n");
	}


	// TIMER
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	OLED_options_init();

	LOG_DEBUG("Complete peripheral initialization\r\n");
	HAL_Delay(1000);
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
			control_battery_state(&s.voltage, &s.current,&s.BME280temperature[0]); // todo
			handle_battery_state();

			// OLED
			OLED_manage();

			// SD
			SDcardWriteData();

			// Transmit over uart
			LOG_DATA("%u,%u,%.2f,%.2f,"
		             "%.2f,%ld,%.2f,%ld,%.2f,%ld,%.2f,"
		             "%.2f,%.2f,%.2f,%.2f,"
		             "%i,%i,%i,%i,%i\r\n",
					s.tvoc_ppb, s.co2_eq_ppm, s.scaled_ethanol_signal/512.0f, s.scaled_h2_signal/512.0f,
					s.BME280temperature[0], s.BME280pressure[0], s.BME280temperature[1], s.BME280pressure[1], s.BME280temperature[2], s.BME280pressure[2], s.BME280humidity,
					s.voltage, s.current, st.set_current_charge, st.set_current_discharge,
					st.battery_state, st.auto_mode_current, st.is_measurements_started, st.discharge_relay, st.charge_relay);

			st._interrupt_flag = false;
		}
		// menu poczatkowe
		else if (st.is_measurements_started == false || st.screen_clear == true) {
			OLED_manage();
		}
		if (st._interrupt_flag == true && st.current_screen_type == SCREEN_MENU && st.menu_current == MENU_MAIN) {
			st._interrupt_flag = false;
			display_bottom_bar();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// encoder
	if (htim->Instance == TIM11) {
	    int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
	    int16_t  pos = raw >> 2;   // /4

	    if (pos != st.enc_count)
	    {
	        st.enc_count = pos;
	        st.screen_clear = true;
	    }
	}
	// main loop sensor data
	if (htim == &htim4 /*&& st.is_measurements_started == true*/){
		if (st._interrupt_flag == true){
			LOG_DEBUG("Flaga _interrupt_flag jest juz 1\r\n");
		}
		else{
			st._interrupt_flag = true;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Encoder Button
    if (GPIO_Pin == enc_KEY_Pin) {
        static uint32_t last_press_time = 0;
        uint32_t now = HAL_GetTick();

        // debounce
        if (now - last_press_time >= 500) {
            st.is_enc_pressed = true;
            st.screen_clear = true;
            last_press_time = now;
        }
    }
    // Set battery type based on the state of PB14 and PB15 pins
    else if (GPIO_Pin == GPIO_PIN_14) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
            st.battery_current = 1;
        else
            st.battery_current = 0;
    }
    else if (GPIO_Pin == GPIO_PIN_15) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET)
            st.battery_current = 2;
        else
            st.battery_current = 0;
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
