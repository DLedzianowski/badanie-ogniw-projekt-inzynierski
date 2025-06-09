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
 * 	MISO	PA6
 * 	MOSI 	PB5
 * 	CLK  	PA5
 *
 * 	OLED +3.3V
 *	--CS	PF13
 *	--RST	PF14
 *	--DC	PF15
 *	BME	+3.3V
 *	--CS	PE9
 *	SD card	+3.3V
 *	--CS	PE11
 *
 *	PINOUT I2C
 *	SCL PB8
 *	SDA PB9
 *	SGP30 0x58
 *	INA219 0x40
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include "BMPXX80.h"
#include "fatfs_sd.h"
#include "sensirion_common.h"
#include "sgp30.h"
#include "INA219.h"

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
uint8_t isProgramStarted = 0;
uint8_t _interruptFlag = 0;

uint16_t adcPosition = 0;

struct sensors s = {0};
INA219_t myina219;

FATFS fs;
FIL fil;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SDcardInit(char* folder_name);
void SDcardWriteData(struct sensors *s);
void SDcardClose(void);
void OLEDdisplay(struct sensors *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SDcardInit(char* folder_name) {
    FRESULT res;
    uint8_t retry_count = 5;

    while (retry_count--) {
        res = f_mount(&fs, "", 1);
        if (res == FR_OK) {
            break;
        }
        printf("Error mounting filesystem! (%d). Retrying...\r\n", res);
     	  ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
        HAL_Delay(RETRY_DELAY_MS);
    }

    retry_count = 5;
    while (retry_count--) {
        res = f_open(&fil, "test.txt", FA_OPEN_ALWAYS | FA_WRITE);
        if (res == FR_OK) {
            break;
        }
        printf("Error opening SDcard file! (%d). Retrying...\r\n", res);
     	  ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
        HAL_Delay(RETRY_DELAY_MS);
    }

    res = f_lseek(&fil, f_size(&fil));
    if (res != FR_OK) {
        printf("Error seeking to end of file! (%d)\r\n", res);
        f_close(&fil);
        return;
    }

    f_puts("\n--- Nowy pomiar ---\n", &fil);
    f_puts("TVOC_ppb,CO2_eq_ppm,Ethanol_signal,H2_signal,Temperatura,Cisnienie,Napiecie_mV,Prad_mA,Moc_mW\n", &fil);

    f_sync(&fil);

}

void SDcardWriteData(struct sensors *s) {
	// ERROR SDcard -> OLED
	if (f_lseek(&fil, f_size(&fil)) != FR_OK) {
  	 printf("Error seeking in file!\r\n");
  	 ST7735_WriteString(10, 140, "Error in file!", Font_7x10, ST7735_RED, ST7735_BLACK);
  	 return;
	}

	char buffer[200];
	snprintf(buffer, sizeof(buffer), "%u,%u,%.2f,%.2f,%.2f,%ld,%u,%d,%u\n",
			s->tvoc_ppb, s->co2_eq_ppm, s->scaled_ethanol_signal/512.0f, s->scaled_h2_signal/512.0f, s->BMP280temperature, s->BMP280pressure,s->INA219_Voltage, s->INA219_Current, s->INA219_Power);

	if (f_puts(buffer, &fil) < 0) {
  	 printf("Error writing to file!\r\n");
	}

	if (f_sync(&fil) != FR_OK) {
		printf("Error syncing file!\r\n");
	}
	f_sync(&fil);
}

void SDcardClose(void) {
    if (f_close(&fil) != FR_OK) {
        printf("Error closing file!\r\n");
    }
}

void OLEDdisplay(struct sensors *s) {
    char buffer[100];
    int tempInt = (int)(s->BMP280temperature * 100);
    int tempFrac = tempInt % 100;

    // Temperatura
    snprintf(buffer, sizeof(buffer), "Temp: %d.%02d C", tempInt / 100, tempFrac);
    ST7735_WriteString(5,  5, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // Ciśnienie
    snprintf(buffer, sizeof(buffer), "Prs:  %ld Pa", s->BMP280pressure);
    ST7735_WriteString(5,  20, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // TVOC
    snprintf(buffer, sizeof(buffer), "TVOC: %4u ppb", s->tvoc_ppb);
    ST7735_WriteString(5,  35, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // CO2eq
    snprintf(buffer, sizeof(buffer), "CO2:  %4u ppm", s->co2_eq_ppm);
    ST7735_WriteString(5,  50, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // Etanol/512.0
    float ethanol = s->scaled_ethanol_signal / 512.0f;
    snprintf(buffer, sizeof(buffer), "EtOH: %.2f", ethanol);
    ST7735_WriteString(5,  65, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // H2/512.0
    float h2 = s->scaled_h2_signal / 512.0f;
    snprintf(buffer, sizeof(buffer), "H2:   %.2f", h2);
    ST7735_WriteString(5,  80, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // INA219_Current
    snprintf(buffer, sizeof(buffer), "Current:  %4d mA", s->INA219_Current);
    ST7735_WriteString(5,  95, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // INA219_Voltage
    snprintf(buffer, sizeof(buffer), "Voltage:  %4u mV", s->INA219_Voltage);
    ST7735_WriteString(5,  110, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    // INA219_Power
    snprintf(buffer, sizeof(buffer), "Power:  %4u mW", s->INA219_Power);
    ST7735_WriteString(5,  125, buffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);

}

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // OLED
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);

  // BMP
  BMP280_Init(&hspi1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
  // SGP
	if (sgp_probe() != STATUS_OK) {
		printf("SGP sensor error\r\n");
	}
	// INA
	INA219_Init(&myina219, &hi2c1, INA219_ADDRESS);
	//INA219_setCalibration_32V_2A(&myina219);

	// SD
	SDcardInit("test.txt");
	isProgramStarted = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	// stele probkowanie
  	if (_interruptFlag == 1){

    	// ADC
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 1);
    	adcPosition = HAL_ADC_GetValue(&hadc1) ;
    	printf("ADC: %.2f%%\r\n", (adcPosition / 4095.0f)*100);

    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (adcPosition / 4095.0f)*1000);


  		// BMP
			BMP280_ReadTemperatureAndPressure(&s.BMP280temperature, &s.BMP280pressure);

			// SGP
			sgp_measure_iaq_blocking_read(&s.tvoc_ppb, &s.co2_eq_ppm);
			sgp_measure_signals_blocking_read(&s.scaled_ethanol_signal, &s.scaled_h2_signal);
			//sgp_set_absolute_humidity()

			// INA219
			s.INA219_Current = INA219_ReadCurrent_raw(&myina219);
			s.INA219_Voltage = INA219_ReadBusVoltage(&myina219);
			s.INA219_Power = INA219_ReadPower(&myina219);


			// SD
			SDcardWriteData(&s);

			// OLED
			OLEDdisplay(&s);

			//printf("{%u,%u,%.2f,%.2f,%.2f,%ld,%u,%d,%u}\r\n",
			//		s.tvoc_ppb, s.co2_eq_ppm, s.scaled_ethanol_signal/512.0f, s.scaled_h2_signal/512.0f, s.BMP280temperature, s.BMP280pressure, s.INA219_Voltage, s.INA219_Current, s.INA219_Power);

			_interruptFlag = 0;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim == &htim7 && isProgramStarted == 1){
  	if (_interruptFlag == 1){
  		printf("Flaga _interruptFlag jest juz 1");
  	}
  	_interruptFlag = 1;
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
