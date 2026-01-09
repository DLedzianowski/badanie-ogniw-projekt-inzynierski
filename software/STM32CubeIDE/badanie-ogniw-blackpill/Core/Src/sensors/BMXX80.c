/*
 * BMXX80.c
 *
 *	The MIT License.
 *	Based on Adafuit libraries.
 *  Created on: 10.08.2018
 *      Author: Mateusz Salamon
 *      www.msalamon.pl
 *
 */
#define CS1_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_5

#define CS2_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_3

#define CS3_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_8

#include "main.h"
#include "stm32f4xx_hal.h"
//#include "gpio.h"
#include "sensors/BMPXX80.h"

#include "math.h"

//
//	 Private variables
//
SPI_HandleTypeDef *spi_h;

typedef struct {
    GPIO_TypeDef *CS_Port;
    uint16_t CS_Pin;
    uint8_t _temperature_res, _pressure_oversampling, _huminidity_oversampling,  _mode, h1, h3;
    int8_t h6;
    int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9, h2, h4, h5;
    uint16_t t1, p1;
    int32_t t_fine;
} BME280_Handle;

BME280_Handle sensors[] = {
    { CS1_GPIO_Port, CS1_Pin },
    { CS2_GPIO_Port, CS2_Pin },
    { CS3_GPIO_Port, CS3_Pin }
};


//
//	Functions
//
uint8_t BME280_Read8(uint8_t addr, uint8_t index)
{
    uint8_t tmp[2] = {0};
	tmp[0] = addr;
	tmp[0] |= (1<<7);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi_h, tmp, tmp, 2, 10);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_SET);
	return tmp[1];
}

uint16_t BME280_Read16(uint8_t addr, uint8_t index)
{
	uint8_t tmp[3];
	tmp[0] = addr;
	tmp[0] |= (1<<7);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi_h, tmp, tmp, 3, 10);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_SET);
	return ((tmp[1] << 8) | tmp[2]);
}

uint16_t BME280_Read16LE(uint8_t addr, uint8_t index)
{
	uint16_t tmp;

	tmp = BME280_Read16(addr, index);
	return (tmp >> 8) | (tmp << 8);
}

void BME280_Write8(uint8_t address, uint8_t data, uint8_t index)
{
	uint8_t tmp[2];
	tmp[0] = address;
	tmp[0] &= ~(1<<7);
	tmp[1] = data;
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi_h, tmp, tmp, 2, 10);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_SET);
}

uint32_t BME280_Read24(uint8_t addr, uint8_t index)
{
	uint8_t tmp[4];
	tmp[0] = addr;
	tmp[0] |= (1<<7);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi_h, tmp, tmp, 3, 10);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_SET);
	return ((tmp[1] << 16) | tmp[2] << 8 | tmp[3]);
}


void BME280_SetConfig(uint8_t standby_time, uint8_t filter, uint8_t index)
{
	BME280_Write8(BME280_CONFIG, (uint8_t)(((standby_time & 0x7) << 5) | ((filter & 0x7) << 2)) & 0xFC, index);
}

uint8_t BME280_IsReadingCalibration(uint8_t index)
{
	uint8_t Status = BME280_Read8(BME280_STATUS, index);

	return ((Status & 1) != 0);
}


uint8_t BME280_Init(SPI_HandleTypeDef *spi_handler, uint8_t temperature_resolution, uint8_t pressure_oversampling, uint8_t huminidity_oversampling, uint8_t mode, uint8_t index) {
	uint8_t HumReg,i;

	spi_h = spi_handler;
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(sensors[index].CS_Port, sensors[index].CS_Pin, GPIO_PIN_SET);

	if (mode > BME280_NORMALMODE)
	    mode = BME280_NORMALMODE;
	sensors[index]._mode = mode;
	if(mode == BME280_FORCEDMODE)
		mode = BME280_SLEEPMODE;

	if (temperature_resolution > BME280_TEMPERATURE_20BIT)
		temperature_resolution = BME280_TEMPERATURE_20BIT;
	sensors[index]._temperature_res = temperature_resolution;
	if (pressure_oversampling > BME280_PRESSURE_ULTRAHIGHRES)
		pressure_oversampling = BME280_PRESSURE_ULTRAHIGHRES;
	sensors[index]._pressure_oversampling = pressure_oversampling;
	if (huminidity_oversampling > BME280_HUMINIDITY_ULTRAHIGH)
		huminidity_oversampling = BME280_HUMINIDITY_ULTRAHIGH;
	sensors[index]._huminidity_oversampling = huminidity_oversampling;

/*
	uint8_t loop_count = 0;
	uint8_t chip_id = 0;
	do {
		chip_id = BME280_Read8(BME280_CHIPID, index);
		HAL_Delay(1);

		loop_count++;
		if (loop_count > 100) {
			return 0;
		}
	} while (chip_id == 0x80);

	if (chip_id != 0x60 && chip_id != 0x58) {
	    return 0;
	}
*/
	while(BME280_Read8(BME280_CHIPID, index) != 0x60 && BME280_Read8(BME280_CHIPID, index) != 0x58);

	BME280_Write8(BME280_SOFTRESET, 0xB6, index);

	for(i = 0; i<30; i++)
		HAL_Delay(10);

	while(BME280_IsReadingCalibration(index))
		for(i = 0; i<10; i++)
			HAL_Delay(1000);



	/* read calibration data */
	sensors[index].t1 = BME280_Read16LE(BME280_DIG_T1, index);
	sensors[index].t2 = BME280_Read16LE(BME280_DIG_T2, index);
	sensors[index].t3 = BME280_Read16LE(BME280_DIG_T3, index);

	sensors[index].p1 = BME280_Read16LE(BME280_DIG_P1, index);
	sensors[index].p2 = BME280_Read16LE(BME280_DIG_P2, index);
	sensors[index].p3 = BME280_Read16LE(BME280_DIG_P3, index);
	sensors[index].p4 = BME280_Read16LE(BME280_DIG_P4, index);
	sensors[index].p5 = BME280_Read16LE(BME280_DIG_P5, index);
	sensors[index].p6 = BME280_Read16LE(BME280_DIG_P6, index);
	sensors[index].p7 = BME280_Read16LE(BME280_DIG_P7, index);
	sensors[index].p8 = BME280_Read16LE(BME280_DIG_P8, index);
	sensors[index].p9 = BME280_Read16LE(BME280_DIG_P9, index);

	sensors[index].h1 = BME280_Read8(BME280_DIG_H1, index);
	sensors[index].h2 = BME280_Read16LE(BME280_DIG_H2, index);
	sensors[index].h3 = BME280_Read8(BME280_DIG_H3, index);
	sensors[index].h4 = ((BME280_Read8(BME280_DIG_H4, index) << 4 ) | (BME280_Read8(BME280_DIG_H4+1, index) & 0xF));
	sensors[index].h5 = ((BME280_Read8(BME280_DIG_H5+1, index) << 4) | (BME280_Read8(BME280_DIG_H5, index) >> 4));
	sensors[index].h6 = (int8_t)BME280_Read8(BME280_DIG_H6, index);

	HumReg = BME280_Read8(BME280_HUM_CONTROL, index);
	HumReg &= 0xF8;
	HumReg |= sensors[index]._huminidity_oversampling;
	BME280_Write8(BME280_HUM_CONTROL, HumReg, index);
	HumReg = BME280_Read8(BME280_HUM_CONTROL, index);
	BME280_Write8(BME280_CONTROL, ((temperature_resolution<<5) | (pressure_oversampling<<2) | mode), index);

	if(mode == BME280_NORMALMODE)
	{
		BME280_SetConfig(BME280_STANDBY_MS_0_5, BME280_FILTER_OFF, index);
	}
	return 1;
}



float BME280_ReadTemperature(uint8_t index)
{
	  int32_t var1, var2;

	  if(sensors[index]._mode == BME280_FORCEDMODE)
	  {
		  uint8_t mode;
		  uint8_t ctrl = BME280_Read8(BME280_CONTROL, index);
		  ctrl &= ~(0x03);
		  ctrl |= BME280_FORCEDMODE;
		  BME280_Write8(BME280_CONTROL, ctrl, index);

		  mode = BME280_Read8(BME280_CONTROL, index); 	// Read written mode
		  mode &= 0x03;							// Do not work without it...

		  if(mode == BME280_FORCEDMODE)
		  {
			  while(1) // Wait for end of conversion
			  {
				  mode = BME280_Read8(BME280_CONTROL, index);
				  mode &= 0x03;
				  if(mode == BME280_SLEEPMODE)
					  break;
			  }
		  }
	  }

	  int32_t adc_T = BME280_Read24(BME280_TEMPDATA, index);
	  if (adc_T == 0x800000)
		  return -99;

	  adc_T >>= 4;

	  var1  = ((((adc_T>>3) - ((int32_t)sensors[index].t1 <<1))) *
			  ((int32_t)sensors[index].t2)) >> 11;

	  var2  = (((((adc_T>>4) - ((int32_t)sensors[index].t1)) *
			  ((adc_T>>4) - ((int32_t)sensors[index].t1))) >> 12) *
			  ((int32_t)sensors[index].t3)) >> 14;

	  sensors[index].t_fine = var1 + var2;

	  float T  = (sensors[index].t_fine * 5 + 128) >> 8;
	  return T/100;

	  return -99;
}


uint8_t BME280_ReadTemperatureAndPressure(float *temperature, int32_t *pressure, uint8_t index)
{
	  int64_t var1, var2, p;

	  // Must be done first to get the t_fine variable set up
	  *temperature = BME280_ReadTemperature(index);

	  if(*temperature == -99)
		  return -1;

	  int32_t adc_P = BME280_Read24(BME280_PRESSUREDATA, index);
	  adc_P >>= 4;

	  var1 = ((int64_t)sensors[index].t_fine) - 128000;
	  var2 = var1 * var1 * (int64_t)sensors[index].p6;
	  var2 = var2 + ((var1*(int64_t)sensors[index].p5)<<17);
	  var2 = var2 + (((int64_t)sensors[index].p4)<<35);
	  var1 = ((var1 * var1 * (int64_t)sensors[index].p3)>>8) +
	    ((var1 * (int64_t)sensors[index].p2)<<12);
	  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)sensors[index].p1)>>33;

	  if (var1 == 0) {
	    return 0;  // avoid exception caused by division by zero
	  }
	  p = 1048576 - adc_P;
	  p = (((p<<31) - var2)*3125) / var1;
	  var1 = (((int64_t)sensors[index].p9) * (p>>13) * (p>>13)) >> 25;
	  var2 = (((int64_t)sensors[index].p8) * p) >> 19;

	  p = ((p + var1 + var2) >> 8) + (((int64_t)sensors[index].p7)<<4);
	  *pressure = (int32_t)p/256;

	  return 0;
}

uint8_t BME280_ReadTemperatureAndPressureAndHuminidity(float *temperature, int32_t *pressure, float *huminidity, uint8_t index)
{
	int64_t var1, var2, p;

	// Must be done first to get the t_fine variable set up
	*temperature = BME280_ReadTemperature(index);

	if(*temperature == -99)
	  return -1;

	int32_t adc_P = BME280_Read24(BME280_PRESSUREDATA, index);
	adc_P >>= 4;

	var1 = ((int64_t)sensors[index].t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)sensors[index].p6;
	var2 = var2 + ((var1*(int64_t)sensors[index].p5)<<17);
	var2 = var2 + (((int64_t)sensors[index].p4)<<35);
	var1 = ((var1 * var1 * (int64_t)sensors[index].p3)>>8) +
			((var1 * (int64_t)sensors[index].p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)sensors[index].p1)>>33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)sensors[index].p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)sensors[index].p8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)sensors[index].p7)<<4);
	*pressure = (int32_t)p/256;

	// Pressure section
	int32_t adc_H = BME280_Read16(BME280_HUMIDDATA, index);
	if (adc_H == 0x8000) // value in case humidity measurement was disabled
		return -1; //error

	int32_t v_x1_u32r;

	v_x1_u32r = (sensors[index].t_fine - ((int32_t)76800)); // 27'365

	v_x1_u32r = (((((adc_H << 14) - (((int32_t)sensors[index].h4) << 20) -
				  (((int32_t)sensors[index].h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
			   (((((((v_x1_u32r * ((int32_t)sensors[index].h6)) >> 10) *
					(((v_x1_u32r * ((int32_t)sensors[index].h3)) >> 11) + ((int32_t)32768))) >> 10) +
				  ((int32_t)2097152)) * ((int32_t)sensors[index].h2) + 8192) >> 14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
							 ((int32_t)sensors[index].h1)) >> 4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	float h = (v_x1_u32r>>12);
	*huminidity = h / 1024.0;
	return 0;
}
