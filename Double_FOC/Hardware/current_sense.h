#ifndef __CURRENT_SENSE
#define __CURRENT_SENSE

#include "stm32f1xx_hal.h"

#define CURRENT_ADC ADC1
#define CURRENT_ADC_CLK_ENABLE __HAL_RCC_ADC1_CLK_ENABLE()
#define INA240_VREF 1.65f

typedef struct{
	ADC_HandleTypeDef *hadc;
	uint32_t adc_channel;
	float shunt_resistor;
	float gain;
	float offset_voltage;
	float current_scale;
}CurrentSense_HandleTypeDef;

void CurrentSense_Init(CurrentSense_HandleTypeDef *hcs, ADC_HandleTypeDef *hadc, uint32_t channel, float shunt_resistor);
float CurrentSense_GetCurrent(CurrentSense_HandleTypeDef *hcs);
float CurrentSense_GetCurrentAverage(CurrentSense_HandleTypeDef *hcs, uint8_t samples);
void CurrentSense_CalibrateOffset(CurrentSense_HandleTypeDef *hcs, uint16_t samples);
static uint16_t Read_ADC_Single(ADC_HandleTypeDef *hadc);
static float ADC_ToVoltage(uint16_t adc_value);

#endif

