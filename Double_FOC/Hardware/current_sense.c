#include "current_sense.h"
#include "main.h"
#include "MyFOC.h"

static float ADC_ToVoltage(uint16_t adc_value);
static uint16_t Read_ADC_Single(ADC_HandleTypeDef *hadc);

//初始化电流模块
void CurrentSense_Init(CurrentSense_HandleTypeDef *hcs, ADC_HandleTypeDef *hadc, uint32_t channel, float shunt_resistor){
	hcs->hadc = hadc;
	hcs->adc_channel = channel;
	hcs->shunt_resistor = shunt_resistor;
	hcs->gain = 50.0f;
	hcs->offset_voltage = 0.0f;
	hcs->current_scale = 1.0f/(hcs->gain*hcs->shunt_resistor);
}

//获取单次电流测量值
float CurrentSense_GetCurrent(CurrentSense_HandleTypeDef *hcs){
	uint16_t adc_value;
	float adc_voltage, shunt_voltage, current;
	
	// 读取ADC值
	adc_value = Read_ADC_Single(hcs->hadc);
	adc_voltage = ADC_ToVoltage(adc_value);
	shunt_voltage = adc_voltage - hcs->offset_voltage - INA240_VREF;
	current = shunt_voltage * hcs->current_scale;
	
	return current;
}

//获取平均电流值(用于滤波)
float CurrentSense_GetCurrentAverage(CurrentSense_HandleTypeDef *hcs, uint8_t samples){
	float sum = 0.0f;
	uint8_t i;
	
	if (samples == 0) samples = 1;
	
	for(i = 0; i < samples; i++){
		sum += CurrentSense_GetCurrent(hcs);
		HAL_Delay(1);
	}
	
	return sum/samples;
}

// 校准零点偏移(在电流为0时调用)
void CurrentSense_CalibrateOffset(CurrentSense_HandleTypeDef *hcs, uint16_t samples){
	uint32_t sum = 0;
	uint16_t i;
	
	if (samples == 0) samples = 1;
	
	for (i = 0; i < samples; i++){
		sum += Read_ADC_Single(hcs->hadc);
		HAL_Delay(1);
	}
	
	hcs->offset_voltage = ADC_ToVoltage(sum/samples);
}

//读取单次ADC值
static uint16_t Read_ADC_Single(ADC_HandleTypeDef *hadc){
	uint16_t adc_value;
	
	// ADC启动转换
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 10);
	
	adc_value = HAL_ADC_GetValue(hadc);
	
	HAL_ADC_Stop(hadc);
	
	adc_value = LowPassFilter(adc_value);
	return adc_value;
}

// adc转化为电压值
static float ADC_ToVoltage(uint16_t adc_value){
	return (adc_value * 3.3f) / 4095.0f;
}

