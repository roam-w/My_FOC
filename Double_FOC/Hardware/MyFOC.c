#include "MyFOC.h"
#include "main.h"
#include "tim.h"

float voltage_limit = 11;
float voltage_power_supply = 12.6;
float shaft_angle = 0;
float zero_electric_angle = 0, Ualpha = 0, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

int PP = 7, DIR = -1;
float Tp = 0.3;

//角度归一化[0， 2*PI]
float _normalizeAngle(float angle){
	float a = fmod(a, 2*PI);
	return a > 0 ? a : (a + 2*PI);
}

// 电角度求解
float _electricAngle(float shaft_angle){
	return _normalizeAngle((float)(DIR*PP) * shaft_angle);
}

// 设置PWM到控制器输出
void setPWM(float Ua, float Ub, float Uc, float* dc){
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
	dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
	dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
	
	dc[0] = dc_a*255;
	dc[1] = dc_b*255;
	dc[2] = dc_c*255;
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dc_a*255);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dc_b*255);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dc_c*255);
}

// 逆变换输出PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el, float* dc){
	angle_el = _normalizeAngle(angle_el);
	
	//帕克逆变换
	Ualpha = -Uq*sin(angle_el);
	Ubeta = Uq*cos(angle_el);
	
	//克拉克逆变换
	Ua = Ualpha + voltage_power_supply/2;
	Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
	Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
	
	setPWM(Ua, Ub, Uc, dc);
}

// 变换获取电流iq
float Get_iq(float ia, float ib, float angle_el){
	angle_el = _normalizeAngle(angle_el);
	//克拉克变换
	float ialpha = ia;
	float ibeta = (2*ib+ia) / sqrt(3);
	//帕克变换
	float iq = ibeta*cos(angle_el) - ialpha*sin(angle_el);
	
	return iq;
}


// 低通滤波
float LowPassFilter(float x){
	static uint32_t last_time = 0;
	static uint32_t last_y = 0;
	uint32_t current_time = HAL_GetTick();
	float dt = (current_time - last_time) / 1000.0f;
	
	if (dt > 0.3f){
		last_y = x;
		last_time = current_time;
		return x;
	}
	
	float alpha = Tp / (Tp + dt);
	float y = alpha * last_y + (1.0f-alpha) * x;
	last_y = y;
	last_time = current_time;
	return y;
}

//获取当前微秒值
uint32_t micros(void) {
    uint32_t ms;
    uint32_t cnt;
    
    // 禁用中断以防止读取时SysTick更新
    __disable_irq();
    
    ms = HAL_GetTick();           // 获取毫秒数
    cnt = SysTick->VAL;           // 获取当前计数值
    
    // 检查是否在读取过程中发生了SysTick中断
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        ms = HAL_GetTick();       // 重新读取毫秒数
        cnt = SysTick->VAL;       // 重新读取计数值
    }
    
    __enable_irq();
    
    // 计算微秒数
    // SysTick通常配置为1ms重载，频率为 SystemCoreClock / 1000
    // 所以每微秒的计数 = SystemCoreClock / 1000000
    return ms * 1000 + ((SysTick->LOAD - cnt) * 1000 / (SystemCoreClock / 1000));
}
