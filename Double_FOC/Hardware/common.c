#include "common.h"

//获取当前微秒值
uint32_t micros(void) {
    uint32_t ms, cnt;
    uint32_t pend, reload;
    
    __disable_irq();
    
    ms = HAL_GetTick();
    cnt = SysTick->VAL;
    pend = SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ? 1 : 0;
    reload = SysTick->LOAD;
    
    if (pend) {
        ms = HAL_GetTick();
        cnt = SysTick->VAL;
    }
    
    __enable_irq();
    
    // 更精确的计算方式
    // 注意：SysTick->LOAD存储的是重载值-1
//    uint32_t ticks_per_ms = reload + 1;
    uint32_t us_per_tick = 1000000 / SystemCoreClock;
    
    // 计算从上次中断到现在的微秒数
    uint32_t micros_from_tick = (reload - cnt) * us_per_tick;
    
    return ms * 1000 + micros_from_tick;
}

// 低通滤波
float LowPassFilter(float x){
	float Tp = 0.3;		//滤波系数
	
	static uint32_t last_time = 0;
	static float last_y = 0;
	uint32_t current_time = micros();
	float dt = (current_time - last_time) * 1e-6f;
	
	if (dt > 0.3f || dt <= 0.0f){
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


