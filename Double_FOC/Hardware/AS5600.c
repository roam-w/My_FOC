#include "AS5600.h"
//#include "tim.h"
#include "common.h"


/**
  * @brief  初始化AS5600编码器
  * @param  has5600: AS5600结构体指针
  * @param  hi2c: I2C句柄指针
  * @retval 无
  */
void AS5600_Init(AS5600_HandleTypeDef *has5600, I2C_HandleTypeDef *hi2c)
{
    has5600->hi2c = hi2c;
    has5600->address = AS5600_I2C_ADDRESS;
    has5600->raw_angle = 0;
    has5600->angle = 0;
		has5600->last_angle = 0;
		has5600->rotation = 0;
		has5600->total_angle = 0;
}	

/**
  * @brief  读取AS5600原始角度值（0-4095）
  * @param  has5600: AS5600结构体指针
  * @retval 原始角度值（0-4095）
  */
uint16_t AS5600_ReadRawAngle(AS5600_HandleTypeDef *has5600)
{
    uint8_t buffer[2] = {0};
    uint16_t angle = 0;
    
    // 设置要读取的寄存器地址
    uint8_t reg_addr = AS5600_REG_RAW_ANGLE;
    
    // 写入寄存器地址
    HAL_I2C_Master_Transmit(has5600->hi2c, has5600->address << 1, &reg_addr, 1, HAL_MAX_DELAY);
    
    // 读取角度数据（2个字节）
    HAL_I2C_Master_Receive(has5600->hi2c, has5600->address << 1, buffer, 2, HAL_MAX_DELAY);
    
    // 组合角度值：高8位在buffer[0]，低8位在buffer[1]
    // AS5600输出的是12位数据（0-4095）
    angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    angle = angle & 0x0FFF;  // 取低12位
    
    // 更新结构体数据
    has5600->raw_angle = angle;
		
    return angle;
}

/**
  * @brief  读取AS5600处理后的角度值（0-4095）
  * @param  has5600: AS5600结构体指针
  * @retval 角度值（0-4095）
  */
uint16_t AS5600_ReadAngle(AS5600_HandleTypeDef *has5600)
{
    uint8_t buffer[2] = {0};
    uint16_t angle = 0;
    
    // 设置要读取的寄存器地址
    uint8_t reg_addr = AS5600_REG_ANGLE;
    
    // 写入寄存器地址
    HAL_I2C_Master_Transmit(has5600->hi2c, has5600->address << 1, &reg_addr, 1, HAL_MAX_DELAY);
    
    // 读取角度数据（2个字节）
    HAL_I2C_Master_Receive(has5600->hi2c, has5600->address << 1, buffer, 2, HAL_MAX_DELAY);
    
    // 组合角度值
    angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    angle = angle & 0x0FFF;  // 取低12位
    
    // 更新结构体数据
    has5600->angle = angle;
    
    return angle;
}

// 获取角度值（度）
float AS5600_GetDegree(AS5600_HandleTypeDef *has5600){
	uint16_t raw = AS5600_ReadRawAngle(has5600);
	return raw / 4096.0f * 360.0f;
}

// 获取角度值（弧度）
float AS5600_GetRadian(AS5600_HandleTypeDef *has5600){
	uint16_t raw = AS5600_ReadRawAngle(has5600);
	return raw / 2048.0f * 3.1415926f;
}

//获取累计角度(度)
float AS5600_GetTotalAngle(AS5600_HandleTypeDef *has5600){
	uint16_t last_angle = has5600->last_angle;
	uint16_t current_angle = AS5600_ReadRawAngle(has5600);
	
	int16_t diff = (int16_t)current_angle - (int16_t)last_angle;
	
	if (diff > 2048) has5600->rotation--;
	if (diff < -2048) has5600->rotation++;
	
	float total_angle = has5600->rotation * 360.0f + current_angle / 4096.0f * 360.0f;
	
	has5600->last_angle = current_angle;
	has5600->total_angle = total_angle;
	
	return total_angle;
}

// 获取旋转速度
float AS5600_GetVelocity(AS5600_HandleTypeDef *has5600) {
	static uint32_t last_time = 0;
	static float last_angle = 0;
	static bool first_call = true;
	
	// 获取当前定时器计数值（单位：毫秒）
	uint32_t current_time = TIM2->CNT;
	
	// 处理第一次调用
	if (first_call) {
			first_call = false;
			last_angle = AS5600_GetTotalAngle(has5600);
			last_time = current_time;
			return 0;
	}
	
	// 获取当前角度
	float current_angle = AS5600_GetTotalAngle(has5600);
	float angle_diff = current_angle - last_angle;
	
	// 计算时间差（毫秒），处理计数器溢出
	uint32_t time_diff_ms;
	if (current_time >= last_time) {
			time_diff_ms = current_time - last_time;
	} else {
			// 计数器溢出处理
			time_diff_ms = (65535 - last_time) + current_time;
	}
	
	// 转换为秒
	float time_diff = time_diff_ms * 1e-4f;
	
//	if (time_diff < 0.001f) return 0;

	
	// 计算角速度（度/秒）
	float velocity = angle_diff / time_diff;
	
	// 更新状态
	last_angle = current_angle;
	last_time = current_time;
	
	// 转换为RPM（每转360度，每分钟60秒）
	// velocity / 6.0f 相当于 (velocity / 360) * 60 = velocity / 6
	return LowPassFilter(velocity / 6.0f);
//	return angle_diff;
}
