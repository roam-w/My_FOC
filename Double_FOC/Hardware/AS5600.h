#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// AS5600 I2C地址
#define AS5600_I2C_ADDRESS    0x36

// AS5600寄存器地址
#define AS5600_REG_RAW_ANGLE  0x0C
#define AS5600_REG_ANGLE      0x0E

// AS5600结构体
typedef struct {
    I2C_HandleTypeDef *hi2c;      // I2C句柄
    uint8_t address;              // 设备地址
    uint16_t raw_angle;           // 原始角度值
    uint16_t angle;               // 处理后角度值
		uint16_t last_angle;        	// 上次读取角度值	
		int16_t rotation;							// 旋转圈数
} AS5600_HandleTypeDef;

// 函数声明
void AS5600_Init(AS5600_HandleTypeDef *has5600, I2C_HandleTypeDef *hi2c);
uint16_t AS5600_ReadRawAngle(AS5600_HandleTypeDef *has5600);
uint16_t AS5600_ReadAngle(AS5600_HandleTypeDef *has5600);
float AS5600_GetDegree(AS5600_HandleTypeDef *has5600);
float AS5600_GetRadian(AS5600_HandleTypeDef *has5600);
float AS5600_GetTotalAngle(AS5600_HandleTypeDef *has5600);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
