#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define PI 3.1415926f

// AS5600 I2C地址
#define AS5600_I2C_ADDR         0x36

// AS5600寄存器地址
#define AS5600_REG_RAW_ANGLE    0x0C
#define AS5600_REG_ANGLE        0x0E
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_MAGNITUDE    0x1B
#define AS5600_REG_CONF         0x07
#define AS5600_REG_ZPOS         0x01
#define AS5600_REG_MPOS         0x03
#define AS5600_REG_MANG         0x05

// 角度相关常量
#define AS5600_MAX_RAW_ANGLE    4096.0f  // 12位分辨率
#define AS5600_FULL_CIRCLE      360.0f
#define AS5600_RADIANS_PER_STEP (2.0f * M_PI / AS5600_MAX_RAW_ANGLE)

// 状态标志
#define AS5600_STATUS_MD        0x20    // 磁铁检测
#define AS5600_STATUS_ML        0x10    // 磁铁强度过低
#define AS5600_STATUS_MH        0x08    // 磁铁强度过高

// 错误代码
#define AS5600_OK               0
#define AS5600_ERROR_I2C        1
#define AS5600_ERROR_MAGNET     2
#define AS5600_ERROR_CALIB      3

// 结构体定义
typedef struct {
    I2C_HandleTypeDef *hi2c;            // I2C句柄
    
    // 角度数据
    uint16_t raw_angle;                 // 原始角度值(0-4095)
    uint16_t last_raw_angle;            // 上一次原始角度
    float current_angle;                // 当前角度(0-360度)
    float total_angle;                  // 累计总角度(度)
    float angle_rad;                    // 当前角度(弧度)
    
    // 多圈计数和方向
    int32_t rotation_count;             // 转动圈数
    int8_t direction;                   // 转动方向: 1=顺时针, -1=逆时针, 0=静止
    int32_t cumulative_steps;           // 累计步数
    
    // 校准数据
    uint16_t zero_offset;               // 零位偏移(原始值)
    float zero_angle;                   // 零位角度(度)
    bool is_calibrated;                 // 是否已校准
    uint16_t calibration_samples;       // 校准采样数
    
    // 滤波和性能
    float filtered_angle;               // 滤波后角度
    float filter_alpha;                 // 滤波器系数
    uint32_t update_time;               // 最后更新时间
    uint32_t error_count;               // 错误计数
    
    // 磁铁状态
    bool magnet_detected;               // 磁铁是否检测到
    bool magnet_too_weak;               // 磁铁太弱
    bool magnet_too_strong;             // 磁铁太强
    
} AS5600_HandleTypeDef;

// 函数声明
// 初始化函数
void AS5600_Init(AS5600_HandleTypeDef *has5600, I2C_HandleTypeDef *hi2c);
uint8_t AS5600_Begin(AS5600_HandleTypeDef *has5600);

// 角度读取函数
uint8_t AS5600_Update(AS5600_HandleTypeDef *has5600);
uint16_t AS5600_GetRawAngle(AS5600_HandleTypeDef *has5600);
float AS5600_GetAngle(AS5600_HandleTypeDef *has5600);
float AS5600_GetTotalAngle(AS5600_HandleTypeDef *has5600);
float AS5600_GetAngleRadians(AS5600_HandleTypeDef *has5600);
int32_t AS5600_GetRotationCount(AS5600_HandleTypeDef *has5600);

// 方向检测函数
int8_t AS5600_GetDirection(AS5600_HandleTypeDef *has5600);
int32_t AS5600_GetCumulativeSteps(AS5600_HandleTypeDef *has5600);
float AS5600_GetAngularVelocity(AS5600_HandleTypeDef *has5600);

// 校准函数
uint8_t AS5600_CalibrateZero(AS5600_HandleTypeDef *has5600);
uint8_t AS5600_SetZeroPosition(AS5600_HandleTypeDef *has5600);
uint8_t AS5600_SaveZeroToROM(AS5600_HandleTypeDef *has5600);
uint8_t AS5600_ClearCalibration(AS5600_HandleTypeDef *has5600);

// 状态检测函数
uint8_t AS5600_CheckMagnetStatus(AS5600_HandleTypeDef *has5600);
bool AS5600_IsMagnetDetected(AS5600_HandleTypeDef *has5600);
bool AS5600_IsMagnetStrengthOK(AS5600_HandleTypeDef *has5600);

// 配置函数
void AS5600_SetFilterCoefficient(AS5600_HandleTypeDef *has5600, float alpha);
uint32_t AS5600_GetErrorCount(AS5600_HandleTypeDef *has5600);
void AS5600_ResetTotalAngle(AS5600_HandleTypeDef *has5600);

// 内部函数
uint8_t _AS5600_ReadRegister(AS5600_HandleTypeDef *has5600, uint8_t reg, uint16_t *value);
uint8_t _AS5600_WriteRegister(AS5600_HandleTypeDef *has5600, uint8_t reg, uint16_t value);
int16_t _AS5600_CalculateAngleDiff(AS5600_HandleTypeDef *has5600);

#endif /* __AS5600_H */

