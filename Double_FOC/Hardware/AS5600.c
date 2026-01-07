#include "AS5600.h"

//初始化AS5600
void AS5600_Init(AS5600_HandleTypeDef *has5600, I2C_HandleTypeDef *hi2c)
{
    has5600->hi2c = hi2c;

    has5600->raw_angle = 0;
    has5600->last_raw_angle = 0;
    has5600->current_angle = 0.0f;
    has5600->total_angle = 0.0f;
    has5600->angle_rad = 0.0f;
    has5600->filtered_angle = 0.0f;

    has5600->rotation_count = 0;
    has5600->direction = 0;
    has5600->cumulative_steps = 0;

    has5600->zero_offset = 0;
    has5600->zero_angle = 0.0f;
    has5600->is_calibrated = false;
    has5600->calibration_samples = 32; 
    
    has5600->filter_alpha = 0.3f; 
    
    has5600->magnet_detected = false;
    has5600->magnet_too_weak = false;
    has5600->magnet_too_strong = false;
    has5600->error_count = 0;
    has5600->update_time = HAL_GetTick();
}

// 开始编码器
uint8_t AS5600_Begin(AS5600_HandleTypeDef *has5600)
{
    uint8_t status = AS5600_CheckMagnetStatus(has5600);
    if(status != AS5600_OK)
        return status;
    
    return AS5600_Update(has5600);
}

//更新AS5600数据
uint8_t AS5600_Update(AS5600_HandleTypeDef *has5600)
{
    uint16_t raw_angle;
    uint8_t result;
    
    result = _AS5600_ReadRegister(has5600, AS5600_REG_RAW_ANGLE, &raw_angle);
    if(result != AS5600_OK)
    {
        has5600->error_count++;
        return result;
    }
    
    has5600->last_raw_angle = has5600->raw_angle;
    has5600->raw_angle = raw_angle & 0x0FFF; 
    
    int16_t angle_diff = _AS5600_CalculateAngleDiff(has5600);
    
    has5600->cumulative_steps += angle_diff;
    
    has5600->rotation_count = has5600->cumulative_steps / 4096;
    
    float raw_angle_deg = ((float)has5600->raw_angle / AS5600_MAX_RAW_ANGLE) * AS5600_FULL_CIRCLE;
    
    if(has5600->is_calibrated)
    {
        has5600->current_angle = raw_angle_deg - has5600->zero_angle;
        if(has5600->current_angle < 0)
            has5600->current_angle += AS5600_FULL_CIRCLE;
        if(has5600->current_angle >= AS5600_FULL_CIRCLE)
            has5600->current_angle -= AS5600_FULL_CIRCLE;
    }
    else
    {
        has5600->current_angle = raw_angle_deg;
    }
    
    has5600->angle_rad = has5600->current_angle * PI / 180.0f;
    
    has5600->filtered_angle = has5600->filter_alpha * has5600->current_angle + 
                             (1.0f - has5600->filter_alpha) * has5600->filtered_angle;
    
    has5600->total_angle = has5600->rotation_count * AS5600_FULL_CIRCLE + has5600->current_angle;
    
    has5600->update_time = HAL_GetTick();
    
    return AS5600_OK;
}

//计算角度差值(处理边界情况)
int16_t _AS5600_CalculateAngleDiff(AS5600_HandleTypeDef *has5600)
{
    int16_t raw_diff = (int16_t)has5600->raw_angle - (int16_t)has5600->last_raw_angle;
    
    if(raw_diff > 2048)
    {
        raw_diff -= 4096;
        has5600->direction = -1;
    }
    else if(raw_diff < -2048)
    {
        raw_diff += 4096;
        has5600->direction = 1;
    }
    else if(raw_diff > 0)
    {
        has5600->direction = 1;
    }
    else if(raw_diff < 0)
    {
        has5600->direction = -1;
    }
    else
    {
        has5600->direction = 0;
    }
    
    return raw_diff;
}

//读取原始角度值
uint16_t AS5600_GetRawAngle(AS5600_HandleTypeDef *has5600)
{
    return has5600->raw_angle;
}

//读取角度值
float AS5600_GetAngle(AS5600_HandleTypeDef *has5600)
{
    return has5600->current_angle;
}

//读取累计角度
float AS5600_GetTotalAngle(AS5600_HandleTypeDef *has5600)
{
    return has5600->total_angle;
}

//读取弧度
float AS5600_GetAngleRadians(AS5600_HandleTypeDef *has5600)
{
    return has5600->angle_rad;
}

//读取圈数
int32_t AS5600_GetRotationCount(AS5600_HandleTypeDef *has5600)
{
    return has5600->rotation_count;
}

//读取方向
int8_t AS5600_GetDirection(AS5600_HandleTypeDef *has5600)
{
    return has5600->direction;
}

//获取累计步数
int32_t AS5600_GetCumulativeSteps(AS5600_HandleTypeDef *has5600)
{
    return has5600->cumulative_steps;
}

//获取角速度(度)
float AS5600_GetAngularVelocity(AS5600_HandleTypeDef *has5600)
{
    static uint32_t last_time = 0;
    static float last_angle = 0;
    
    uint32_t current_time = HAL_GetTick();
    float time_diff = (current_time - last_time) / 1000.0f; 
    
    if(time_diff < 0.001f) return 0;
    
    float angle_diff = has5600->total_angle - last_angle;
    
    if(angle_diff > 180.0f)
        angle_diff -= 360.0f;
    else if(angle_diff < -180.0f)
        angle_diff += 360.0f;
    
    float velocity = angle_diff / time_diff;
    
    last_time = current_time;
    last_angle = has5600->total_angle;
    
    return velocity;
}

//零漂校准
uint8_t AS5600_CalibrateZero(AS5600_HandleTypeDef *has5600)
{
    uint32_t angle_sum = 0;
    uint16_t samples_collected = 0;
    
    for(uint16_t i = 0; i < has5600->calibration_samples; i++)
    {
        uint16_t raw_angle;
        uint8_t result = _AS5600_ReadRegister(has5600, AS5600_REG_RAW_ANGLE, &raw_angle);
        
        if(result == AS5600_OK)
        {
            angle_sum += (raw_angle & 0x0FFF);
            samples_collected++;
        }
        
        HAL_Delay(2); 
    }
    
    if(samples_collected == 0)
        return AS5600_ERROR_CALIB;
    
    has5600->zero_offset = angle_sum / samples_collected;
    has5600->zero_angle = ((float)has5600->zero_offset / AS5600_MAX_RAW_ANGLE) * AS5600_FULL_CIRCLE;
    has5600->is_calibrated = true;
    
    return AS5600_OK;
}

//设置当前位置为零点
uint8_t AS5600_SetZeroPosition(AS5600_HandleTypeDef *has5600)
{
    uint8_t result = AS5600_Update(has5600);
    if(result != AS5600_OK)
        return result;
    
    has5600->zero_offset = has5600->raw_angle;
    has5600->zero_angle = ((float)has5600->zero_offset / AS5600_MAX_RAW_ANGLE) * AS5600_FULL_CIRCLE;
    has5600->is_calibrated = true;
    
    return AS5600_OK;
}

//保存零位到AS5600ROM
uint8_t AS5600_SaveZeroToROM(AS5600_HandleTypeDef *has5600)
{
    uint16_t zpos_value = has5600->zero_offset & 0x07FF;
    uint8_t data[2];
    
    data[0] = (zpos_value >> 2) & 0xFF;  // ZPOS[10:2]
    data[1] = (zpos_value & 0x03) << 6;  // ZPOS[1:0]在bit7,6
    
    return _AS5600_WriteRegister(has5600, AS5600_REG_ZPOS, (data[0] << 8) | data[1]);
}

//清除校准数据
uint8_t AS5600_ClearCalibration(AS5600_HandleTypeDef *has5600)
{
    has5600->zero_offset = 0;
    has5600->zero_angle = 0.0f;
    has5600->is_calibrated = false;
    
    return AS5600_OK;
}

//检查磁铁状态
uint8_t AS5600_CheckMagnetStatus(AS5600_HandleTypeDef *has5600)
{
    uint8_t status_reg;
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(has5600->hi2c, AS5600_I2C_ADDR << 1, 
                                 AS5600_REG_STATUS, I2C_MEMADD_SIZE_8BIT, 
                                 &status_reg, 1, 10);
    
    if(hal_status != HAL_OK)
    {
        has5600->error_count++;
        return AS5600_ERROR_I2C;
    }
    
    has5600->magnet_detected = (status_reg & AS5600_STATUS_MD) != 0;
    has5600->magnet_too_weak = (status_reg & AS5600_STATUS_ML) != 0;
    has5600->magnet_too_strong = (status_reg & AS5600_STATUS_MH) != 0;
    
    if(!has5600->magnet_detected)
        return AS5600_ERROR_MAGNET;
    
    if(has5600->magnet_too_weak || has5600->magnet_too_strong)
        return AS5600_ERROR_MAGNET;
    
    return AS5600_OK;
}

//检查磁铁是否检测
bool AS5600_IsMagnetDetected(AS5600_HandleTypeDef *has5600)
{
    return has5600->magnet_detected;
}

//检查磁铁强度
bool AS5600_IsMagnetStrengthOK(AS5600_HandleTypeDef *has5600)
{
    return !(has5600->magnet_too_weak || has5600->magnet_too_strong);
}

//设置滤波系数
void AS5600_SetFilterCoefficient(AS5600_HandleTypeDef *has5600, float alpha)
{
    if(alpha < 0.0f) alpha = 0.0f;
    if(alpha > 1.0f) alpha = 1.0f;
    has5600->filter_alpha = alpha;
}

//获取错误计数
uint32_t AS5600_GetErrorCount(AS5600_HandleTypeDef *has5600)
{
    return has5600->error_count;
}

//重置累计角度
void AS5600_ResetTotalAngle(AS5600_HandleTypeDef *has5600)
{
    has5600->total_angle = has5600->current_angle;
    has5600->rotation_count = 0;
    has5600->cumulative_steps = 0;
}

//读取AS5600寄存器
uint8_t _AS5600_ReadRegister(AS5600_HandleTypeDef *has5600, uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = {0};
    HAL_StatusTypeDef hal_status;
    
    hal_status = HAL_I2C_Mem_Read(has5600->hi2c, AS5600_I2C_ADDR << 1, 
                                 reg, I2C_MEMADD_SIZE_8BIT, 
                                 data, 2, 10);
    
    if(hal_status != HAL_OK)
    {
        has5600->error_count++;
        return AS5600_ERROR_I2C;
    }
    
    *value = (data[0] << 8) | data[1];
    return AS5600_OK;
}

//写入AS5600寄存器
uint8_t _AS5600_WriteRegister(AS5600_HandleTypeDef *has5600, uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    HAL_StatusTypeDef hal_status;
    
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;
    
    hal_status = HAL_I2C_Mem_Write(has5600->hi2c, AS5600_I2C_ADDR << 1, 
                                  reg, I2C_MEMADD_SIZE_8BIT, 
                                  data, 2, 10);
    
    if(hal_status != HAL_OK)
    {
        has5600->error_count++;
        return AS5600_ERROR_I2C;
    }
    
    return AS5600_OK;
}

