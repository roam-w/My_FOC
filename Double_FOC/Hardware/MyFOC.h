#ifndef __MYFOC_H
#define __MYFOC_H

#include "stm32f1xx_hal.h"
#include <math.h>

#define PI 3.1415926f

float _normalizeAngle(float angle);
float _electricAngle(float angle);
void setPWM(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float Get_iq(float ia, float ib, float angle_el);
float LowPassFilter(float x);
uint32_t micros(void);

#endif /* __MYFOC_H */

