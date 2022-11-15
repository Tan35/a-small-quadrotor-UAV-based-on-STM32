#ifndef _BSP_MOTO_H_
#define _BSP_MOTO_H_
#include "stm32f10x.h"

#define MOTOR_MAX 999

void Motor_SetPwm(int16_t PWM1,int16_t PWM2,int16_t PWM3,int16_t PWM4);
void Motor_Init(void);

#endif
