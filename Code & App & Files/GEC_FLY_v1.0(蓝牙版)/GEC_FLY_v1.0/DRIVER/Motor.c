/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴电机控制初始化协议
**********************************************/

#include "motor.h"

void Motor_SetPwm(int16_t PWM1,int16_t PWM2,int16_t PWM3,int16_t PWM4)
{		
	if(PWM1>MOTOR_MAX)	
		PWM1 = MOTOR_MAX;
	else if(PWM1<0)
		PWM1 = 0;
		
	if(PWM2>MOTOR_MAX)	
		PWM2 = MOTOR_MAX;
	else if(PWM2<0)
		PWM2 = 0;
		
	if(PWM3>MOTOR_MAX)	
		PWM3 = MOTOR_MAX;
	else if(PWM3<0)
		PWM3 = 0;
		
	if(PWM4>MOTOR_MAX)	
		PWM4 = MOTOR_MAX;
	else if(PWM4<0)
		PWM4 = 0;

	TIM4->CCR1 = PWM1;
	TIM4->CCR2 = PWM2;
	TIM4->CCR3 = PWM3;
	TIM4->CCR4 = PWM4;
}

void Motor_Init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//打开TIM4的时钟
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = MOTOR_MAX;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
