#ifndef _SPI_H_
#define _SPI_H_
#include "stm32f10x.h"

#define NRF_CS(x)   x? GPIO_SetBits(GPIOA,GPIO_Pin_3):GPIO_ResetBits(GPIOA,GPIO_Pin_3)
#define NRF_CE(x)   x? GPIO_SetBits(GPIOB,GPIO_Pin_5):GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define NRF24L01_IRQ  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)  //IRQ 主机数据输入 

void SPI_NRF_Init(void);
u8 SPI_RW(u8 dat);

#endif
