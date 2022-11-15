#ifndef _SPI_H_
#define _SPI_H_
#include "stm32f10x.h"

#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_5) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define NRF24L01_IRQ  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)  

void SPI1_Init(void);
u8 SPI_RW(u8 dat);

#endif
