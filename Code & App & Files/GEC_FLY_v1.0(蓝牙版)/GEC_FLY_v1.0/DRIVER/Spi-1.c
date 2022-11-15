#include "spi.h"

void SPI_NRF_Init(void)
{
      SPI_InitTypeDef  SPI_InitStructure;
   	  GPIO_InitTypeDef GPIO_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO  , ENABLE);
		// GPIO_SetBits(GPIOB, GPIO_Pin_12); //NRF_CSÔ¤ÖÃÎª¸   Ô­À´³ÌÐò
		GPIO_SetBits(GPIOA, GPIO_Pin_4); //NRF_CSÔ¤ÖÃÎª¸ß 
		
//	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
//	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
//	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; 
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	  	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
//	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
//	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
//	  	GPIO_Init(GPIOB,&GPIO_InitStructure); 
			
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	  	GPIO_Init(GPIOB,&GPIO_InitStructure);
			
//		  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;		//NRF_CS
//	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
			
		  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;		//NRF_CS
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure); 
			
		
//		  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;		//NRF_CE
//	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
			
			GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;		//NRF_CE
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

        /* SPI1 configuration */
        SPI_Cmd(SPI1, DISABLE);             //±ØÐëÏÈ½ûÄÜ,²ÅÄÜ¸Ä±äMODE
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;

        SPI_Init(SPI1, &SPI_InitStructure);

        /* SPI1 enable */
        SPI_Cmd(SPI1, ENABLE);
}
u8 SPI_RW(u8 dat)
{
        /* Loop while DR register in not emplty */
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

        /* Send byte through the SPI2 peripheral */
        SPI_I2S_SendData(SPI1, dat);

        /* Wait to receive a byte */
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

        /* Return the byte read from the SPI bus */
        return SPI_I2S_ReceiveData(SPI1);  
}
