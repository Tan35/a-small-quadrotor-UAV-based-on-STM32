/**********************************************
*�� �� �ţ�         v1.0
*�� �� �ߣ�         ��Ƕ�ɷ�
*����������         ��������
**********************************************/

#include "configer.h"
#include "includes.h"

void GPIO_Configuration(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;		 //���
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14;		                           //LED1234��LED_START, LEDS
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

}


//ϵͳ�жϹ���
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	  /* Enable the USART1 Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

}

//����ϵͳʱ��,ʹ�ܸ�����ʱ��
void RCC_Configuration(void)
{ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
													 |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1, ENABLE );
   	
}

//LED����˸����
void LED_Controls(void)
{
	u8 i;
	for(i=0;i<2;i++)
	{
		OP_LED3;OP_LED4;
		delay_ms(1000);
		OP_LED3;OP_LED4;
		delay_ms(1000);
	}
}











