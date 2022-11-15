#include "stm32f10x.h"
#include "USART.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

char RxBuffer[60];
u16 RxCounter = 0;
u8 Rx_Flag = 0;

void USART1_SendByte(u8 Data)
{ 
   while (!(USART1->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
   USART_SendData(USART1, Data);	 
   
}
void prints(char* data)						          //��ӡ�ַ���
{
	u16 i;
//	DIR485_H ;
	for (i=0; i<strlen(data); i++){
		USART1_SendByte(data[i]);
	}
}
void printRX(char* data,u16 nub)			     // ��ӡ�����ַ���
{
	u16 i;
//	DIR485_H ;
	for (i=0; i<nub; i++){
		USART1_SendByte(data[i]);
	}
}	
void printv(int v)								         //	 ��ӡ����
{
	char num[6]= " ";
	sprintf(num,"%5d",v);
	prints(num);
	}

void USART1_Configuration(void)
{
		USART_InitTypeDef USART_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		USART_InitStructure.USART_BaudRate            = 115200;				 																											  //162.76			   a2c
		USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits            = USART_StopBits_1;
		USART_InitStructure.USART_Parity              = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART1, ENABLE);     
		/*	DIR485_L;  */
}


void uart2_init(u32 bound)
{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART2��GPIOAʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		 //USART2_TX   PA.2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//USART2_RX	  PA.3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);  

		//Usart1 NVIC ����

		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

		//USART ��ʼ������

		USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART2, &USART_InitStructure); //��ʼ������
		USART_ClearFlag(USART2,USART_FLAG_TC);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
		USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
}





void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬����
		  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
#if EN_USART1_RX		  //���ʹ���˽���
	//ʹ�ܽ����ж�
	USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
#endif
}










/******************************************************
	��������ת�ַ�������
	char *itoa(int value, char *string, int radix)
	radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

	����d=-379;
	ִ��	itoa(d, buf, 10); ��

	buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

/******************************************************
	��ʽ�������������
	"\r"	�س���	   USART_OUT(USART1, "abcdefg\r")   
	"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
	"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
	"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, char *Data,...)
{ 
	while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������

			USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}


void USART_OUT1(USART_TypeDef* USARTx, uint8_t *Data,uint8_t Len,...)
{ 
	while(Len--){				                          //�ж��Ƿ񵽴��ַ���������
	    USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); //�ȴ�������
	}
}


