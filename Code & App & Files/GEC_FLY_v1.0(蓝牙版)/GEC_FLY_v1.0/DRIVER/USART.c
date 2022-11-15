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
void prints(char* data)						          //打印字符串
{
	u16 i;
//	DIR485_H ;
	for (i=0; i<strlen(data); i++){
		USART1_SendByte(data[i]);
	}
}
void printRX(char* data,u16 nub)			     // 打印定长字符串
{
	u16 i;
//	DIR485_H ;
	for (i=0; i<nub; i++){
		USART1_SendByte(data[i]);
	}
}	
void printv(int v)								         //	 打印数字
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
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2，GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		 //USART2_TX   PA.2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//USART2_RX	  PA.3
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);  

		//Usart1 NVIC 配置

		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

		//USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART2, &USART_InitStructure); //初始化串口
		USART_ClearFlag(USART2,USART_FLAG_TC);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
		USART_Cmd(USART2, ENABLE);                    //使能串口 
}





void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断
	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级 
#endif
}










/******************************************************
	整形数据转字符串函数
	char *itoa(int value, char *string, int radix)
	radix=10 标示是10进制	非十进制，转换结果为0;  

	例：d=-379;
	执行	itoa(d, buf, 10); 后

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
	格式化串口输出函数
	"\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
	"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
	"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
	"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, char *Data,...)
{ 
	while(*Data!=0){				                          //判断是否到达字符串结束符

			USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}


void USART_OUT1(USART_TypeDef* USARTx, uint8_t *Data,uint8_t Len,...)
{ 
	while(Len--){				                          //判断是否到达字符串结束符
	    USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); //等待发送完
	}
}


