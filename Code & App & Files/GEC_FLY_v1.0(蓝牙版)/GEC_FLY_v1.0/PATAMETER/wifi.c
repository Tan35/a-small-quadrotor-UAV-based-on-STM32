//使用串口2 :定时器2， P10 ->RXD2, P11 -> TXD2


#include "includes.h"
#include "wifi.h"
#include "string.h"

#define u8 unsigned char
#define RS232_QUEUE_CNT 12


#define WIFIControl  FALSE
#define BTControl    TRUE


static volatile unsigned char  uart1buf[128];
char USART1_RX_BUF[64];
char USART2_RX_BUF[64];
unsigned short int  USART1_RX_STA;
unsigned short int  USART2_RX_STA;
char GEC_FLY[18];
static volatile unsigned char  uart1Index = 0 , level1 = 0, level2 = 0, level3 = 0, level4 = 0 ;
static volatile unsigned char  busy = 0;

#if 1
//用循环队列保存数据
typedef struct rs232_queue {
  unsigned char Tail;                    //尾
  unsigned char Head;										 //头
  char nData[RS232_QUEUE_CNT][128];			 //二维数据数组
  unsigned char len[RS232_QUEUE_CNT];		 //长度数组
}RS232_QUEUE;

RS232_QUEUE g_queue;                     

#if 0
static unsigned char IsEmpty()           //空链表
{
  if (g_queue.Tail == g_queue.Head)
    return 1;
  else
    return 0;
}


static void SetEmpty()                 //清空链表
{  
  g_queue.Tail = 0;
  g_queue.Head = 0; 
}

#endif

#if 0
static unsigned char IsFull()          //链表是否满
{
  if (((g_queue.Tail + 1) % RS232_QUEUE_CNT) == g_queue.Head)
    return 1;
  else
    return 0;
}
#endif


#endif


#if 0
char debug_msg[128];                  //调试信息
static unsigned char EnQueue(volatile char *value,unsigned char len) //加入队列尾部
{ 
  
  memcpy((char*)&g_queue.nData[g_queue.Tail][0],(char*)value,len);
  g_queue.nData[g_queue.Tail][len] = '\0';
  g_queue.len[g_queue.Tail] = len;

  sprintf(debug_msg,"   EnQueue:%d,%s\r\n\0",g_queue.Tail,&g_queue.nData[g_queue.Tail][0]);
  debug(debug_msg,strlen(debug_msg));
  sprintf(debug_msg,"   EnQueue: len=%d\r\n\0",(int)len);
  uart1_send(debug_msg,strlen(debug_msg));

  
  sprintf(debug_msg,"%x %x %x %x %x %x %x %x %x\r\n\0",
	    g_queue.nData[g_queue.Tail][0],g_queue.nData[g_queue.Tail][1],g_queue.nData[g_queue.Tail][2],
		g_queue.nData[g_queue.Tail][3],g_queue.nData[g_queue.Tail][4],
		g_queue.nData[g_queue.Tail][5],g_queue.nData[g_queue.Tail][6],g_queue.nData[g_queue.Tail][7],
		g_queue.nData[g_queue.Tail][8]);
  uart1_send(debug_msg,strlen(debug_msg));
  

  g_queue.Tail = (g_queue.Tail + 1) % RS232_QUEUE_CNT; 
  return (g_queue.Tail - g_queue.Head) % RS232_QUEUE_CNT;
}
#endif


#if 0
static unsigned char DeQueue(char *buf,unsigned char *len)               //头插法
{
  unsigned char value = 1;  
  memcpy(buf,(char*)&g_queue.nData[g_queue.Head][0],strlen((char*)&g_queue.nData[g_queue.Head])); 
  memcpy(buf,(char*)&g_queue.nData[g_queue.Head][0],g_queue.len[g_queue.Head]); 
  *len = g_queue.len[g_queue.Head];
  
  sprintf(debug_msg,"   DeQueue:%d,%s\r\n\0",g_queue.Head,&g_queue.nData[g_queue.Head][0]);
  debug(debug_msg,strlen(debug_msg));
  
  sprintf(debug_msg,"DeQueue: len=%d,%x %x %x %x %x %x %x %x %x\r\n\0",strlen((char*)&g_queue.nData[g_queue.Head][0]),
	    g_queue.nData[g_queue.Head][0],g_queue.nData[g_queue.Head][1],g_queue.nData[g_queue.Head][2],
		g_queue.nData[g_queue.Head][3],g_queue.nData[g_queue.Head][4],
		g_queue.nData[g_queue.Head][5],g_queue.nData[g_queue.Head][6],g_queue.nData[g_queue.Head][7],
		g_queue.nData[g_queue.Head][8]);
  uart1_send(debug_msg,strlen(debug_msg));
  	  
   
  g_queue.Head = (g_queue.Head + 1) % RS232_QUEUE_CNT;  
  return value;
}
#endif

#if 0
unsigned char Serial_available() //串口获取
{
  if (IsEmpty())
    return 0;
  else 
    return 1;
}

#endif

#if 0
unsigned char Serial_read(char *msg,unsigned char *len)
{
  return DeQueue(msg,len);  
}

#endif

////////////////////////////////////////////////////////////

//串口2 函数
#if 1

#if 0
void Uart2Init(void)		//9600bps@11.0592MHz
{
  SetEmpty();
  uart1Index = 0;
  uart1_init(115200);    
}
#endif


#if 0
void USART1_IRQHandler(void)                	//串口3中断服务程序
{
  unsigned char Res;
  char msg[128];
  //溢出-如果发生溢出需要先读SR,再读DR寄存器则可清除不断入中断的问题[牛人说要这样]
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET) {
     USART_ClearFlag(USART1,USART_FLAG_ORE);
     USART_ReceiveData(USART1);     
	 //sprintf(debug_msg," UART2 IRQ ORE\r\n\0");
	 //debug(debug_msg,strlen(debug_msg));
  }

  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) { //接收中断(接收到的数据必须是0x0d 0x0a结尾)	 
    //USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    Res = USART_ReceiveData(USART1);		//(USART1->DR);	//读取接收到的数据
	USART_ClearFlag(USART1,USART_FLAG_RXNE);		


    //sprintf(msg,"\r\nuart2 recv:%d\r\n\0",Res);
	//uart1_send("\r\nuart2 recv\r\n\0",15);
	//uart1_send(msg,strlen(msg));
	uart1buf[uart1Index] = Res;
	if (uart1Index == 0 && uart1buf[uart1Index] != 0x81)
	  return;
	uart1Index++;
	if (uart1Index == 8) {
	  sprintf(msg,"\r\n%x %x %x %x %x %x %x %x\r\n\0",
	    uart1buf[0],uart1buf[1],uart1buf[2],uart1buf[3],uart1buf[4],
		uart1buf[5],uart1buf[6],uart1buf[7]);
	  uart1_send(msg,strlen(msg));
	  if (!IsFull()) 
	    EnQueue((char*)uart1buf,8);	 		
	  uart1Index = 0;
	}
  } 
  
} 

#endif


#if(WIFIControl)

 void USART1_IRQHandler(void)
{
	u8 res;	    
	if(USART1->SR & (1<<5))//接收到数据
	{	 
		USART_ClearFlag(USART1,USART_FLAG_ORE);
		res = USART1->DR; 
		//dbgPrintf2(" USART2_IRQHANDLER !\r\n");
		if((USART1_RX_STA & 0x80)==0)//接收未完成
		{
			
			if(USART1_RX_STA & 0x40)//接收到了0x0d
			{
				if(res!=0x0a)
					USART1_RX_STA = 0;//接收错误,重新开始
				else 
				{
					USART1_RX_STA |= 0x80;
				  USART1_RX_BUF[35]=0;
				}//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(res==0x0d)
					USART1_RX_STA |= 0x40; //标志位
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3F]=res;
					USART1_RX_STA++;
					
					if(USART1_RX_STA>63)
							USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}  											 
}

 void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART2->SR & (1<<5))//接收到数据
	{	 
		USART_ClearFlag(USART2,USART_FLAG_ORE);
		res = USART2->DR; 
		//dbgPrintf2(" USART2_IRQHANDLER !\r\n");
		if((USART2_RX_STA & 0x80)==0)//接收未完成
		{
			if(USART2_RX_STA & 0x40)//接收到了0x0d
			{
				if(res!=0x0a)
					USART2_RX_STA = 0;//接收错误,重新开始
				else 
				{
					USART2_RX_STA |= 0x80;
				    USART2_RX_BUF[63]=0;
				}//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(res==0x0d)
					USART2_RX_STA |= 0x40; //标志位
				else
				{
					if(res == '+')
					{
						USART2_RX_STA&=~0x7f;
					}
					USART2_RX_BUF[USART2_RX_STA&0X3F]=res;
					
					USART2_RX_STA++;
					
					if(USART2_RX_STA>63)
							USART2_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}  											 
}

#elif (BTControl)
 void USART1_IRQHandler(void)
{
	u8 res;	    
	if(USART1->SR & (1<<5))//接收到数据
	{	 
		USART_ClearFlag(USART1,USART_FLAG_ORE);
		res = USART1->DR; 
		//dbgPrintf2(" USART2_IRQHANDLER !\r\n");
		if((USART1_RX_STA & 0x80)==0)//接收未完成
		{
			
			if(USART1_RX_STA & 0x40)//接收到了0x0d
			{
				if(res!=0x0a)
					USART1_RX_STA = 0;//接收错误,重新开始
				else 
				{
					USART1_RX_STA |= 0x80;
				  USART1_RX_BUF[35]=0;
				}//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(res==0x0d)
					USART1_RX_STA |= 0x40; //标志位
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3F]=res;
					USART1_RX_STA++;
					
					if(USART1_RX_STA>63)
							USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}  											 
}

 void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART2->SR & (1<<5))//接收到数据
	{	 
		USART_ClearFlag(USART2,USART_FLAG_ORE);
		res = USART2->DR; 
		//dbgPrintf2(" USART2_IRQHANDLER !\r\n");
		if((USART2_RX_STA & 0x80)==0)//接收未完成
		{
			if(USART2_RX_STA & 0x40)//接收到了0x0d
			{
				if(res!=0x0a)
					USART2_RX_STA = 0;//接收错误,重新开始
				else 
				{
					USART2_RX_STA |= 0x80;
				    USART2_RX_BUF[63]=0;
				}//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(res==0x0d)
					USART2_RX_STA |= 0x40; //标志位
				else
				{
					if(res == 0xAA)
					{
						USART2_RX_STA&=~0x7f;
					}
					USART2_RX_BUF[USART2_RX_STA&0X3F]=res;
					
					USART2_RX_STA++;
					
					if(USART2_RX_STA>63)
							USART2_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}  											 
}
#endif

#endif


//查找字符串,在s中查找t
//返 回 值: 1(t在s中的位置)成功 0失败
u8 LookFor_Str(char *s, char *t)
{
		char    *s_temp;       
		s_temp = s;
		if (s == 0 ||t == 0)                           							//是否为空
		return 0;

		while(*s_temp != *t)   //+IPD,0,3:123 \0   //0D 0A XX XX 0D 0A
		{
			s_temp++;
			if(*s_temp == '\0')
					return 0;
		}
	
		return (u8)(s_temp-s)+1;
}

unsigned char  LookFor_Str_gec(void)
{
	u8 i=0;
	char t_tmp[18]={0};
	unsigned char GEC_OK[34]={0};
	memset(GEC_OK,0,sizeof(GEC_OK));
	GEC_OK[0]=0xAA;
	GEC_OK[1]=0x50;
	GEC_OK[2]=0x1C;
	GEC_OK[32]='\r';
	GEC_OK[33]='\n';
	strcpy(GEC_FLY,"AT+CIPSEND=0,34\r\n");
	
	GEC_handle(t_tmp);
	
	dbgPrintf2(t_tmp);
	
	i = strcmp(t_tmp, "GEC");
	if(i==0)
	{	
		for(i=0;i<10;i++)
			WIFI_SendData(GEC_OK);	
		return (u8)1;
	}
	else 
		return (u8)0;
}


#if(WIFIControl)
unsigned char  Wifi_handle(unsigned char *Wifi)  //返回
{
	 unsigned char len , i, j=0; 
	 //unsigned short int Wifi[64]={1};

	if(USART2_RX_STA & 0x80)
	{					   
			len  =  USART2_RX_STA&0x3f;//得到此次接收到的数据长度
			i = LookFor_Str(USART2_RX_BUF, ":");
			
			if(i == 0)
			{USART2_RX_STA=0;	return 0;}			
			for( ; i< len; i++) //LEN=15
			{
					Wifi[j] = USART2_RX_BUF[i];		// +IPD,0,3:123 \0 0D 0A			

					//if(Wifi[j] == 0)
					//break;
				
				  j++;
			}
			USART2_RX_STA=0;
			return j;

	}
	return 0;
}

#elif (BTControl)
	unsigned char  Wifi_handle(unsigned char *Wifi)  //返回
	{
		 unsigned char len , i, j=0; 
		 //unsigned short int Wifi[64]={1};

		if(USART2_RX_STA & 0x80)
		{					   
				len  =  USART2_RX_STA&0x3f;//得到此次接收到的数据长度	
				for( i=0 ; i< len; i++) //LEN=15
				{
						Wifi[j] = USART2_RX_BUF[i];		//123 \0 0D 0A								
						j++;
				}
				USART2_RX_STA=0;
				return j;

		}
		return 0;
	}
	
#endif


unsigned char  GEC_handle(char *Wifi)  //返回
{
	 unsigned char len , i, j=0; 
	 //unsigned short int Wifi[64]={1};

	if(USART2_RX_STA & 0x80)
	{					   
			len  =  USART2_RX_STA&0x3f;//得到此次接收到的数据长度
			i = LookFor_Str(USART2_RX_BUF, ":");
			USART2_RX_STA=0;
			if(i == 0)
				return 0;
			GEC_FLY[11] = USART2_RX_BUF[i-4];
			
			for( ; i< len; i++) //LEN=15
			{
					Wifi[j] = USART2_RX_BUF[i];		// +IPD,0,3:123 \0 0D 0A			

					if(Wifi[j] == 0)
					break;
				
				  j++;
			}
							
			return j;

	}
	return 0;
}

void wifi_process_msg(void)
{
  #define UART2_BUF_LENGTH 64
  char g_uart2_msg[UART2_BUF_LENGTH];
  int len,i = 0;
  unsigned char msglen = 0;
  char msg[16];

  memset((char*)g_uart2_msg,0,UART2_BUF_LENGTH);
 // Serial_read((char*)g_uart2_msg,&msglen);   
  
  len = msglen;
  sprintf(msg,"len=%d\r\n\0",len);
  uart1_send(msg,strlen(msg));

  for (i = 0; i < len; i++) {
    sprintf(msg,"%d \0",g_uart2_msg[i]);
    uart1_send(msg,strlen(msg)); 
  }
}


void AT_INIT(void)
{
	delay_ms(1000);
/**************************************************************************
*改名字把#if 0 改为 #if 1 , 
*GEC_FLY 为改名字的地方
*改完之后，重新上电。
*再改代码，把1换成0.(当然你能忍受上电之后初始化很久的时间，也可以不用修改)
**************************************************************************/
#if 1                                
	dbgPrintf2("AT+CWMODE=2\r\n");  //sta+ap
 	delay_ms(800);
 	dbgPrintf2("AT+RST\r\n");       //reboot
 	delay_ms(2000);
	dbgPrintf2("AT+CWSAP=\"GEC_FLY\",\"\",11,0\r\n");//修改名字
	delay_ms(800);
	dbgPrintf2("AT+CWMODE=3\r\n");  //sta+ap
 	delay_ms(800);
 	dbgPrintf2("AT+RST\r\n");       //reboot
 	delay_ms(2000);	
#endif
	dbgPrintf2("AT+CIPMUX=1\r\n");  //start server AT+CIPMUX=1
	delay_ms(800);
	dbgPrintf2("AT+CIPSERVER=1\r\n");//build server AT+CIPSERVER=1
	delay_ms(800);
	dbgPrintf2("AT+CIPSTO=2000\r\n");//set timeout AT+CIPSTO=2000
	delay_ms(800);
	
}


/******************************************
*发送格式
*串口给WIFI模块发送  AT指令
* AT+CIPSEND =   0  ,    4
*               给谁   多少字节
*wifi模块返回>
*输入发送的内容
**************************************************/
void WIFI_SendData(u8 buf[])                          //NRF发送数据
{	 
		dbgPrintf2(GEC_FLY);
		delay_us(650);
		dbgPrintf3(buf,34);			  
}

