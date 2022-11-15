/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴电机控制初始化协议
**********************************************/

#include "includes.h"
#include "string.h"
#include "motorcontroller.h"

#define		RESEND	10

void motorcontorller_init(void)
{
	u8 len;
	u8 RecvBuf[64];
	u8 SendBuf[34]={0};
	u8 offline = 0;
	u8 recv_flag = 0;
	u32 pd2ms=0,pd20ams=0,pd20bms=0,pd100ms=0;
    memset(SendBuf,0,34);
	SendBuf[32]='\r';
	SendBuf[33]='\n';
	Motor_Init();
	Motor_SetPwm(0,0,0,0);
	
	#if 0
	Motor_SetPwm(200,200,200,200);
	delay_ms(500);
	Motor_SetPwm(600,600,600,600);
	delay_ms(2000);
	Motor_SetPwm(400,400,400,400);
	delay_ms(1000);
	Motor_SetPwm(300,300,300,300);
	delay_ms(300);
	Motor_SetPwm(200,200,200,200);
	delay_ms(300);
	Motor_SetPwm(100,100,100,100);
	delay_ms(300);
	Motor_SetPwm(0,0,0,0);
	delay_ms(300);
	
	
	#endif
	//while(!LookFor_Str_gec());  //"+IPD,1,3:GEC\r\n"
  #if 1
	pd20bms = TIMIRQCNT + 10*ITS_PER_MS;
	while(1)
	{
		if(TIMIRQCNT> pd2ms + 2*ITS_PER_MS-1)							// every 2ms
		{
			 
			GetEulerAngle();								           //获取欧拉角
			if(lock_flag==UNLOCK) 
				AttitudeToMotors(angle.y,angle.x,angle.z);		       //计算控制量
			else
			{
				//LED_Controls();
				MOTOR1=0;	 			
				MOTOR2=0;				
				MOTOR3=0;				
				MOTOR4=0;				
			}
			Motor_SetPwm(MOTOR1,MOTOR2,MOTOR3,MOTOR4);			       //控制电机
			pd2ms = TIMIRQCNT;
		}
				
		if(TIMIRQCNT> (pd20ams + 5*ITS_PER_MS-1))					   // every 20ms
		{
				len = Wifi_handle(RecvBuf);
				if((len>10)&&(len<34))
				{
						if( (RecvBuf[RecvBuf[2]+3]==0x1c) &&  (RecvBuf[0]==0xAA))//CheckSum(RecvBuf, RecvBuf[2]+3))&&(RecvBuf[0]==0xAA))	//校验数据
						{
							    
								if(RecvBuf[1]!=0xC0)		
									OP_LED3;		
								offline=0;
								switch(RecvBuf[1])
								{
										case 0xC0:                			                //获得理想参数
												Getdesireddata(RecvBuf);                    //3~10
												break;
										case 0x10:  										//设置PID参数
												SetPID(RecvBuf);                            //3~20
												break;
										case 0x11:  										//设置传感器校准参数
												SetAccGyroOffset(RecvBuf);                  //3~14
												break;
										case 0x12:  										//W Control offset
												break;
										case 0x14:  										//写内环PID
												SetRatePID(RecvBuf);                        //3~20
												break;
										case 0x20:  										//获得 PID
												recv_flag = RESEND;
												GetPID(SendBuf); 
												break;
										case 0x21:  										//得到传感器校准参数
												recv_flag = RESEND;
												GetAccGyroOffset(SendBuf);	
												break;
										case 0x22: 										 //R Control offset
												break;
										case 0x24: 										 //得到内环PID
												recv_flag = RESEND;
												GetRatePID(SendBuf);	
												break;
										case 0x40:										 //校准姿态
												EnableCalibration();
												break;
										case 0x41:										 //校准遥控器零点												
												break;
									 default:
												break;
								}
						}
				}
				pd20ams = TIMIRQCNT;
		}
		if(TIMIRQCNT>(pd20bms + 20*ITS_PER_MS-1))				                       // every 20ms
		{
			if(recv_flag==0)
			{
						GetState(SendBuf);										      //获取本机状态
			
			}else
						recv_flag--;
			WIFI_SendData(SendBuf);										             //返回本机状态
			OP_LED3;
			pd20bms = TIMIRQCNT;
		}
		
		if(TIMIRQCNT>(pd100ms + 100*ITS_PER_MS-1))	                                // every 100ms
		{
			if(offline>20)
			{
				lock_flag = LOCK;
				offline = 0;
			}
			offline++;
			pd100ms = TIMIRQCNT;
		}
	}
	#endif
	
}



