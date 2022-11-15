

#include "MPU6050.h"
#include "IOI2C.h"
#include "delay.h"


void MPU6050_GPIO_Init(void)
{
   	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;

	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;	                              
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  	
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	 /**/


}


void Single_WriteI2C(unsigned char Regs_Addr,unsigned char Regs_Data) 
{  
	IICwriteByte(SlaveAddress, Regs_Addr, Regs_Data);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void) 
{
	MPU6050_GPIO_Init();
	
	Single_WriteI2C(PWR_MGMT_1, 0x00);//电源管理，典型值：0x00(正常启用)
	delay_ms(2);
	Single_WriteI2C(SMPLRT_DIV, 0x00);//陀螺仪采样率，1000Hz
	delay_ms(2);
	Single_WriteI2C(CONFIG2, 0x03);//低通滤波频率，42Hz
	delay_ms(2);
	Single_WriteI2C(GYRO_CONFIG, 0x18);//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	delay_ms(2);						
	Single_WriteI2C(ACCEL_CONFIG, 0x10);//加速计自检、测量范围及高通滤波频率，典型值：0x10(不自检，8G，5Hz)
	delay_ms(2);
}

unsigned char MPU6050_Check_Busy(void)
{
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0){
	  return 1;
	 }
	 else return 0;
}

void MPU6050_getMotion(u8 *buffer)
{
//    if(MPU6050_Check_Busy())  
	 IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
	
}

