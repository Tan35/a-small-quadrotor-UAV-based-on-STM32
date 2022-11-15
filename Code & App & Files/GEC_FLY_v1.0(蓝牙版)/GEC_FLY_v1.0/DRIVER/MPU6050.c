

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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_initialize(void) 
{
	MPU6050_GPIO_Init();
	
	Single_WriteI2C(PWR_MGMT_1, 0x00);//��Դ��������ֵ��0x00(��������)
	delay_ms(2);
	Single_WriteI2C(SMPLRT_DIV, 0x00);//�����ǲ����ʣ�1000Hz
	delay_ms(2);
	Single_WriteI2C(CONFIG2, 0x03);//��ͨ�˲�Ƶ�ʣ�42Hz
	delay_ms(2);
	Single_WriteI2C(GYRO_CONFIG, 0x18);//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	delay_ms(2);						
	Single_WriteI2C(ACCEL_CONFIG, 0x10);//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x10(���Լ죬8G��5Hz)
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

