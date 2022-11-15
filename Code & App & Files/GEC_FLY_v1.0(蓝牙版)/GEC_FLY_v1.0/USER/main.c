/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴主程序
*----------------------------------------------
*更新日志：
*目的：
*内容：
*更新人：
*时间：
*结果：
**********************************************/

#include "includes.h"

static volatile ErrorStatus HSEStartUpStatus = SUCCESS;

int main(void)
{  
	SystemInit();																	//系统初始化
	RCC_Configuration();														    //RCC配置系统时钟,使能各外设时钟
	NVIC_Configuration();													        //系统中断配置
	GPIO_Configuration();															//GPIO管教配置
	
	USART1_Configuration();											                //串口初始化
	uart2_init(115200);

// 	dbgPrintf(" Init Ticktack !\r\n");
	cycleCounterInit();																//TICKTAK 初始化
	SysTick_Config(SystemCoreClock / 1000);		
/*************************************************************************
*如果要修改wifi的名字请跳转AT_INIT();
*************************************************************************/
	//AT_INIT();
	LED_Controls();    															    //LED闪烁控制
	FilterInit();     																//滤波器初始化
	controllerInit();																//控制器初始化PID参数
  
// 	dbgPrintf(" Init eeprom!\r\n");
	FLASH_Unlock();																	//FLASH解锁，库函数
	EE_Init();																		//eeprom初始化
	EE_Read_ACC_GYRO_Offset();													    //读取传感器校准偏移量
	
/*************************************************************************
* 
*如果PID丢失或者错误，将下面两行注释去掉，重新编译烧写，运行一遍，
*可将PID还原，然后重新注释，再烧写一遍 
*/
//EE_Write_PID();
//EE_Write_Rate_PID();
/*************************************************************************/

  	PIDINIT();
	EE_Read_PID();																											  //读取外环PID 
	EE_Read_Rate_PID();																//读取内环PID

// 	dbgPrintf(" Init adc!\r\n");
	ADC_DMA_Init();																    //ADC初始化

// 	dbgPrintf("Init MPU6050...\r\n");
	IIC_Init();																												  //IIC初始化
	MPU6050_initialize();

// 	dbgPrintf("Init Motor...\r\n");
	 motorcontorller_init();                                                       //电机控制初始化

}

