/**********************************************
*�� �� �ţ�         v1.0
*�� �� �ߣ�         ��Ƕ�ɷ�
*����������         ����������
*----------------------------------------------
*������־��
*Ŀ�ģ�
*���ݣ�
*�����ˣ�
*ʱ�䣺
*�����
**********************************************/

#include "includes.h"

static volatile ErrorStatus HSEStartUpStatus = SUCCESS;

int main(void)
{  
	SystemInit();																	//ϵͳ��ʼ��
	RCC_Configuration();														    //RCC����ϵͳʱ��,ʹ�ܸ�����ʱ��
	NVIC_Configuration();													        //ϵͳ�ж�����
	GPIO_Configuration();															//GPIO�ܽ�����
	
	USART1_Configuration();											                //���ڳ�ʼ��
	uart2_init(115200);

// 	dbgPrintf(" Init Ticktack !\r\n");
	cycleCounterInit();																//TICKTAK ��ʼ��
	SysTick_Config(SystemCoreClock / 1000);		
/*************************************************************************
*���Ҫ�޸�wifi����������תAT_INIT();
*************************************************************************/
	//AT_INIT();
	LED_Controls();    															    //LED��˸����
	FilterInit();     																//�˲�����ʼ��
	controllerInit();																//��������ʼ��PID����
  
// 	dbgPrintf(" Init eeprom!\r\n");
	FLASH_Unlock();																	//FLASH�������⺯��
	EE_Init();																		//eeprom��ʼ��
	EE_Read_ACC_GYRO_Offset();													    //��ȡ������У׼ƫ����
	
/*************************************************************************
* 
*���PID��ʧ���ߴ��󣬽���������ע��ȥ�������±�����д������һ�飬
*�ɽ�PID��ԭ��Ȼ������ע�ͣ�����дһ�� 
*/
//EE_Write_PID();
//EE_Write_Rate_PID();
/*************************************************************************/

  	PIDINIT();
	EE_Read_PID();																											  //��ȡ�⻷PID 
	EE_Read_Rate_PID();																//��ȡ�ڻ�PID

// 	dbgPrintf(" Init adc!\r\n");
	ADC_DMA_Init();																    //ADC��ʼ��

// 	dbgPrintf("Init MPU6050...\r\n");
	IIC_Init();																												  //IIC��ʼ��
	MPU6050_initialize();

// 	dbgPrintf("Init Motor...\r\n");
	 motorcontorller_init();                                                       //������Ƴ�ʼ��

}

