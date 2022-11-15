#ifndef	__FUNC_H
#define	__FUNC_H


#define     LED3(x)       x? (GPIOB->BSRR=GPIO_Pin_13):(GPIOB->BRR=GPIO_Pin_13)
#define     LED4(x)       x? (GPIOB->BSRR=GPIO_Pin_14):(GPIOB->BRR=GPIO_Pin_14)
#define     OP_LED3       (GPIOB->IDR&GPIO_Pin_13)? (GPIOB->BRR=GPIO_Pin_13):(GPIOB->BSRR=GPIO_Pin_13)
#define     OP_LED4       (GPIOB->IDR&GPIO_Pin_14)? (GPIOB->BRR=GPIO_Pin_14):(GPIOB->BSRR=GPIO_Pin_14)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define EE_ACC_X_OFFSET	0
#define EE_ACC_Y_OFFSET	1
#define EE_ACC_Z_OFFSET	2
#define EE_GYRO_X_OFFSET	3
#define EE_GYRO_Y_OFFSET	4
#define EE_GYRO_Z_OFFSET 5
#define EE_PID_ROL_P	6
#define EE_PID_ROL_I	7
#define EE_PID_ROL_D	8
#define EE_PID_PIT_P	9
#define EE_PID_PIT_I	10
#define EE_PID_PIT_D	11
#define EE_PID_YAW_P	12
#define EE_PID_YAW_I	13
#define EE_PID_YAW_D	14

#define EE_RATE_PID_ROL_P	15
#define EE_RATE_PID_ROL_I	16
#define EE_RATE_PID_ROL_D	17
#define EE_RATE_PID_PIT_P	18
#define EE_RATE_PID_PIT_I		19
#define EE_RATE_PID_PIT_D	20
#define EE_RATE_PID_YAW_P	21
#define EE_RATE_PID_YAW_I	22
#define EE_RATE_PID_YAW_D	23

void EE_Write_ACC_GYRO_Offset(void);
void EE_Read_ACC_GYRO_Offset(void);
void EE_Write_PID(void);	//
void EE_Read_PID(void);
void EE_Read_Rate_PID(void);
void EE_Write_Rate_PID(void);	//

u8 CheckSum(u8 *buf,u8 len);
void SetAccGyroOffset(u8 *buf);
void GetAccGyroOffset(u8 *buf);
void GetState(u8 *buf);


#endif

