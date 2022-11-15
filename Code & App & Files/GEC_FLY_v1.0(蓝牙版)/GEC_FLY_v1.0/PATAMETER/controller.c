/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴控制参数
**********************************************/

#include <stdbool.h>
#include "stm32f10x.h"
#include "controller.h"
#include "func.h"
#include "imu.h"
#include "filter.h"


//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;                        //横滚
PidObject pidPitch;                      //俯仰
PidObject pidYaw;						 //航向

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;

int Motor_Thr=0;					   //油门aileron，elevator ，throttle ，rudder
int Motor_Ele=0;					   //俯仰期望
int Motor_Ail=0;					   //横滚期望
int Motor_Rud=0;					   //航向期望

int MOTOR1;
int MOTOR2;
int MOTOR3;	
int MOTOR4;
int NUMBER =1;

float rud_sum = 0;
u8 lock_flag = LOCK;  

void controllerInit()                                       //控制初始化
{
  
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, INNER_PERIOD);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, INNER_PERIOD);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, INNER_PERIOD);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidRollRate, -PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidPitchRate, -PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidYawRate, -PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, OUTER_PERIOD);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, OUTER_PERIOD);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, OUTER_PERIOD);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidRoll, -PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidPitch, -PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimitLow(&pidYaw, -PID_YAW_INTEGRATION_LIMIT);
  
}

//内环PID 计算 ，参数分别为内环实际值和期望值，计算得出电机控制量
void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  rollOutput=pidUpdate(&pidRollRate, rollRateActual, TRUE);

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = pidUpdate(&pidPitchRate, pitchRateActual, TRUE);

  pidSetDesired(&pidYawRate, yawRateDesired);
  TRUNCATE_SINT16(yawOutput, pidUpdate(&pidYawRate, yawRateActual, TRUE));
}

// 外环PID 计算，参数分别为Actual 实际值 Desired 期望值 RateDesired 得出的内环计算值
void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  float yawError;
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, TRUE);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, TRUE);


  // Update PID for yaw axis
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, FALSE);
  
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

int MOTORLimit(float value)
{
  	  if(value>999)
	    {
		  	value=999;
		}
	  else if(value<200)
		{
			value=0;
		}
	  return value;
}	

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
  MOTOR1 = MOTORLimit(thrust - roll + pitch - yaw);
  MOTOR3 = MOTORLimit(thrust + roll + pitch + yaw);
  MOTOR4 = MOTORLimit(thrust + roll - pitch - yaw);
  MOTOR2 = MOTORLimit(thrust - roll - pitch + yaw);
}

 float rollRateDesired;
 float pitchRateDesired;
 float yawRateDesired;

void AttitudeToMotors(float roll, float pitch, float yaw)
{
	static u8 flag = 0;

	if(flag%2==0)	//每两次执行一次外环PID计算
	{
		controllerCorrectAttitudePID(roll, pitch, yaw, Motor_Ail, Motor_Ele, Motor_Rud, &rollRateDesired, &pitchRateDesired, &yawRateDesired);
//		yawRateDesired = Motor_Rud;
	}
	flag++;

	//内环PID
	controllerCorrectRatePID(mpu_info.gyro.y* Gyro2Degree, mpu_info.gyro.x* Gyro2Degree, mpu_info.gyro.z* Gyro2Degree,  
													 rollRateDesired, pitchRateDesired, yawRateDesired);
	distributePower(Motor_Thr, rollOutput, pitchOutput, yawOutput);
}

void SetPID(u8 *buf)                        //写死外环的PID
{
    float p,i,d;
    p = ((u16)((buf[3]<<8)|buf[4]))/100.0;
    i = ((u16)((buf[5]<<8)|buf[6]))/100.0;
    d = ((u16)((buf[7]<<8)|buf[8]))/100.0;
    pidInit(&pidRoll, 0, p, i, d, OUTER_PERIOD);
	
    p = ((u16)((buf[9]<<8)|buf[10]))/100.0;
    i = ((u16)((buf[11]<<8)|buf[12]))/100.0;
    d = ((u16)((buf[13]<<8)|buf[14]))/100.0;
    pidInit(&pidPitch, 0, p, i, d, OUTER_PERIOD);
	
    p = ((u16)((buf[15]<<8)|buf[16]))/100.0;
    i = ((u16)((buf[17]<<8)|buf[18]))/100.0;
    d = ((u16)((buf[19]<<8)|buf[20]))/100.0;
    pidInit(&pidYaw, 0, p, i, d, OUTER_PERIOD);
	EE_Write_PID();
}
void GetPID(u8 *buf)
{
	u16 tmp16;
	
	buf[0] = 0xAA;
	buf[1] = 0x20;
	buf[2] = 28;
	tmp16 = (u16)(pidRoll.kp*100);
	buf[3] = BYTE1(tmp16);
	buf[4] = BYTE0(tmp16);
	tmp16 = (u16)(pidRoll.ki*100);
	buf[5] = BYTE1(tmp16);
	buf[6] = BYTE0(tmp16);
	tmp16 = (u16)(pidRoll.kd*100);
	buf[7] = BYTE1(tmp16);
	buf[8] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitch.kp*100);
	buf[9] = BYTE1(tmp16);
	buf[10] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitch.ki*100);
	buf[11] = BYTE1(tmp16);
	buf[12] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitch.kd*100);
	buf[13] = BYTE1(tmp16);
	buf[14] = BYTE0(tmp16);
	tmp16 = (u16)(pidYaw.kp*100);
	buf[15] = BYTE1(tmp16);
	buf[16] = BYTE0(tmp16);
	tmp16 = (u16)(pidYaw.ki*100);
	buf[17] = BYTE1(tmp16);
	buf[18] = BYTE0(tmp16);
	tmp16 = (u16)(pidYaw.kd*100);
	buf[19] = BYTE1(tmp16);
	buf[20] = BYTE0(tmp16); 
 	buf[21] = 0;
	buf[22] = 0;
	buf[23] = 0;
	buf[24] = 0;
	buf[25] = 0;
	buf[26] = 0;
	buf[27] = 0;
	buf[28] = 0;
	buf[29] = 0;
	buf[30] = 0;
	buf[31] = CheckSum(buf,buf[2]+3);
 }

void SetRatePID(u8 *buf)                                   //设置内环PID的值
{
    float p,i,d;
    p = ((u16)((buf[3]<<8)|buf[4]))/100.0;
    i = ((u16)((buf[5]<<8)|buf[6]))/100.0;
    d = ((u16)((buf[7]<<8)|buf[8]))/100.0;
    pidInit(&pidRollRate, 0, p, i, d, INNER_PERIOD);
	
    p = ((u16)((buf[9]<<8)|buf[10]))/100.0;
    i = ((u16)((buf[11]<<8)|buf[12]))/100.0;
    d = ((u16)((buf[13]<<8)|buf[14]))/100.0;
    pidInit(&pidPitchRate, 0, p, i, d, INNER_PERIOD);
	
    p = ((u16)((buf[15]<<8)|buf[16]))/100.0;
    i = ((u16)((buf[17]<<8)|buf[18]))/100.0;
    d = ((u16)((buf[19]<<8)|buf[20]))/100.0;
    pidInit(&pidYawRate, 0, p, i, d, INNER_PERIOD);
	  EE_Write_Rate_PID();
}


void GetRatePID(u8 *buf)                          
{
	u16 tmp16;
	
	buf[0] = 0xAA;
	buf[1] = 0x24;
	buf[2] = 28;
	tmp16 = (u16)(pidRollRate.kp*100);
	buf[3] = BYTE1(tmp16);
	buf[4] = BYTE0(tmp16);
	tmp16 = (u16)(pidRollRate.ki*100);
	buf[5] = BYTE1(tmp16);
	buf[6] = BYTE0(tmp16);
	tmp16 = (u16)(pidRollRate.kd*100);
	buf[7] = BYTE1(tmp16);
	buf[8] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitchRate.kp*100);
	buf[9] = BYTE1(tmp16);
	buf[10] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitchRate.ki*100);
	buf[11] = BYTE1(tmp16);
	buf[12] = BYTE0(tmp16);
	tmp16 = (u16)(pidPitchRate.kd*100);
	buf[13] = BYTE1(tmp16);
	buf[14] = BYTE0(tmp16);
	tmp16 = (u16)(pidYawRate.kp*100);
	buf[15] = BYTE1(tmp16);
	buf[16] = BYTE0(tmp16);
	tmp16 = (u16)(pidYawRate.ki*100);
	buf[17] = BYTE1(tmp16);
	buf[18] = BYTE0(tmp16);
	tmp16 = (u16)(pidYawRate.kd*100);
	buf[19] = BYTE1(tmp16);
	buf[20] = BYTE0(tmp16);   
 	buf[21] = 0;
	buf[22] = 0;
	buf[23] = 0;
	buf[24] = 0;
	buf[25] = 0;
	buf[26] = 0;
	buf[27] = 0;
	buf[28] = 0;
	buf[29] = 0;
	buf[30] = 0;
	buf[31] = CheckSum(buf,buf[2]+3); 
}

void Getdesireddata(u8 *buf)                                 //理想参数
{
	static u8 flag=0;
	
	int ch1, ch2, ch3, ch4;
	ch1 = (buf[3]<<8)|buf[4];
	ch2 = (buf[5]<<8)|buf[6];
	ch3 = (buf[7]<<8)|buf[8];
	ch4 = (buf[9]<<8)|buf[10];
	if(ch1<10)
	{
		lock_flag = LOCK;
		flag = 0;
	}
	else
		lock_flag = UNLOCK;
	Motor_Thr=ch1;					   			  			 //油门期望
	ch2=(int)(ch2-1500);					     			 //航向期望
    Motor_Ail=(int)(1500- ch3);					             //横滚期望
	Motor_Ele=(int)(1500 - ch4);					         //俯仰期望 值域

	Motor_Ele=Motor_Ele/100.0;							     //转化单位为角度-15~15度
	Motor_Ail=Motor_Ail/100.0;
	ch2 /=30;
	if(ch2*ch2<25)
		ch2=0;
	rud_sum += (ch2/10.0);  								 //
	Motor_Rud = rud_sum;
				
	if((Motor_Thr>10)&&(flag==0))						    //复位 YAW
	{
		flag++;
		angle.z = 0;
		Motor_Rud = 0;
		rud_sum = 0;
		pidYaw.prevError = 0;
		pidYaw.integ = 0;
		pidYawRate.prevError = 0;
		pidYawRate.integ = 0;
	}

	LED3(!buf[17]);
	LED4(!buf[18]);
		
}



/*
*初始化 给定特定的PID值和姿态参数
*/

void SetPIDInit(void)                        //写死外环的PID
{
    float p,i,d;
    p = 80;
    i = 0.0;
    d = 0.0;
    pidInit(&pidRoll, 0, p, i, d, OUTER_PERIOD);
	
    p = 80;
    i = 0.0;
    d = 0.0;
    pidInit(&pidPitch, 0, p, i, d, OUTER_PERIOD);
	
    p = 10;
    i = 0.0;
    d = 0.0;
    pidInit(&pidYaw, 0, p, i, d, OUTER_PERIOD);
	EE_Write_PID();
}

void SetRatePIDInit(void)                                   //设置内环PID的值
{
    float p,i,d;
    p = 0.7;
    i = 0.2;
    d = 1.2;
    pidInit(&pidRollRate, 0, p, i, d, INNER_PERIOD);
	
    p = 0.7;
    i = 0.2;
    d = 1.2;
    pidInit(&pidPitchRate, 0, p, i, d, INNER_PERIOD);
	
    p = 10;
    i = 3;
    d = 0.0;
    pidInit(&pidYawRate, 0, p, i, d, INNER_PERIOD);
	  EE_Write_Rate_PID();
}

void PIDINIT(void)
{
	SetPIDInit(); 
	SetRatePIDInit();
}








