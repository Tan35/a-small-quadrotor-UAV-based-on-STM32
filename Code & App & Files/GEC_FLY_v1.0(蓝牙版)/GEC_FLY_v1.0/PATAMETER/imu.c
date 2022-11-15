/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴微处理器
**********************************************/

#include "imu.h"
#include "math.h"
#include "MPU6050.h"
#include "delay.h"
#include "filter.h"

Struct_MPU_Info mpu_info;
u8 mpu6050_buffer[14];	
u8 calibration_flag = 0;
static Struct_3i16   accelLPF;
static Struct_3int   accelStoredFilterValues;
static int32_t    imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

void CalibrateMPU6050(void)
{
	static u8 cali_count = 0;
	static int32_t acc_x=0,acc_y=0,acc_z=0,gyro_x=0,gyro_y=0,gyro_z=0;
	if(!cali_count)
	{
		mpu_info.offset_acc.x = 0;
		mpu_info.offset_acc.y = 0;
		mpu_info.offset_acc.z = 0;
		mpu_info.offset_gyro.x = 0;
		mpu_info.offset_gyro.y = 0;
		mpu_info.offset_gyro.z = 0;
	}
	else if(cali_count<100)
	{
		acc_x += mpu_info.acc.x;
		acc_y += mpu_info.acc.y;
		acc_z += mpu_info.acc.z;
		gyro_x += mpu_info.gyro.x;
		gyro_y += mpu_info.gyro.y;
		gyro_z += mpu_info.gyro.z;
	}
	else
	{
		mpu_info.offset_acc.x = acc_x/100;
		mpu_info.offset_acc.y = acc_y/100;
		mpu_info.offset_acc.z = acc_z/100;
		mpu_info.offset_gyro.x = gyro_x/100;
		mpu_info.offset_gyro.y = gyro_y/100;
		mpu_info.offset_gyro.z = gyro_z/100;
		
		cali_count = 0;
		acc_x=0;
		acc_y=0;acc_z=0;gyro_x=0;gyro_y=0;gyro_z=0;
		calibration_flag = 0;
	}
	cali_count++;

}

void GetMPU6050Data(void)
{
	MPU6050_getMotion(mpu6050_buffer);
	mpu_info.acc.x=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - mpu_info.offset_acc.x;	
	mpu_info.acc.y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - mpu_info.offset_acc.y;
	mpu_info.acc.z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - mpu_info.offset_acc.z;
	mpu_info.gyro.x=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - mpu_info.offset_gyro.x;
	mpu_info.gyro.y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - mpu_info.offset_gyro.y;
	mpu_info.gyro.z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - mpu_info.offset_gyro.z;
	if(calibration_flag)
		CalibrateMPU6050();
}

void EnableCalibration(void)
{
	calibration_flag = 1;
}


float GetDeltaT(uint32_t now)
{
	static uint32_t last;
	float	deltaT = (now - last) * 1e-6;	
	last = now;
	
	return deltaT;
}

void GetEulerAngle(void)
{
	float deltaT;
	Struct_3f gy,ac;

	GetMPU6050Data();	                         //获得姿态参数
	imuAccIIRLPFilter(&mpu_info.acc, &accelLPF, &accelStoredFilterValues,imuAccLpfAttFactor);		//iir滤波
	gy.x = mpu_info.gyro.x * Gyro2Radian;
	gy.y = mpu_info.gyro.y * Gyro2Radian;
	gy.z = mpu_info.gyro.z * Gyro2Radian;
	ac.x = accelLPF.x;
	ac.y = accelLPF.y;
	ac.z = accelLPF.z;
	deltaT = GetDeltaT(GetSysTime_us());
	DCM_CF(gy,ac,deltaT);		               //余弦矩阵获取欧拉角
}




