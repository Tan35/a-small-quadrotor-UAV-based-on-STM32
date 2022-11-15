#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f10x.h"

typedef struct
{
	float x;
	float y;
	float z;
}Struct_3f;

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
}Struct_3int;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Struct_3i16;

typedef struct
{
	Struct_3i16 acc;
	Struct_3i16 gyro;
	Struct_3i16 offset_acc;
	Struct_3i16 offset_gyro;
}Struct_MPU_Info;

#define ACC_1G 			4096		//由加速度计的量程确定
#define RAD_TO_DEG 		57.29577951f //弧度变角度
#define Gyro2Radian		0.0010653	//角速度变成弧度 2000度每秒
#define Gyro2Degree 		0.0610351	//角速度变成度      2000度每秒

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  10

/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */

#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)
#define IMU_LOOP_TIME	(1000000.0/IMU_UPDATE_FREQ)	//单位为uS
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

extern Struct_MPU_Info mpu_info;
void EnableCalibration(void);
void GetEulerAngle(void);



#endif
