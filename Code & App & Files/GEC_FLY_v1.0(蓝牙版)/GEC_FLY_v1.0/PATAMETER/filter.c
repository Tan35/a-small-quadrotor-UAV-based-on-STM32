/**********************************************
*版 本 号：         v1.0
*创 建 者：         粤嵌股份
*功能描述：         四轴滤波
**********************************************/

#include "stm32f10x.h"
#include "filter.h"
#include <stdio.h>                    /* standard I/O .h-file                 */
#include <string.h>                   /* string and memory functions          */
#include <math.h>

float gyro_cf;
Struct_3f angle;

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  if (attenuation > (1<<IIR_SHIFT))
  {
    attenuation = (1<<IIR_SHIFT);
  }
  else if (attenuation < 1)
  {
    attenuation = 1;
  }

  // Shift to keep accuracy
  inScaled = in << IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}

void imuAccIIRLPFilter(Struct_3i16* in, Struct_3i16* out, Struct_3int* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

//互补滤波器系数计算
float CF_Factor_Cal(float deltaT, float tau)
{
	return tau / (deltaT + tau);
}

//一阶互补滤波器
void CF_1st(Struct_3f *gyroData, Struct_3f accData, float cf_factor)
{ 
	gyroData->x = gyroData->x* cf_factor + accData.x*(1 - cf_factor);
	gyroData->y = gyroData->y* cf_factor + accData.y*(1 - cf_factor);
	gyroData->z = gyroData->z* cf_factor + accData.z*(1 - cf_factor);
}


void from_euler(Struct_3f euler, Struct_3f *g, Struct_3f *m)
{
		float sinx = sinf(euler.x);
    float cosx = cosf(euler.x);
    float siny = sinf(euler.y);
    float cosy = cosf(euler.y);
    float sinz = sinf(euler.z);
    float cosz = cosf(euler.z);
    Struct_3f a,b,c,d;

    a.x = cosy * cosz;
    a.y = (sinx * siny * cosz) + (cosx * sinz);
    a.z = -(cosx * siny * cosz) + (sinx * sinz);
    b.x = -cosy * sinz;
    b.y = -(sinx * siny * sinz) + (cosx * cosz);
    b.z = (cosx * siny * sinz) + (sinx * cosz);
    c.x = siny;
    c.y = -sinx * cosy;
    c.z = cosx * cosy;

    d.x = a.x*g->x + a.y*g->y + a.z*g->z;
    d.y = b.x*g->x + b.y*g->y + b.z*g->z;
    d.z = c.x*g->x + c.y*g->y + c.z*g->z;
    g->x = d.x;
    g->y = d.y;
    g->z = d.z;
    
    d.x = a.x*m->x + a.y*m->y + a.z*m->z;
    d.y = b.x*m->x + b.y*m->y + b.z*m->z;
    d.z = c.x*m->x + c.y*m->y + c.z*m->z;
    m->x = d.x;
    m->y = d.y;
    m->z = d.z;

}

//弧度转角度
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

void get_rollpitch(Struct_3f g)
{
	angle.x = degrees(atan2f(g.y,g.z));
	angle.y = degrees(atan2f(-g.x, sqrtf(g.y * g.y + g.z* g.z)));	
}
void get_yaw(Struct_3f m)
{
	angle.z = degrees(atan2f(m.y,m.x));
}



//余弦矩阵计算姿态
void DCM_CF(Struct_3f gyro,Struct_3f acc, float deltaT)
{
	static Struct_3f deltaGyroAngle, LastGyro;
	static Struct_3f Vector_G = {0, 0, ACC_1G}, Vector_M = {1000, 0, 0};
	
	//计算陀螺仪角度变化，二阶龙格库塔积分	
	deltaGyroAngle.x = (gyro.x + LastGyro.x) * 0.5 * deltaT;
	LastGyro.x = gyro.x;
	deltaGyroAngle.y = (gyro.y + LastGyro.y) * 0.5 * deltaT;
	LastGyro.y = gyro.y;
	deltaGyroAngle.z = (gyro.z + LastGyro.z) * 0.5 * deltaT;
	LastGyro.z = gyro.z;
	
	//计算表示单次旋转的余弦矩阵
	from_euler(deltaGyroAngle,&Vector_G,&Vector_M);
		
	//互补滤波，使用加速度测量值矫正角速度积分漂移
	CF_1st(&Vector_G, acc, gyro_cf);

	//计算飞行器的ROLL和PITCH
	get_rollpitch(Vector_G);	
	
	//计算飞行器的YAW
	angle.z += mpu_info.gyro.z* Gyro2Degree * deltaT;
}


void FilterInit(void)
{
	//互补滤波器系数计算
	gyro_cf = CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);	

}

