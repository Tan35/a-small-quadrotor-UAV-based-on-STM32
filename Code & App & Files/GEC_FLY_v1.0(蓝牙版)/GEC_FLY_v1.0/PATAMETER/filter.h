#ifndef		__FILTER_H
#define		__FILTER_H
#include "imu.h"


#define IIR_SHIFT         8

#define GYRO_CF_TAU 	1.2f

extern Struct_3f angle;


int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);
void imuAccIIRLPFilter(Struct_3i16* in, Struct_3i16* out, Struct_3int* storedValues, int32_t attenuation);

float CF_Factor_Cal(float deltaT, float tau);

void DCM_CF(Struct_3f gyro,Struct_3f acc, float deltaT);
void FilterInit(void);


#endif
