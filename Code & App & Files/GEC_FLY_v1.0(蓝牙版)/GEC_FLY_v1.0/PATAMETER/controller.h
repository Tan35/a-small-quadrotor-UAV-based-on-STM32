

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdbool.h>
#include "pid.h"

#define MPU6050_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6050_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6050_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6050_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

#define	INNER_PERIOD		(IMU_UPDATE_DT)
#define	OUTER_PERIOD		(IMU_UPDATE_DT*2.0)

#define		LOCK		1
#define		UNLOCK	0

extern int MOTOR1;
extern int MOTOR2;
extern int MOTOR3;	
extern int MOTOR4;

extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;

extern float rollRateDesired;
extern float pitchRateDesired;
extern float yawRateDesired;

extern int16_t rollOutput;
extern int16_t pitchOutput;
extern int16_t yawOutput;

extern int Motor_Thr;					   //油门aileron，elevator ，throttle ，rudder
extern int Motor_Ele;					   //俯仰期望
extern int Motor_Ail;					   //横滚期望
extern int Motor_Rud;					   //航向期望
extern u8 lock_flag;
void controllerInit(void);

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 */
void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

/**
 * Make the controller run an update of the rate PID. The output is
 * the actuator force.
 */
void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void controllerResetAllPID(void);

/**
 * Get the actuator output.
 */
void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

void SetPID(u8 *buf);
void GetPID(u8 *buf);

extern void PIDINIT(void);
void SetRatePID(u8 *buf);
void GetRatePID(u8 *buf);
void Getdesireddata(u8 *buf);
void AttitudeToMotors(float roll, float pitch, float yaw);

#endif /* CONTROLLER_H_ */
