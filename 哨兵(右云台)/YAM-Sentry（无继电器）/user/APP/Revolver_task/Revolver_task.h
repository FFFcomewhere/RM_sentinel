#ifndef REVOLVERTASK_H
#define REVOLVERTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"

//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1		//低速
#define FRI_MID  	2		//中速	
#define FRI_HIGH 	3		//高速      
#define FRI_MAD  	4		//打符射速
#define FRI_SENTRY  5		//哨兵射速

extern void Revolver_task(void *pvParameters);

void REVOLVER_Rest(void);
void REVOL_SpeedLoop(void);
void REVOL_PositionLoop(void);
void REVOL_ceshi_PositionLoop(void);
void REVOL_SpeedStuck(void);
void REVOL_PositStuck(void);
void Revolver_Angle_Rest(void);
void Revolver_Init(void);
void REVOL_UpdateMotorAngleSum(void);
void Revolver_UpdateMotorAngle( int16_t angle );
void Revolver_UpdateMotorCurrent( int16_t current );
void Revolver_UpdateMotorSpeed( int16_t speed );
void Revolver_RC_Ctrl(void);
void Revolver_AUTO_Ctrl(void);
extern void Revolver_CANSend(void);
extern void Revolver_CANSend2(void);
bool REVOLVER_Rc_Switch(void);
void Reset_Fric(void);
#endif
