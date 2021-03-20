#ifndef FRICTION_H
#define FRICTION_H

#include "main.h"

void FRICTION_StopMotor(void);

//遥控模式下的一些标志位
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2

//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1	
#define FRI_SENTRY  2		//哨兵射速

/*********摩擦轮主控************/
void friction_RC_Ctrl(void);
void friction_AUTO_Ctrl(void);
void friction_Init(void);
void FRICTION_StopMotor(void);
bool FRIC_RcSwitch(void);
void Fric_mode(uint16_t Fric_Speed_Level);
/****摩擦轮辅助函数*****/
void Friction_Ramp(void);
//uint16_t Fric_GetHeatInc(void);
float Fric_GetSpeedReal(void);

#endif
