#ifndef _FRICTION_H
#define _FRICTION_H

#include "main.h"

void FRICTION_StopMotor(void);

/*********Ħ��������************/
void FRICTION_Ctrl( void );
bool FRIC_RcSwitch( void );
void FRIC_KeyLevel_Ctrl(void);
void Friction_Rc_Ctrl( void );
/****Ħ���ָ�������*****/
void Friction_Ramp(void);
//uint16_t Fric_GetHeatInc(void);
float Fric_GetSpeedReal(void);

#endif
