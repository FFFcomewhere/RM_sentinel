#ifndef _CAP_TASK_H
#define _CAP_TASK_H

#include "main.h"
#include "stdbool.h"
#include "CAN_receive.h"

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

#define CAP_TASK_INIT_TIME 57
#define CAP_CONTROL_TIME_MS 2


extern void CapTask(void *pvParameters);


void Cap_UpdateTarget_Power(int16_t power );
void Cap_Init(void);
void Cap_RC_Ctrl(void);
void CAN1_Cap_Send(uint16_t temPower);


#endif
