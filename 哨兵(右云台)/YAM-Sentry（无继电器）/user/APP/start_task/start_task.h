/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      �������񣬽�һ������������������Դ�������������ȼ�,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"

#include "Remote_Control.h"

void startTast(void);


/*---------------------����ϵͳ״̬-------------------------*/
void System_control(void *pvParameters);

typedef enum
{
    RC   = 0,  
    AUTO  = 1,  

} eRemoteMode;  // ң�ط�ʽ


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;

eRemoteMode SYSTEM_GetRemoteMode( void );
eSystemState SYSTEM_GetSystemState( void );
void system_update(void);
void remote_StateControl(void);
/*---------------------------------------------------*/
/*----------------�����õ�------------------------*/
void LED0(void *pvParameters);
typedef struct
{
	
  const RC_ctrl_t *rc_test;               //����ʹ�õ�ң����ָ��
	
} RC_TEST_t;
/*---------------------------------------------------*/
#endif
