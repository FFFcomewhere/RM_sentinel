/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define CAP_CAN CAN1
#define Revolver_CAN CAN1
#define SENSOR_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
		      
    CAN_REVOLVER_ALL_ID = 0x200,
		CAN_2006_M1_ID = 0x203,
	
		CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_PIT_MOTOR_IDR= 0x205,   //ID为1
		CAN_YAW_MOTOR_IDR= 0x206,   //ID为2 
		
		 
    CAN_SIGNAL_TRANSFER_ALL_ID = 0X300,
 
		
	  CAN_CAP_ID=0x211,

} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;   //编码值
    int16_t speed_rpm;    //转速
    int16_t given_current;  //电流值
    uint8_t temperate;
    int16_t last_ecd;  //上一次编码值
} motor_measure_t;

//void GIMBAL_MPU_Update(void);

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t pitchr, int16_t pitchl, int16_t yawr, int16_t yawl);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送拨盘电机控制命令
void CAN_CMD_Revolver(int16_t motor1,int16_t motor2);
//发送传感器信号
void CAN_CMD_Sensor(int16_t left,int16_t right);

//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
