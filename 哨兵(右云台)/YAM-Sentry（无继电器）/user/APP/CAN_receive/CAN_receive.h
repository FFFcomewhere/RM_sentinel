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
#define TRANSFER_CAN CAN1
#define Send_Mode_CAN CAN1
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,

    CAN_REVOLVER_ALL_ID = 0x200,
		CAN_2006_M2_ID = 0x204, 

		CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_PIT_MOTOR_IDR= 0x207,   //ID为3
		CAN_YAW_MOTOR_IDR= 0x208,   //ID为4
		
    CAN_SIGNAL_TRANSFER_ALL_ID = 0X217,  //右云台传输数据值左云台
    CAN_SEND_MODE_ID = 0X250    //就是左云台发送的模式数据
		
	  

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



//发送传感器和云台电机信号
void CAN_CMD_Transfer(int16_t sensor_left,int16_t senser_right, int16_t Pitch_right, int16_t Yaw_right, int16_t Revolver_Final_Output_right);
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
