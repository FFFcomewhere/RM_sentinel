/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
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
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357


extern void chassis_task(void *pvParameters);


//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 5000.0f//15000
#define M3505_MOTOR_SPEED_PID_KI 0.0f  //10
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#define CHASSIS_ACCEL_X_NUM 0.1666666667f    //一阶低通滤波参数
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f



//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
 



//巡逻下的速度参数
#define AUTO_PARA 1.3

#define CHASSIS_SPEED_LOW 0
#define CHASSIS_SPEED_NORMAL 1
#define CHASSIS_SPEED_HIGH 2

  
//底盘电机最大速度
#define MAX_WHEEL_SPEED 1.6f*2  

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1.6f*AUTO_PARA //1.5

//自主模式运动过程的前进速度
#define AUTO_MOVE_SPEED 1.7f*AUTO_PARA




//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

#define CHASSIS_RC_DEADLINE 10




//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define MODE_CHANNEL 0

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002



/*--------------------myself-------------*/
#define Omni_Speed_Max 9000  //速度限幅

#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //电机减数比

#define LIMIT_CHASSIS_MAX         9000     //功率限制情况下底盘单个电机最大输出

//底盘电流限幅
#define iTermChassis_Max          3000     //积分限幅




typedef enum
{
	
	CHASSIS_MECH_MODE = 0,//机械
	CHASSIS_STOP_MODE = 1,//哨兵模式
	CHASSIS_R_MODE = 2,//哨兵向右运动
	CHASSIS_L_MODE = 3,//哨兵向左运动
	
} ChassisCtrlMode;


//底盘电机ID
typedef enum
{
	
	FRON = 0,  // 前
	BACK = 1,  // 后
	
} ChassisWheel;

//传感器
typedef enum
{
	
	LEFT = 0,  // 左
  RIGHT = 1,  // 右
	
} SensorDirection;



typedef struct
{
	bool TO_left;
	bool TO_right;
	bool left;
	bool right;
  bool stop;
}Flag;


void sensor_update(void);
void chassis_feedback_update(void);
void Chassis_Motor_Angle_PID( ChassisWheel Wheel );
void Chassis_MotorOutput(void);
void CHASSIS_REST(void);
void Chassis_Omni_Move_Calculate(void);
void Chassis_Angle_Move_Calculate(void);
void Chassis_Rc_Control(void);
void CHASSIS_CANSend(void);
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed );
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current );
void Chassis_Init(void);
void Chassis_AUTO_Ctrl(void);
void Chassis_Set_Mode(void);
void Chassis_Set_Contorl(void);
void Chassis_Motor_Speed_PID(void);
#endif
