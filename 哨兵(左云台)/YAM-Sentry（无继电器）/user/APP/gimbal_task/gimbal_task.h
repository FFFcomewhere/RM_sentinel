/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

#define EEE   99999999999     //消除编译警告用的

#define PALST_COMPS_YAW        (-7)     //陀螺仪YAW角速度补偿
#define PALST_COMPS_PITCH      (41)      //陀螺仪PITCH角速度补偿

//#define max_relative_angle 8191
//#define min_relative_angle -8191


// TEST 自己测试  REAL 装配完成
//PID
#define TEST  1
#define REAL  -1
#define MODE   TEST

#if MODE ==	TEST
/*----------------------------------------速度环--------------------------------------------*/
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 8000.0f
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 8.5f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 40.0f  //32
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 3.0f
#define YAW_SPEED_PID_MAX_OUT 10000.0f
#define YAW_SPEED_PID_MAX_IOUT 1000.0f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
//#define PITCH_GYRO_SPEED_PID_KP_ 2000.0f   //5500 由于pid切换过程中导致计算数值变化，所以更改初始PID
#define PITCH_GYRO_SPEED_PID_KP 3000.0f   //5500
#define PITCH_GYRO_SPEED_PID_KI 0.0f       //1.8
#define PITCH_GYRO_SPEED_PID_KD 0.0f      //0.9

#define PITCH_GYRO_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_GYRO_SPEED_PID_MAX_IOUT 5000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_SPEED_PID_KP 80.0f  //32
#define YAW_GYRO_SPEED_PID_KI 0.5f
#define YAW_GYRO_SPEED_PID_KD 4.0f

#define YAW_GYRO_SPEED_PID_MAX_OUT 10000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT 1000.0f

/*----------------------------------------角度环---------------------------	-----------------*/
////**陀螺仪模式**//
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP_ 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_KP 10.0f //15
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.01f
#define PITCH_GYRO_ABSOLUTE_PID_KD 1.5f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 30.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 600.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.1f
#define YAW_GYRO_ABSOLUTE_PID_KD 1.2f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 800.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//**机械模式**//
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f    //15
#define PITCH_ENCODE_RELATIVE_PID_KI 0.2f
#define PITCH_ENCODE_RELATIVE_PID_KD 5.5f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 50.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 10.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 5000.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f

#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 6000.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//注意此处电机反装，max实际对应最小值，例如pitch_max 实际为向下最大限幅

#define max_yaw_relative_angle      4*PI
#define min_yaw_relative_angle     	-4*PI
#define max_pitch_relative_angle     -2.1
#define min_pitch_relative_angle     -2.65

#define auto_yaw_ccw                2.2
#define auto_yaw_cw                 -0.1
#define auto_pitch_up               -2.1 
#define auto_pitch_down              -2.66


#define mid_yaw_angle               1.05                   
#define mid_pitch_angle             -2.1

#define glancing_angle              -PI




#endif

//掉头云台速度
#define TurnSpeed 0.04f

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10


//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN -0.000006f //0.005



//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f


//云台控制周期
#define GIMBAL_CONTROL_TIME 1


//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191


//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;




void GIMBAL_task(void *pvParameters);
/*----------------------------------myself---------------------------------------*/

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);   //云台PID初始化，用于角度环
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);//云台PID计算，用于角度环
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);//云台PID清零


void RC_Set_Mode(void);  //遥控器选择控制模式

void GIMBAL_Set_Control(void); //遥控器模式计算云台的目标值
void GIMBAL_Set_Key_Control(void); //键盘模式计算云台的目标值

void GIMBAL_Behaviour_Control_Set(fp32 add_yaw_mech, fp32 add_yaw_gyro, fp32 add_pitch_mech , fp32 add_pitch_gyro);//云台根据不同状态设置不同的函数
void gimbal_absolute_angle_control(fp32 yaw, fp32 pitch);//云台陀螺仪模式的控制函数，主要写了180°调头

#define PITCH 0
#define YAW 1


#define MECH 0
#define GYRO 1
#define ROLL 2


#define NOW  0
#define LAST 1

//云台模式选择
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_CRUISE_MODE = 1,
} GimbalCtrlMode;



/* 云台操作模式:
   
   普通             	NORMAL
   调头180°             AROUND
   打符             	BUFF
   补弹,pitch水平   	LEVEL
   机械模式pitch抬头	HIGH
   快速扭头90°          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//正常模式,进行模式选择
	GIMBAL_AROUND  = 1,//180°调头
	GIMBAL_TURN    = 2,//90°扭头
	GIMBAL_AUTO    = 3,//自瞄
	
}eGimbalAction;



typedef struct
{
  bool pitch_up;
	bool pitch_down;
	bool yaw_cw;
	bool yaw_ccw;
}Auto_Mode;



typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;

/*       临界值处理结构体        */
typedef struct 
{

	float LastAngle;       //上一次读取的角度
	float CurAngle;	       //当前读取的角度
	float AngleSum;	       //角度累加值
	
}Critical_t;

	

void GIMBAL_InitCtrl(void);
void GIMBAL_Rc_Ctrl( void ); //遥控器模式
void GIMBAL_AUTO_Ctrl(void); //哨兵巡航模式
void GIMBAL_AUTO_Mode_Ctrl(void);
void GIMBAL_AUTO_PREDICT_Mode_Ctrl(void);
void vPitch_Mech_PositionLoop(void);  //error_delta 陀螺仪角速度
void vPitch_Gyro_PositionLoop(void);
void vYaw_Gyro_PositionLoop(void);
void vYaw_Mech_PositionLoop(void);

void GIMBAL_PositionLoop(void);
void GIMBAL_kPID_Init(void);
void GIMBAL_CanSend(void);
void GIMBAL_UpdateCurrent( char ID, int16_t current );
void GIMBAL_UpdateSpeed( char ID, int16_t speed );
void GIMBAL_UpdateAngle( char ID, int16_t angle );
void Gimbal_Error_Read(void); //读取误差值
void GIMBAL_MPU_Update(void);
void MPU_Update_last(void);

void Critical_Handle_Init(Critical_t *critical, float get);
float Gimbal_Yaw_Gryo_AngleSum(Critical_t *critical, float get);
void Gimbal_Chass_Separ_Limit(void);
int16_t GIMBAL_GetOffsetAngle(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);

void GIMBAL_NORMAL_Mode_Ctrl(void);//云台键盘响应模式
void GIMBAL_LEVEL_Mode_Ctrl(void);//补弹模式
void GIMBAL_AUTO_Mode_Ctrl(void);//自瞄模式
void	GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);//云台打符，摄像头在云台
void GIMBAL_BASE_Mode_Ctrl(void);///桥头吊射模式
	

bool GIMBAL_IfAuto_MobPre_Yaw(void);//自瞄yaw轴预测是否已经开启
bool GIMBAL_MOBPRE_YAW_FIRE(void);//yaw轴开启预测的时候云台是否到位
bool GIMBAL_BUFF_YAW_READY(void);//打符YAW轴是否移动到位
bool GIMBAL_BUFF_PITCH_READY(void);//打符PITCH是否移动到位
bool GIMBAL_AUTO_PITCH_SB(void);//是否在自瞄哨兵
bool GIMBAL_AUTO_PITCH_SB_SK(void);//是否在中等距离自瞄哨兵

#endif
