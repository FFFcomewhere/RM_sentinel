/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       AV  DV左右调头 弹仓开启 云台不动模式 未明。   卡尔曼滤波还需要了解，自瞄，打符控制还需要了解。
  * @history    板子充电口朝我   陀螺仪排针超外   左正右负    电机充电口朝我  R字 左正右负    电流左正右负       电机反装的话  左负右正   电流左负右正
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "gimbal_task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "main.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "chassis_task.h"


#define gimbal_total_pid_clear(void)               \
    {                                              \
        Gimbal_PID_clear(&Gimbal_Yaw_Mech_PID);    \
				PID_clear(&gimbal_yaw_motor_mech_pid);     \
			  Gimbal_PID_clear(&Gimbal_Pitch_Mech_PID);  \
			  PID_clear(&gimbal_pitch_motor_mech_pid);   \
    }

		
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }	
		
/*---------------------------------------------------限幅-------------------------------------------------------------------*/

		
/*--------------------------------------myself-------------------------------*/
extern  RC_ctrl_t rc_ctrl;    //定义遥控器结构体参数
GimbalCtrlMode  modeGimbal;   //定义云台控制模式    机械/陀螺仪
eGimbalAction  actGimbal;     //定义云台运动模式  调头 自瞄 打符等
Critical_t Yaw_Gyro_Angle;    

extern VisionRecvData_t VisionRecvData; //定义视觉接收的数据结构体

Gimbal_PID_t Gimbal_Yaw_Mech_PID;      //PID一系列结构体
Gimbal_PID_t Gimbal_Yaw_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_Mech_PID;
Gimbal_PID_t Gimbal_Pitch_Gyro_PID;
PidTypeDef gimbal_yaw_motor_gyro_pid;
PidTypeDef gimbal_pitch_motor_gyro_pid;
PidTypeDef gimbal_yaw_motor_mech_pid;
PidTypeDef gimbal_pitch_motor_mech_pid;
/*-----------------------------------------------------PID参数----------------------------------------------------------------------------*/

//陀螺仪参数
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值
float angleMpu[3][2];

//机械角度中间变量,从CAN中读取数据
int16_t  angleMotorPit,  angleMotorYaw; 
int16_t  speedMotorPit,  speedMotorYaw; 
int16_t  currentMotorPit,  currentMotorYaw; 

float PitchAngle, YawAngle;
//期望角度
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro
extern float Cloud_Angle_Target_GD[2][2];   //  pitch/yaw    mech/gyro  定义在key_control.c里

//测量角度
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

//测量电机转速
float Cloud_Speed_Measure[2][2];//  pitch/yaw    mech/gyro

//测量电机电流值
float Cloud_Current_Measure[2][2];//  pitch/yaw    mech/gyro

//测量角速度
float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro


float motor_gyro_set[2][2];  //PID计算外环结果，角速度设定值  pitch/yaw    mech/gyro
float motor_gyro_set[2][2]; 

float current_set[2][2];      //PID计算内环结果，输出  pitch/yaw    mech/gyro
float current_set[2][2];

float given_current[2][2];     //PID最终赋值变量  pitch/yaw    mech/gyro
float given_current[2][2];
float fMotorOutput[4] = {0};

float Error[2][2]; //误差值读取
/*---------------------------------------------------*自瞄*----------------------------------------------------------------------------*/
Auto_Mode auto_mode;
bool Auto_Mode_falg = 0;
uint32_t Auto_Mode_Count = 0;

float pitch_angle_raw;
float yaw_angle_raw;
float Auto_Error_Pitch[2];
float Auto_Error_Yaw[2];
float Auto_Distance;
uint32_t Gimbal_Vision_Time[2];

//下层自瞄预测用
float Auto_Distance;//预测距离
float vision_time_update_time;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
float yaw_speed_k = 0;//yaw速度预测比例
float kf_yaw_angcon = 0;//yaw预测比例限幅
float pitch_speed_k = 0;//pitch速度预测比例
float kf_pitch_angcon = 0;//pitch预测比例限幅
float debug_kf_y_angle;//yaw预测暂存
float debug_kf_p_angle;//pitch预测暂存
float debug_kf_angle_temp;//预测角度斜坡暂存量
float debug_kf_angle_ramp = 20;//预测角度斜坡变化量
float kf_speed_yl = 0;//速度过低关闭预测
uint16_t Auto_KF_Delay = 0;//自瞄突然开启,卡尔曼滤波开启延时
float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
float debug_y_sb_sk;//哨兵预测系数
float debug_y_sb_brig_sk;//桥头哨兵
float debug_p_sk;//移动预测系数,越大预测越多
float debug_auto_err_y=120;// = 10;//15;//10;//15;//yaw角度过大关闭预测              具体值在程序中还需要修改               
float debug_auto_err_p;//pitch角度过大关闭预测
float debug_kf_delay=80;// = 150;//100;//200;//120;//150;//预测延时开启              具体值在程序中还需要修改
float debug_kf_speed_yl;//yaw速度过低关闭预测
float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
float debug_kf_speed_yh;//yaw速度过高关闭预测
float debug_kf_speed_pl;//pitch速度过低关闭预测
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
float debug_kf_p_angcon;//pitch预测量限幅


/*自动打弹用的一些标志位*/
bool Mobility_Prediction_Yaw = FALSE;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪

uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖


/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2
extKalman_t Vision_Distance_Kalman;                     //定义视觉距离卡尔曼滤波结构体
speed_calc_data_t Vision_Yaw_speed_Struct;              //定义视觉yaw速度测速结构体
speed_calc_data_t Vision_Pitch_speed_Struct;            //定义视觉pitch速度测速结构体
kalman_filter_t yaw_kalman_filter;                      //定义yaw卡尔曼滤波器结构体
kalman_filter_t pitch_kalman_filter;                    //定义pitch卡尔曼滤波器结构体

/*----------------------------------------------------------遥控器相关变量-------------------------------------------------------------------*/

//上电斜坡变量
float Slope_Begin_Pitch = 0.005  ;  //刚上电时移动快慢
float Slope_Begin_Yaw = 0.005 ;

float rc_add_yaw, rc_add_pit;       //遥控器增量
int16_t yaw_channel, pitch_channel; //遥控器中间变量
extern uint8_t Vision_Get_New_Data;

bool op=0;

//每2ms执行一次任务函数
void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		
		/* 代码段 */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING && Cloud_Angle_Measure[PITCH][MECH]!=0)//初始化模式 这里pitch上电狂甩，所以加了判断
       GIMBAL_InitCtrl();
		
		else
		{
			if(SYSTEM_GetRemoteMode() == RC)
			{
        RC_Set_Mode();
			  GIMBAL_Set_Control();
			  actGimbal = GIMBAL_NORMAL;
			}
			if(SYSTEM_GetRemoteMode() == AUTO)
			{
				if(op==0)
				{
					modeGimbal = CLOUD_MECH_MODE;   //初始pid默认机械模式防止掉头
					op=1;
				}
				if(Vision_If_Update()==TRUE && Auto_Mode_Count >= 5)    //数据更新
				{				
						modeGimbal = CLOUD_CRUISE_MODE;
						GIMBAL_AUTO_Mode_Ctrl();
				}
				else                            //视觉无数据更新
				{
					if(Vision_If_Update()==TRUE)
						Auto_Mode_Count++;
					else
						Auto_Mode_Count=0;
						
//					modeGimbal=CLOUD_MECH_MODE;
//					GIMBAL_AUTO_Ctrl();
				}
			}

		}
		GIMBAL_PositionLoop();
		GIMBAL_CanSend();


		

		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}


/**
  * @brief  云台初始化,主要是PID初始化
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_InitCtrl(void)
{	
		static bool bAngleRecord  = FALSE;
	  static portTickType  ulTimeCurrent = 0;
	  auto_mode.pitch_up = TRUE;
  	auto_mode.pitch_down = FALSE;
	  auto_mode.yaw_cw = TRUE;
	  auto_mode.yaw_ccw = FALSE;
	
	  if (xTaskGetTickCount( ) - ulTimeCurrent > TIME_STAMP_100MS)//保证不断电情况下下次可用
		   bAngleRecord = FALSE;

		 
		ulTimeCurrent = xTaskGetTickCount( );
		 
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]   = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		static const fp32 Pitch_gyro_speed_pid[3] = {PITCH_GYRO_SPEED_PID_KP, PITCH_GYRO_SPEED_PID_KI, PITCH_GYRO_SPEED_PID_KD};
    static const fp32 Yaw_gyro_speed_pid[3]   = {YAW_GYRO_SPEED_PID_KP, YAW_GYRO_SPEED_PID_KI, YAW_GYRO_SPEED_PID_KD};
		modeGimbal = CLOUD_MECH_MODE;
    //初始化yaw电机pid		
		GIMBAL_PID_Init(&Gimbal_Yaw_Gyro_PID,YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Yaw_Mech_PID,YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_yaw_motor_gyro_pid, PID_POSITION, Yaw_gyro_speed_pid, YAW_GYRO_SPEED_PID_MAX_OUT, YAW_GYRO_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_yaw_motor_mech_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);		
    //初始化pitch电机pid_
		GIMBAL_PID_Init(&Gimbal_Pitch_Gyro_PID, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_Mech_PID, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_pitch_motor_gyro_pid, PID_POSITION, Pitch_gyro_speed_pid, PITCH_GYRO_SPEED_PID_MAX_OUT, PITCH_GYRO_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_pitch_motor_mech_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);		
		
		gimbal_total_pid_clear();		
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		
			//记录上电时云台机械角度
	  if (bAngleRecord == FALSE)
	  {
	  	bAngleRecord = TRUE;	
		  Cloud_Angle_Target[PITCH][MECH] =   Cloud_Angle_Measure[PITCH][MECH];
		  Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	  } 
		
		//平缓地让云台移动到中间,防止刚上电狂甩
	  Cloud_Angle_Target[PITCH][MECH] = RAMP_float( mid_pitch_angle, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
	  Cloud_Angle_Target[YAW][MECH]   = RAMP_float( mid_yaw_angle, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);
}
/*-----------------------------------------------云台遥控器控制模式选择和遥控器目标值计算--------------------------------------------------*/
/**
  * @brief  云台遥控器控制模式
  * @param  void
  * @retval void
  * @attention 
  */
void RC_Set_Mode(void)
{
	if(IF_RC_SW2_DOWN)
		modeGimbal = CLOUD_CRUISE_MODE;
	
	else
		modeGimbal = CLOUD_MECH_MODE;
}

/**
  * @brief  计算云台的目标值
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_Set_Control(void)
{
		Auto_Mode_falg = 1;
	  //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
    rc_deadline_limit(RC_CH0_RLR_OFFSET, yaw_channel, RC_deadband);
    rc_deadline_limit(RC_CH1_RUD_OFFSET, pitch_channel, RC_deadband);	
	
    rc_add_yaw = yaw_channel * Yaw_RC_SEN ;
    rc_add_pit = pitch_channel * Pitch_RC_SEN ;		
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
    VisionRecvData.identify_target = FALSE;		
#if YAW_POSITION ==	YAW_DOWN
    if (Cloud_Angle_Target[YAW][MECH] > min_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
		
    else if (Cloud_Angle_Target[YAW][MECH] < max_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
		
#else
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
		
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
		
#endif	
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		Cloud_Angle_Target[YAW][MECH]-= rc_add_yaw;		
	
//		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
//		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //是否超过最大 最小值
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
	}
	
	Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
	Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][MECH];
}



/*---------------------------------哨兵模式下pitch轴上下转动，yaw轴360度转----------------------------------*/
float erro_pitch = 0.0f;
float erro_yaw = 0.0f;

void GIMBAL_AUTO_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE; //自主模式下使用机械模式

	Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
	
	if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
			Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
	else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
			Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;

/*---------------------yaw轴----------------------*///装上电滑环以后360度转，现在不能
	
	if(auto_mode.yaw_cw == TRUE)
	{			
		Cloud_Angle_Target[YAW][MECH] = RAMP_float( auto_yaw_cw, Cloud_Angle_Target[YAW][MECH], 0.02 );
		erro_yaw = Cloud_Angle_Measure[YAW][MECH]-auto_yaw_cw ;
		if(erro_yaw < 0.15f )
		{
			auto_mode.yaw_cw = FALSE;
			auto_mode.yaw_ccw = TRUE;
		}
	}

	else if(auto_mode.yaw_ccw == TRUE)
	{			
		Cloud_Angle_Target[YAW][MECH] = RAMP_float( auto_yaw_ccw, Cloud_Angle_Target[YAW][MECH], 0.02 );
		erro_yaw = auto_yaw_ccw-Cloud_Angle_Measure[YAW][MECH];
		if(erro_yaw < 0.15f)
		{
			auto_mode.yaw_cw = TRUE;
			auto_mode.yaw_ccw = FALSE; 
		}
	}	
	
	
/*---------------------pitch轴--------------------*/	
	if(auto_mode.pitch_up == TRUE)
	{			
		Cloud_Angle_Target[PITCH][MECH] = RAMP_float( auto_pitch_up, Cloud_Angle_Target[PITCH][MECH], 0.03 );
		erro_pitch = auto_pitch_up - Cloud_Angle_Measure[PITCH][MECH];
		if( erro_pitch < 0.15f )
		{
			auto_mode.pitch_up = FALSE;
			auto_mode.pitch_down = TRUE;
		}
	}
	
	else if(auto_mode.pitch_down == TRUE)
	{		
    Cloud_Angle_Target[PITCH][MECH] = RAMP_float( auto_pitch_down, Cloud_Angle_Target[PITCH][MECH], 0.01);		
		erro_pitch =Cloud_Angle_Measure[PITCH][MECH] - auto_pitch_down;
		if(erro_pitch  < 0.01f)
		{
			auto_mode.pitch_up = TRUE;
			auto_mode.pitch_down = FALSE;
		}
	}
	
	
}



/*---------------------------------自瞄----------------------------------*/
/**
  * @brief  云台哨兵控制模式
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_AUTO_Mode_Ctrl(void)
{
		
		if(Vision_If_Update()==TRUE)
		{  
		static float pitch_angle_ref;
		static float yaw_angle_ref;
		Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
		Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
		Vision_Get_Distance(&Auto_Distance);
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][MECH]-Auto_Error_Pitch[NOW]);
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][MECH]+Auto_Error_Yaw[NOW]);
		Vision_Clean_Update_Flag();//清零,否则会一直执行
			Auto_Error_Pitch[NOW] = 0;
			Auto_Error_Yaw[NOW]= 0 ;
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//获取新数据到来的时间
		
		if(VisionRecvData.identify_target == TRUE)                     //识别到了目标
		{
			Cloud_Angle_Target[YAW][GYRO] = yaw_angle_ref;
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}

		
		if (Cloud_Angle_Target[PITCH][GYRO] > max_pitch_relative_angle)
				Cloud_Angle_Target[PITCH][GYRO] = max_pitch_relative_angle;

		else if (Cloud_Angle_Target[PITCH][GYRO] < min_pitch_relative_angle)
				Cloud_Angle_Target[PITCH][GYRO] = min_pitch_relative_angle;

		if (Cloud_Angle_Target[YAW][GYRO] > max_yaw_relative_angle)
				Cloud_Angle_Target[YAW][GYRO] = max_yaw_relative_angle;

		else if (Cloud_Angle_Target[YAW][GYRO] < min_yaw_relative_angle)
				Cloud_Angle_Target[YAW][GYRO] = min_yaw_relative_angle;
		
		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
		
	}
}
/*----------------------------自瞄预测----------------------------------*/

void GIMBAL_AUTO_PREDICT_Mode_Ctrl(void)
{	
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度
	
	Mobility_Prediction_Yaw = FALSE;
	Mobi_Pre_Yaw_Fire = FALSE;
	
	//获取角度偏差量,已经转化为弧度类型
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman,Auto_Distance);
	
	
	if(Vision_If_Update() == TRUE)                    //数据更新
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][GYRO]+Auto_Error_Pitch[NOW]);//得到的角度误差后面可能需要放大或者加上补偿
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);//得到的角度误差后面可能需要放大或者加上补偿
		Vision_Clean_Update_Flag();//清零,否则会一直执行
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//获取新数据到来的时间
	}
	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //更新卡尔曼滤波测量值
	{
		vision_time_update_time = Gimbal_Vision_Time[NOW] - Gimbal_Vision_Time[LAST];//计算视觉延迟
		pitch_angle_raw = pitch_angle_ref;//更新二阶卡尔曼滤波测量值
		yaw_angle_raw = yaw_angle_ref;
		Gimbal_Vision_Time[NOW] = Gimbal_Vision_Time[LAST];
	}
	
	if(VisionRecvData.identify_target == TRUE)                     //识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct,Gimbal_Vision_Time[NOW],yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct,Gimbal_Vision_Time[NOW],pitch_angle_raw);
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,yaw_angle_raw,Vision_Angle_Speed_Yaw);//对角度和速度进行二阶卡尔曼滤波融合
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,pitch_angle_raw,Vision_Angle_Speed_Pitch);
		
		Auto_KF_Delay++;//滤波延迟开启
		
		//目标距离过近时减小预测                              这里没写!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		yaw_speed_k = debug_y_sk; //yaw速度预测比例
		kf_yaw_angcon = debug_kf_y_angcon; //yaw预测比例限幅
		kf_speed_yl = debug_kf_speed_yl; //速度过低关闭预测
		
		
	  if(fabs(Auto_Error_Yaw[NOW])<debug_auto_err_y && fabs(yaw_kf_result[KF_SPEED]) > kf_speed_yl && fabs(pitch_kf_result[KF_SPEED]) < debug_kf_speed_yh)//预测开启条件
	  {
			if(yaw_kf_result[KF_SPEED] >= 0)
	  		debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1;
			
			else if(yaw_kf_result[KF_SPEED] <0)
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1;
			
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp,-debug_kf_y_angcon,debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp,debug_kf_y_angle,debug_kf_angle_ramp);//预测量缓慢变化
			debug_kf_y_angle = constrain_float(debug_kf_y_angle,-debug_kf_y_angcon,debug_kf_y_angcon);
		  Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE]+debug_kf_y_angle;
			
			if((yaw_kf_result[KF_SPEED] > 0) && (Auto_Error_Yaw[NOW] < 0.3f)) //具体正负比较还是需要debug看
			{
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
				mobpre_yaw_left_delay++;
				
				if(mobpre_yaw_left_delay > 0)//具体延时时间
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位,可以开火

				else 
					Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
			}
			
			else if((yaw_kf_result[KF_SPEED] < 0) && (Auto_Error_Yaw[NOW] > -0.3f))//具体正负比较还是需要debug看
			{
				mobpre_yaw_left_delay = 0;//向右预测开火延时重置
				mobpre_yaw_right_delay++;
				
				if(mobpre_yaw_right_delay > 0)//具体延时时间更改数字
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位,可以开火

				else 
					Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
			}
			
	  	else
		  {
				Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
	  	}
			
			Mobility_Prediction_Yaw = TRUE;
			mobpre_yaw_stop_delay = 0;
			

//				pitch_speed_k = debug_p_sk/2.f;
//		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
//        debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[1] + debug_kf_speed_pl);
//			
//    		Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	  }
		
		if((Auto_KF_Delay > debug_kf_delay) && (fabs(Auto_Error_Pitch[NOW]) > debug_auto_err_p) && (fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl) && (VisionRecvData.distance/100 < 4.f))
		{
		  if(VisionRecvData.auto_too_close == TRUE)
		  {
		  	pitch_speed_k = debug_p_sk/2.f;
		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
		  }
			
			if(pitch_kf_result[KF_SPEED]>=0)
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			
			else 
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);
			}
			
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);//Pitch限幅
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		else
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	}
	
	else  //未识别到目标
	{
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
		
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,Cloud_Angle_Measure[YAW][GYRO],0);
    pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,Cloud_Angle_Measure[PITCH][MECH],0);		

		GIMBAL_AUTO_Ctrl();	
		Auto_KF_Delay = 0;
		
		//预测延迟重置
	}
}


/*-------------------------PID总计算在这里------------------------------*/
/**
  * @brief  pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
void GIMBAL_PositionLoop(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_Yaw_Mech_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else
	{
		
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID, Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][GYRO], Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID, Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH], Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid, Cloud_Palstance_Measure[YAW][MECH], motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_gyro_pid, Cloud_Palstance_Measure[PITCH][MECH], motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO];
    given_current[PITCH][GYRO]	=	current_set[PITCH][MECH];
	}
}

extern float Chassis_Final_Output[4];


/*-------------------------------------------------------电流发送函数在这里---------------------------------------------------------------*/
void GIMBAL_CanSend(void)
{

		
	if(modeGimbal == CLOUD_MECH_MODE)
	{
		fMotorOutput[YAW]   = given_current[YAW][MECH];
		fMotorOutput[PITCH] = given_current[PITCH][MECH];
	}
		//陀螺仪模式云台电流变化
	else
	{
		fMotorOutput[YAW]   = given_current[YAW][GYRO];
		fMotorOutput[PITCH] = given_current[PITCH][GYRO];
	}
	
	//fMotorOutput[YAW] = 0;
	//fMotorOutput[PITCH] = 5000;
	
		
	
}





/*--------------------------辅助函数------------------------------------------------------------*/
/*        临界值结构体初始化    获取陀螺仪角度，角速度    目标速度计算函数   YAW轴偏离中心角度    限制云台与底盘分离角度  函数          */
/**
  * @brief 临界值结构体初始化
  * @param  critical:临界值结构体指针
  *    get:当前读取到的角度（陀螺仪角或机械角度）
  * @retval void 
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	
	critical->AngleSum = get;//0;
	critical->CurAngle = get;
	critical->LastAngle = get;
	
	Cloud_Angle_Target[YAW][GYRO] = get; 

}


float speed_threshold = 5.f;//速度过快
float debug_speed;//左正右负,一般都在1左右,debug看
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度
		
		
//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//速度斜坡变化
//		}                                                                                           //DEBUG用
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//速度斜坡变化
//		}
		
		
		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
		S->processed_speed = 0;//时间过长则认为速度不变

	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}

/**
  * @brief  计算YAW偏离中心角度,底盘跟随模式用
  * @param  void
  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
  */
int16_t GIMBAL_GetOffsetAngle(void)
{
	return FALSE;
}


/*----------------------自瞄，预测yaw轴的辅助函数------------------------------------------------------------------------------*/
/**
  * @brief  自瞄yaw轴预测是否已经开启
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
	return FALSE;
}

/**
  * @brief  yaw轴开启预测的时候云台是否到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 左右各有延迟，换向时记得清零反向和静止时的延迟
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
	return FALSE;
}




/*------------------------哨兵的自瞄和预测辅助函数-----------------------------------------------------------------*/
/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
bool GIMBAL_AUTO_PITCH_SB(void)
{
	return FALSE;
}


/**
  * @brief  是否在中等距离自瞄哨兵,加大预测
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
	return FALSE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*----------------更新角度的机械角度和陀螺仪角度的函数------------------------------------------------------------*/

//计算相对云台中值的角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd = ecd - 4096;
    if (relative_ecd > Half_ecd_range)
        relative_ecd += ecd_range;
    else if (relative_ecd < -Half_ecd_range)
        relative_ecd -= ecd_range;

    return relative_ecd * Motor_Ecd_to_Rad;
}

/**
  * @brief  更新云台机械角度,角速度,电流值,can1中断中调用
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_UpdateAngle( char ID, int16_t angle )
{
	if (ID == PITCH)
	{
		angleMotorPit = angle;
		Cloud_Angle_Measure[PITCH][MECH]  = motor_ecd_to_angle_change(angleMotorPit);
		PitchAngle = angleMotorPit;
	}
	else if (ID == YAW)
	{
		angleMotorYaw = angle;
		Cloud_Angle_Measure[YAW][MECH]  = motor_ecd_to_angle_change(angleMotorYaw);
		YawAngle = angleMotorYaw;
	}
}

void GIMBAL_UpdateSpeed( char ID, int16_t speed )
{
	if (ID == PITCH)
	{
		speedMotorPit = speed;
		Cloud_Speed_Measure[PITCH][MECH]  = speedMotorPit;
	}
	else if (ID == YAW)
	{
		speedMotorYaw = speed;
		Cloud_Speed_Measure[YAW][MECH]  = speedMotorYaw;
	}
}

void GIMBAL_UpdateCurrent( char ID, int16_t current )
{
	if (ID == PITCH)
	{
		currentMotorPit = current;
		Cloud_Current_Measure[PITCH][MECH]  = currentMotorPit;
	}
	else if (ID == YAW)
	{
		currentMotorYaw = current;
		Cloud_Current_Measure[YAW][MECH]  = currentMotorYaw;
	}
}


void Gimbal_Error_Read(void)
{
	Error[YAW][MECH]  = Cloud_Angle_Target[YAW][MECH] -Cloud_Angle_Measure[YAW][MECH];
	Error[YAW][GYRO]  = Cloud_Angle_Target[YAW][GYRO] -Cloud_Angle_Measure[YAW][GYRO];
	Error[PITCH][MECH]= Cloud_Angle_Target[PITCH][MECH] -Cloud_Angle_Measure[PITCH][MECH];
	Error[PITCH][GYRO]= Cloud_Angle_Target[PITCH][GYRO] -Cloud_Angle_Measure[PITCH][GYRO];
}


static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;

}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
        return 0.0f;
		
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
        return;
		
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
  * @brief  更新云台姿态,500HZ,loop中调用
  * @param  void
  * @retval void
  * @attention 角度适度放大
  */
float AngleMpuYaw[2];
float AngleMpuPitch[2];
float AngleMpuRoll[2];

void GIMBAL_MPU_Update(void)
{
	//读取陀螺仪  角度   角速度   
	mpu_dmp_get_data( &angleMpuPitch, &angleMpuRoll, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	AngleMpuYaw[NOW]   = angleMpuYaw-AngleMpuYaw[LAST];
	AngleMpuPitch[NOW] = angleMpuPitch-AngleMpuPitch[LAST];
	AngleMpuRoll[NOW]  = angleMpuRoll-AngleMpuRoll[LAST];
	
			//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  (AngleMpuPitch[NOW]*PI)/180;
	  Cloud_Angle_Measure[YAW][GYRO] = (AngleMpuYaw[NOW]*PI)/180 ;
	  //theta_format(Cloud_Angle_Measure[YAW][GYRO]);
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = ((palstanceMpuPitch + PALST_COMPS_PITCH)*PI)/180;
		Cloud_Palstance_Measure[YAW][MECH]   = ((palstanceMpuYaw+PALST_COMPS_YAW)*PI)/180;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH)/10;   //直接读陀螺仪角速度
		Cloud_Palstance_Measure[YAW][GYRO]   = (palstanceMpuYaw+PALST_COMPS_YAW)/10;  //经过计算得出的角速度
}


void MPU_Update_last(void)
{
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	AngleMpuYaw[LAST] = angleMpuYaw;
	AngleMpuPitch[LAST] = angleMpuPitch;
	AngleMpuRoll[LAST] = angleMpuRoll;
}