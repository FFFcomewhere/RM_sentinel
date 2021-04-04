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
#include "chassis_task.h"
#include "start_task.h"
#include "rc.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "key.h"
#include "stdio.h"
#include "arm_math.h"

#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include "Remote_Control.h"
#include "usart6.h"
#include "chassis_behaviour.h"
#include "delay.h"

#include "kalman.h"
#include "judge.h"
#include "user_lib.h"


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
		

extern  RC_ctrl_t rc_ctrl;
ChassisCtrlMode Chassis_Mode;
extKalman_t Chassis_Error_Kalman; //定义一个kalman指针
		
PidTypeDef motor_pid[4];
PidTypeDef chassis_angle_pid;

first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;
		
#define abs(x) ((x)>0? (x):(-(x)))
		
extern int16_t Sensor_data[2];

bool remote_change = FALSE;	
bool if_beat = FALSE;              //哨兵被击打
uint16_t last_remain_HP;
uint16_t now_remain_Hp;

		
extern float Revolver_Final_Output;
extern  float fMotorOutput[4] ;
		
	

//主任务
void chassis_task(void *pvParameters)
{
//    //空闲一段时间
      vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
				{
            Chassis_Init();					
				}
				else
				{
					
					if (SYSTEM_GetRemoteMode() == RC) //遥控模式
					{													
						Chassis_Set_Mode();    //切换模式  
            chassis_feedback_update();	//更新数据			  				
            Chassis_Rc_Control(); //遥控器输入   
						Chassis_Set_Contorl();  //不同模式不同处理	  					
					}				
					else
					{
						sensor_update();    //更新传感器信息
						chassis_feedback_update();
						Chassis_AUTO_Ctrl();
						Chassis_Set_Contorl();  //不同模式不同处理
					}
				}
				Chassis_Omni_Move_Calculate();  //底盘全向运动分析
				Chassis_Motor_Speed_PID(); //PID计算				
				
				//右云台处理
				
				CAN_CMD_Transfer(Sensor_data[LEFT], Sensor_data[RIGHT],fMotorOutput[PITCH] , fMotorOutput[YAW] ,Revolver_Final_Output);
				
				vTaskDelay(TIME_STAMP_2MS);
			}
}

/*----------------------------------------------***定义底盘全向移动变量***-------------------------------------------------*/

float Chassis_Move_X;//前后

fp32 motor_chassis_speed[4];     //定义四个轮子的速度，用于更新   m/s
/*----------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------***灵敏度***-------------------------------------------------------------*/

//机械模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Mech_Chassis_Standard; //定义机械模式平移遥控器响应

/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***速度限幅***-------------------------------------------------------------*/


fp32 vx_max_speed;  //前进方向最大速度 单位m/s
fp32 vx_min_speed;  //前进方向最小速度 单位m/s
fp32 vy_max_speed;  //左右方向最大速度 单位m/s
fp32 vy_min_speed;  //左右方向最小速度 单位m/s
fp32 vz_max_speed;  //左右方向最大速度 单位m/s
fp32 vz_min_speed;  //左右方向最小速度 单位m/s

float Chassis_Standard_Move_Max;//底盘前后左右平移限速
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化


/*-----------------------------------------------***斜坡参数***-------------------------------------------------------------*/

float Slope_Chassis_Move_Z;//斜坡计算出的移动变量,这是目标输出量的实时斜坡值

uint16_t timeInc;//斜坡增加变化时间
uint16_t timeInc_Saltation;//前后方向突变下的斜坡增加量,比正常情况下要小

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//键盘模式下扭头斜坡,主要用在扭屁股模式中
float Slope_Chassis_Revolve_Move;

/*-----------------------------------------------***光电开关***--------------------------------------------------------*/
#define Change_TurnBack_Time   100       //识别到的时间，大于这个时间才算识别到        
fp32 CJ_L;
fp32 CJ_R;
bool flag=TRUE;
Flag change;
/*-----------------------------------------------***定义PID参数***--------------------------------------------------------------*/

//底盘期望速度
float Chassis_Speed_Target[4];//ID

//底盘速度误差
float Chassis_Speed_Error[4];//ID

//底盘期望角度
float Chassis_Angle_Target[4];//ID

//底盘角度误差
float Chassis_Angle_Error[4];//ID

//底盘测量角度
float Chassis_Angle_Measure[4];

//底盘测量速度
int16_t Chassis_Speed_Measure[4];

//底盘测量电流值
int16_t Chassis_Current_Measure[4];

//传感器信号
int16_t Sensor_data[2];


fp32 Speed_Measure[2];
fp32 current_measure[2];
//底盘速度误差和
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//底盘角度误差和
float Chassis_Angle_Error_Sum[4];//ID
float Chassis_Angle_Error_NOW[4], Chassis_Angle_Error_LAST[4];

//单级PID参数
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	  pidTermChassis[4];//ID,计算输出量

//底盘电机输出量
float Chassis_Final_Output[4];
/*----------------------------------------------------------------------------------------------------------------------*/






//底盘电机输出量
float Chassis_Final_Output[4];

/**
  * @brief  底盘初始化  主要是PID 限幅初始化
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Init(void)
{
   	flag = TRUE;
	  remote_change = TRUE;
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    uint8_t i;
		
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {

        PID_Init(&motor_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
			
    }		
			
	 //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
		
		vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
		vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
		
		
    /**************PID参数*************************/	
	  Chassis_Speed_kpid[FRON][KP] = 11;
	  Chassis_Speed_kpid[FRON][KI] = 0.08;//0.08;
	  Chassis_Speed_kpid[FRON][KD] = 0;
	
	  Chassis_Speed_kpid[BACK][KP] = 18;
	  Chassis_Speed_kpid[BACK][KI] = 0.08;//0.08;
	  Chassis_Speed_kpid[BACK][KD] = 0;
		
    chassis_feedback_update();
    change.TO_left = FALSE;
    change.TO_right = FALSE;	
    change.left = FALSE;	
    change.right = FALSE;		
}




/**
  * @brief  更新底盘电机数据   速度  与云台的叠加角度
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void chassis_feedback_update(void)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
      motor_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN*Chassis_Speed_Measure[i];  //转换成m/s的转速			
    }	
}


/**
  * @brief  遥控器切换模式，并修改相应变量
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Mode(void)
{
	if(IF_RC_SW2_DOWN)
	{
    Chassis_AUTO_Ctrl();
		flow_led_on(3);   
		flow_led_off(2);
	}
	else
	{
		Chassis_Mode=CHASSIS_MECH_MODE;  //其他机械模式	
		flow_led_on(2);
		flow_led_off(3);
	}
}



/*********************************************************************************************************/

/**
  * @brief  遥控器控制，得出底盘全向运动速度
  * @param  void
  * @retval void
  * @attention  Chassis_Move_Z Chassis_Move_X  Chassis_Move_Y
  *              
  */
	
void Chassis_Rc_Control(void)
{
    //遥控器原始通道值
    int16_t vx_channel;
    fp32 vx_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	  if(Chassis_Mode == CHASSIS_MECH_MODE)
		{
      rc_deadline_limit(rc_ctrl.rc.ch[2], vx_channel, CHASSIS_RC_DEADLINE);
			vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	  }
		else if(Chassis_Mode == CHASSIS_R_MODE)
		{
      vx_set_channel = AUTO_MOVE_SPEED;	
		}
		else if(Chassis_Mode == CHASSIS_L_MODE)
		{
			vx_set_channel = -AUTO_MOVE_SPEED;	
		}
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
	
    //停止信号，不需要缓慢减速速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
       chassis_cmd_slow_set_vx.out = 0.0f;
    }
    Chassis_Move_X = chassis_cmd_slow_set_vx.out;
}




/*-------------------------------------***光电开关判断(0判断到了1没判断到)***----------------------------------------*/

void sensor_update(void)
{	

	//右云台处理
	CJ_L=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10); //D
	CJ_R=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11); //C
	
	
	Sensor_data[LEFT] = CJ_L;
	Sensor_data[RIGHT] = CJ_R;
	
	


	static uint16_t change_time = 0;
	


	if(remote_change == TRUE) //从机械模式切过来
	{
		remote_change = FALSE;
		CHASSIS_REST();
		Chassis_Mode = CHASSIS_L_MODE;
	}
	if(CJ_L==1 && CJ_R==0 && Chassis_Mode==CHASSIS_R_MODE)  //左边的没有识别到，右边的识别到了，且正在往右动，就往左边动
	{
		change_time++;
		if(change_time > Change_TurnBack_Time)
		{
			change_time = 0;
			change.TO_left = TRUE;
			change.TO_right = FALSE;
			flag = TRUE;
		}
	}
	else if(CJ_L==0 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)  //左边的识别到，右边的没有识别到，且正在往左动，就往右边动
	{
		change_time++;
		if(change_time > Change_TurnBack_Time) 
		{
		  change_time = 0;
		  change.TO_right = TRUE;
			change.TO_left = FALSE;
			flag = TRUE;
		}
	}
	else if(((CJ_R==0 && CJ_L==0 )|| if_beat)&& Chassis_Mode==CHASSIS_R_MODE && flag == TRUE)  //如果受到击打，或者左右都识别到障碍物1s以上，就按反方向动 
	{
		if(if_beat)
			if_beat = 0;
		
		change_time++;		
		if(change_time > 500)
		{
		  change_time = 0;
		  change.TO_left = TRUE;
			change.TO_right = FALSE;
			flag = FALSE;
		}
	}
	else if(((CJ_R==0 && CJ_L==0 )|| if_beat) && Chassis_Mode==CHASSIS_L_MODE && flag == TRUE)
	{
		if(if_beat)
			if_beat = 0;
		
		change_time++;		
		if(change_time > 500)
		{
		  change_time = 0;
		  change.TO_right = TRUE;
			change.TO_left = FALSE;
			flag = FALSE;
		}
	}
}

/**
  * @brief  哨兵自主
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X
  *              
  */
void Chassis_AUTO_Ctrl(void)
{
	sensor_update();
  if(change.TO_left == TRUE)  //左边的没有识别到，右边的识别到了，且正在往右动，就往左边动
	  Chassis_Mode = CHASSIS_L_MODE;  //往左
		
	else if (change.TO_right == TRUE)	//左边的识别到，右边的没有识别到，且正在往左动，就往右边动		
	  Chassis_Mode = CHASSIS_R_MODE;  //往右
		
	else if (CJ_L==1 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)  //初始化的时候
	{
		Chassis_Mode = CHASSIS_L_MODE;
		flag = TRUE;
	}
	
	
	
//	else if(CJ_L==1 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)
//	{
//		Chassis_Mode = CHASSIS_R_MODE;
//		flag = TRUE;
//	}
}



/**
  * @brief  不同模式不同处理方式
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Contorl(void)
{
	static fp32 change_time = 0;
	if(Chassis_Mode == CHASSIS_MECH_MODE)			
	{    
		remote_change = TRUE;
    Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);		
	}	
	else if(Chassis_Mode == CHASSIS_R_MODE)
	{		
	  change.left = FALSE;
		if(change.right == FALSE)
		{
			Chassis_Move_X = fp32_constrain( 0, -vx_max_speed, vx_max_speed);//前后
			change_time++;
			if(change_time > 300)
			{      
				change_time = 0;
				change.right = TRUE;			
			}
	  }
		else if(change.right == TRUE)
		{
		  Chassis_Move_X = fp32_constrain( AUTO_MOVE_SPEED, -vx_max_speed, vx_max_speed);//前后
		}
	}
	else if(Chassis_Mode == CHASSIS_L_MODE)
	{
		change.right = FALSE;
		if(change.left == FALSE)
		{
			Chassis_Move_X = fp32_constrain( 0, -vx_max_speed, vx_max_speed);//前后
			change_time++;
			if(change_time > 300)
			{      
				change_time = 0;
				change.left = TRUE;
			}
	  }
		else if(change.left == TRUE)
		{
		  Chassis_Move_X = fp32_constrain( -AUTO_MOVE_SPEED, -vx_max_speed, vx_max_speed);//前后
		}
	}
}
/**
  * @brief  底盘全向算法,计算各电机转速
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  *              	X前(+)后(-)     Y左(-)右(+)     Z扭头
  */
void Chassis_Omni_Move_Calculate(void)
{	
		Chassis_Speed_Target[FRON] = Chassis_Move_X;
		Chassis_Speed_Target[BACK] = -Chassis_Move_X;
}



/**
  * @brief  底盘电机PID计算输出
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Speed_PID(void) 
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;
		
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        temp = fabs(Chassis_Speed_Target[i]);
        if (max_vector < temp)
        {
             max_vector = temp;
        }
    }	

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            Chassis_Speed_Target[i] *= vector_rate;
        }
    }

    //计算pid

    for (i = 0; i < 4; i++)
    {
        PID_Calc(&motor_pid[i],motor_chassis_speed[i], Chassis_Speed_Target[i]);
    }

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        Chassis_Final_Output[i] = motor_pid[i].out;
    }		
		
}




/**
  * @brief  获取电机角度
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle )
{
    Chassis_Angle_Measure[Wheel] = angle;
}


/**
  * @brief  获取电机转速
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed )
{
	Chassis_Speed_Measure[ Wheel ] = speed;
	Speed_Measure[ Wheel ] = speed;
}


/**
  * @brief  获取电机转矩电流
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current )
{
	Chassis_Current_Measure[ Wheel ] = current;
	current_measure[ Wheel ] = current;
}

/**
  * @brief  获取光电传感器信号
  * @param  ID,CAN数据
  * @retval void
  * @attention  1为未感应到 ， 2为感应到
  */
void Sensor_UpdateMotorCur(SensorDirection direction, int16_t data)
{
	Sensor_data[direction] = data;
}





/**
  * @brief  初始化电机参数
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X Chassis_Move_Y Chassis_Move_Z 归0
  */
void CHASSIS_REST(void)
{
	Chassis_Move_X = 0;
}
