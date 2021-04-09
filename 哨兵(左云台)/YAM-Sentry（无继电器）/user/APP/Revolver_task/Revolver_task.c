#include "main.h"
#include "Revolver_task.h"
#include "start_task.h"
#include "friction.h"
#include "fric.h"
#include "friction.h"
#include "vision.h"
#include "led.h"
#include "laser.h"
#include "stdio.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include "Remote_Control.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
extern  RC_ctrl_t rc_ctrl;
extern VisionRecvData_t VisionRecvData;
#define abs(x) ((x)>0? (x):(-(x)))

/******拨盘,控制逻辑与云台类似*********/

//拨盘电机模式,位置环与速度环
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//拨盘模式选择
//拨盘电机模式,位置环与速度环


/*********测试**********/
#define Revolver_Angle_Mid  4092
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_STOP_MODE = 2,
}eRevolverCtrlMode;
eRevolverCtrlMode Revolver_mode;

/*****************高射频必须降低射速，拨弹必须和摩擦轮配合***********************/


#define REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速
#define Revolver_Speed    1620
/*       拨盘型号预编译       */
#define SEVEN_REVOLVER    0    	//7格拨盘
#define EIGHT_REVOLVER    1		//8格拨盘
#define TEN_REVOLVER      2		//10格拨盘
#define TWELVE_REVOLVER   3		//12格拨盘

#define REVOLVER_CHOOSE  EIGHT_REVOLVER	//选择拨盘型号

#if REVOLVER_CHOOSE == EIGHT_REVOLVER			
	#define		REVOL_SPEED_GRID      8				//拨盘格数
	#define 	AN_BULLET         (36864.0f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == SEVEN_REVOLVER
	#define 	REVOL_SPEED_GRID      7				//拨盘格数
	#define   	AN_BULLET     	  (42130.2857f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == TEN_REVOLVER
	#define 	REVOL_SPEED_GRID  	  10			//拨盘格数
	#define    	AN_BULLET     	  (29491.2f)		//单个子弹电机位置增加值

#elif REVOLVER_CHOOSE == TWELVE_REVOLVER
	#define 	REVOL_SPEED_GRID      12			//拨盘格数
	#define    	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值

#endif	



/*------------------------------------------------***位置PID***---------------------------------------------------------*/
float Revolver_Angle_Measure_Sum;//拨盘测量角度累计和,用于位置PID
int16_t Revolver_Angle_Measure_Prev;//上次剩下的累加和角度,用于圈数计算判断
//底盘期望角度
float  Revolver_Angle_Target_Sum;
float  Revolver_Buff_Target_Sum;//打符模式下的目标值暂存，此时目标角度用斜坡处理
float  Revolver_Buff_Ramp = AN_BULLET/40;//40ms转一格,一定不能超过50ms


/********射击**********/
//发射子弹数,按一下加一颗,发一颗减一次
int16_t Key_ShootNum;//鼠标射击计数

//累计和
float Revolver_Angle_Measure_Sum;//拨盘测量角度累计和,用于位置PID
int16_t Revolver_Angle_Measure_Prev;//上次剩下的累加和角度,用于圈数计算判断

//拨盘电机输出量,正数逆时针
float Revolver_Final_Output;

//拨盘角度误差
float Revolver_Angle_Error[2];//  inner/outer



float pTermRevolSpeed, iTermRevolSpeed;	
float pTermRevolAngle[2], iTermRevolAngle[2],pidTermRevolAngle[2];//  inner/outer
float Revolver_Speed_kpid[3];//	kp/ki/kd
float Revolver_Angle_kpid[2][3];//  inner/outer    kp/ki/kd
float kRc_Chassis_Standard; //定义机械模式平移遥控器响应


/************卡弹************/
#define Stuck_Revol_PIDTerm   500      //PID输出大于这个数则认为有可能卡弹
#define Stuck_Speed_Low       60       //测量速度低于这个数,则认为有可能卡弹

#define Stuck_SpeedPID_Time   100       //速度连续 ms过小,PID连续  ms过大
#define Stuck_TurnBack_Time   500       //倒转时间,时间越长倒得越多
uint32_t Stuck_Speed_Sum = 0;//计算卡弹次数,速度环
uint32_t Stuck_Posit_Sum = 0;//计算卡弹次数,位置环

portTickType posishoot_time;//射击延时测试


/**********限幅*************/
//最终输出限幅
float Revolver_Output_Max;
float Revolve_Move_Max;
float Revolve_Posi_Max;
float kRc_Mech_Standard; //定义机械模式平移遥控器响应
float kRc_Mech_Revolve;  //定义机械模式旋转遥控器响应
float iTermRevolPosiMax;//位置环积分限幅
float iTermRevolSpeedMax;//速度环积分限幅


/*******************拨盘参数**********************/

//拨盘测量角度
float Revolver_Angle_Measure;

//拨盘测量速度
float Revolver_Speed_Measure;

//拨盘测量电流
float Revolver_Current_Measure;

//拨盘速度误差
float Revolver_Speed_Error;

//拨盘目标转速
float  Revolver_Speed_Target;//转速过低容易卡弹,尽量让转速上6000




//拨盘速度环射频
int16_t Revolver_Freq;

#define  Revolver_Speed_Low 2.5
#define  Revolver_Speed_Mid 3   //5
#define  Revolver_Speed_High 10

/*********************************************摩擦轮*********************************************************/

#define    REVOL_CAN_OPEN    340  //摩擦轮实际速度超过这个值才允许拨盘转动,根据摩擦轮最小目标速度来改变


uint8_t revol_remot_change = TRUE;
void Revolver_task(void *pvParameters)
{
//    //空闲一段时间
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
				{
				   REVOLVER_Rest();
           Revolver_Init();	
					 friction_Init();
					 
				}
				else
				{
					if (SYSTEM_GetRemoteMode() == RC) //遥控模式
					{										
						Revolver_RC_Ctrl();					
						friction_RC_Ctrl();
					}
					else
					{
						Revolver_AUTO_Ctrl();
						friction_AUTO_Ctrl();
						revol_remot_change = TRUE;
					}
				}
				
				if (Revolver_mode==REVOL_SPEED_MODE)//速度环
				{
					REVOL_SpeedLoop();
					Fric_mode(FRI_LOW);
				}

				else if(Revolver_mode==REVOL_POSI_MODE)//位置环
				{
					REVOL_PositionLoop();
					Fric_mode(FRI_LOW);
				}
				
				else if(Revolver_mode==REVOL_STOP_MODE)
				{
				  REVOL_SpeedLoop();
					Fric_mode(FRI_OFF);
				}
				
				
				
//				if(Fric_GetSpeedReal() > REVOL_CAN_OPEN)//摩擦轮开启
//		    {
//					laser_on();
//					Revolver_CANSend();
//				}
//				else
//				{
//					Revolver_Speed_Target = 0;//摩擦轮关闭,拨盘不给动
//					Revolver_Angle_Rest();//摩擦轮关闭，屏蔽这期间的打弹指令
//					REVOL_SpeedLoop();
//					laser_off();
//					Revolver_CANSend();
//				}
				
				vTaskDelay(TIME_STAMP_2MS);
			}
}


/**
  * @brief  拨盘重启
  * @param  void
  * @retval void
  * @attention 枪口超热量重置
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//位置环发弹清零
	Revolver_Speed_Target = 0;//速度环停止转动
	
	
	
	//速度环位置重置
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Measure;
	
	
	//PID积分清零
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

void Revolver_Init(void)
{
	/* 目标值 */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;
	Key_ShootNum = 0;
	/**************遥控器灵敏度*************************/	
  kRc_Mech_Standard = 14.f;  //调节摇杆灵敏度,调节速度
  kRc_Mech_Revolve  = 11.4f; //调节机械模式摇杆扭头灵敏度(太小会影响最高速度)
	
	
/********************************************/
/**************PID参数*************************/	
	
//角度	
	Revolver_Angle_kpid[OUTER][KP] = 0.08;//0.005
	Revolver_Angle_kpid[OUTER][KI] = 0;
	Revolver_Angle_kpid[OUTER][KD] = 0;
	Revolver_Angle_kpid[INNER][KP] = 6;//1.8
	Revolver_Angle_kpid[INNER][KI] = 0;
	Revolver_Angle_kpid[INNER][KD] = 0;
	
		/* PID参数 */
	  //速度环
	Revolver_Speed_kpid[KP] = 14;
	Revolver_Speed_kpid[KI] = 0.0f;
	Revolver_Speed_kpid[KD] = 0;
	

	/* 位置环目标角度 */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;//不能置0,否则上电会反转
	Revolver_Buff_Target_Sum  = Revolver_Angle_Measure;	
	
	
		
/********************************************/
/**************限幅参数*************************/
  Revolve_Move_Max = 10000;
	iTermRevolSpeedMax  = 250;
	iTermRevolPosiMax   = 2500;
	Revolver_Output_Max = 9999;  
	
/********************************************/
}


/**
  * @brief  拨盘角度清零
  * @param  void
  * @retval void
  * @attention 模式切换时用,防止下次切回去会突然动一下
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Target_Sum;
}



/**
  * @brief  拨盘的遥控模式
  * @param  void
  * @retval void
  * @attention  遥控用速度环
  *              
  */


void Revolver_RC_Ctrl(void)
{	
	
	/*******************点射版**********************/
	
	if(IF_RC_SW2_UP)//机械模式下单发，方便测试弹道
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环
		if(REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++;//打一颗
		}
		
		if(revol_remot_change == TRUE)//刚从键盘模式切换过来，清空发弹数据
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest();//防止突然从键盘切到遥控狂转
		}
		
		if(Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Buff_Target_Sum += AN_BULLET;
		}
		
		if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//缓慢转过去
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
		}
		
		if(IF_RC_SW1_UP)
		{
     Reset_Fric();
		}
		else
		{
     Fric_mode(FRI_MID);
		}
		
		REVOL_PositStuck();//卡弹判断及倒转
	}
	/**************连发版*******************/
	else if(IF_RC_SW2_MID)//自动模式
	{
		Revolver_mode = REVOL_SPEED_MODE;//速度环
		
		if(IF_RC_SW1_DOWN)//sw1下打弹
		{
			Revolver_Freq = Revolver_Speed_Mid;//射频选择
			//速度环转速设置
			Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
		}
		else	//遥控模式关闭拨盘
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
		}
		if(IF_RC_SW1_UP)  //SW1上关闭摩擦轮
		{
     Reset_Fric();
		}
		else
		{
    Fric_mode(FRI_MID);
		}
			
		REVOL_SpeedStuck();//卡弹判断及倒转
	}
	/********************************************/
}


/**
  * @brief  stop Fric
  * @param  void
  * @retval void
  * @attention 
  */

extern uint16_t Fric_Speed_Level;
extern float Friction_Speed_Target;
extern float Friction_Speed_Real;	

void Reset_Fric(void)
{
	 Fric_Speed_Level = FRI_OFF;
	 Friction_Speed_Target = 0;
	 Friction_Speed_Real   = 0;
}


/*****************************************摇摆测试***************************************/

//long key,key0;
//void Revolver_RC_Ctrl(void)
//{	

//  if(IF_RC_SW1_DOWN)
//	{
//		Revolver_mode=REVOL_POSI_MODE;		
//		key0=key%2;		
//		if(revol_remot_change == TRUE)//刚从键盘模式切换过来，清空发弹数据
//		{
//			revol_remot_change = FALSE;
//			Revolver_Angle_Rest();//防止突然从键盘切到遥控狂转
//		}		
//    if(key0 == 0)
//		{
//			Revolver_Buff_Target_Sum = -120000;
//		}
//		else
//		{
//			Revolver_Buff_Target_Sum = 120000;
//		}	
//		if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//缓慢转过去
//		{
//			Revolver_Angle_Target_Sum = RAMP_float  (Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, 500);
//		}	
//		if(abs(Revolver_Buff_Target_Sum - Revolver_Angle_Target_Sum)< 200)
//		{
//			key=key+1;
//		}	
//	}
//}

/*****************************************************************************************/

void Revolver_AUTO_Ctrl(void)
{
	

	//如果识别到装甲板且识别到靶心 拨盘转动	
	if (Vision_If_Update() && VisionRecvData.centre_lock)
	{
		Revolver_mode = REVOL_SPEED_MODE;  //速度
		Revolver_Speed_Target = constrain_float(Revolver_Speed, -Revolve_Move_Max, Revolve_Move_Max);
		REVOL_SpeedStuck();//卡弹判断及倒转
		Vision_Clean_Update_Flag();
	}
	
}




/**
  * @brief  速度环PID控制
  * @param  void
  * @retval void
  * @attention  遥控只有速度环
  */
void REVOL_SpeedLoop(void)
{  

	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//典型单级PID算法
	pTermRevolSpeed   = Revolver_Speed_Error * Revolver_Speed_kpid[KP];
	iTermRevolSpeed  += Revolver_Speed_Error * Revolver_Speed_kpid[KI];
	iTermRevolSpeed   = constrain( iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax );

	Revolver_Final_Output = constrain_float( pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max );
//	
	
	
	//fzj
	
//	//获取转过的总角度值
//	REVOL_UpdateMotorAngleSum( );
//	
//	//外环计算
//	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
//	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];

//	//内环计算
//	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
//	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];		
//	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
//	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );
//  
//	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);
}


/**
  * @brief  位置环PID控制
  * @param  void
  * @retval void
  * @attention  键盘模式
  */
void REVOL_PositionLoop(void)
{
	//获取转过的总角度值
	REVOL_UpdateMotorAngleSum();
	
	//外环计算
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];
  
		//内环计算
	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];		
	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );

	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);

}



void REVOL_UpdateMotorAngleSum(void)
{		 
	//临界值判断法
	if (abs(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//过半圈且过零点
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;

		else
			//超过了一圈
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
	}
	else      
		//未过临界值,累加上转过的角度差
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;

	//记录此时电机角度,下一次计算转过角度差用,用来判断是否转过1圈
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}




///**
//  * @brief  拨盘遥控打弹
//  * @param  void
//  * @retval void
//  * @attention 
//  */
//#define REVOL_STEP0    0		//失能标志
//#define REVOL_STEP1    1		//SW1复位标志
//#define REVOL_STEP2    2		//弹仓开关标志
//uint8_t	Revolver_Switch = 0;//弹仓遥控模式开关标志位转换
//bool REVOLVER_Rc_Switch(void)
//{
//	if (IF_RC_SW2_UP)//机械模式
//	{
//		if (IF_RC_SW1_UP)
//		{
//			if (Revolver_Switch == REVOL_STEP1)
//				Revolver_Switch = REVOL_STEP2;

//			else if (Revolver_Switch == REVOL_STEP2)
//				Revolver_Switch = REVOL_STEP0;
//		}
//		else		
//			Revolver_Switch = REVOL_STEP1;
//	}
//	else
//		Revolver_Switch = REVOL_STEP0;
//	
//	if (Revolver_Switch == REVOL_STEP2)
//		return TRUE;
//	else
//		return FALSE;
//}

/**
  * @brief  拨盘遥控打弹
  * @param  void
  * @retval void
  * @attention 
  */
#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志
uint8_t	Revolver_Switch = 0;//弹仓遥控模式开关标志位转换
bool REVOLVER_Rc_Switch(void)
{
	
	if (IF_RC_SW2_UP )
	{
		if (IF_RC_SW1_DOWN)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else		
		{
			Revolver_Switch = REVOL_STEP1;
		}
	}
	else
	{
		Revolver_Switch = REVOL_STEP0;
	}
	
	if (Revolver_Switch == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*****************************卡弹处理**************************************/

/**
  * @brief  速度环式卡弹处理
  * @param  void
  * @retval void
  * @attention  卡住就反转n格
  */
void REVOL_SpeedStuck(void)
{
	static uint16_t  stuck_time    = 0;//卡弹计时
	static uint16_t  turnback_time = 0;//倒转计时
	static bool Revol_Speed_ifStuck = FALSE;//卡弹判断

	if (Revol_Speed_ifStuck == TRUE)//已确认卡弹,开始倒转计时
	{
		Revolver_Speed_Target = -4000;//倒转
		turnback_time++;//倒转一定时间

		if (turnback_time > Stuck_TurnBack_Time)//倒转完成
		{
			turnback_time  = 0;
			Revol_Speed_ifStuck = FALSE;//可以正转
		}			
	}
	else
	{
		if ( abs(Revolver_Final_Output) >= Stuck_Revol_PIDTerm //PID输出过大
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//速度过低
			stuck_time++;//卡弹计时

		else
			stuck_time = 0;//没有长时间卡弹,及时清零

		if (stuck_time > Stuck_SpeedPID_Time)//卡了超过60ms
		{
			Stuck_Speed_Sum++;//卡弹计数,给机械组的大兄弟点面子
			stuck_time = 0;
			Revol_Speed_ifStuck = TRUE;//标记可以进入倒转计时
		}
	}
}


/**
  * @brief  位置环式卡弹处理
  * @param  void
  * @retval void
  * @attention  卡住就反转n格
  */
void REVOL_PositStuck(void)
{
	static uint16_t  stuck_time      = 0;//卡弹计时
	static uint16_t  turnback_time   = 0;//倒转计时
	static bool Revol_Posit_ifStuck = FALSE;//卡弹判断
	
	if (Revol_Posit_ifStuck == TRUE)//卡弹后开始倒转计时
	{
		//卡弹了则在判断是否卡弹这段时间内鼠标按下的指令都清零
		Key_ShootNum = 0;
		
		turnback_time++;//倒转计时,1ms一次
		if (turnback_time > Stuck_TurnBack_Time)//倒转时间够了
		{
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum;//正常旋转,旋转回本来想要它转到的位置
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			Revol_Posit_ifStuck = FALSE;//认为此时不再卡弹了
			turnback_time = 0;//倒转时间清零,为下次倒转做准备	
		}
	}
	else
	{
		if ( abs(Revolver_Final_Output)  >= Stuck_Revol_PIDTerm //PID输出过大
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//速度过低
			stuck_time++;//统计卡了多长时间

		else
			stuck_time = 0;//不卡了,时间清零
		
		if (stuck_time > Stuck_SpeedPID_Time)//卡太久了,提示要倒转
		{
			//倒转不能放在Revol_Posit_ifStuck == TRUE中,否则就不是堵一次倒转1/2格了
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum - AN_BULLET ;//倒转 1格  
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			
			Stuck_Posit_Sum++;//卡弹计数,给机械组的大兄弟点面子
			stuck_time = 0;
			Revol_Posit_ifStuck = TRUE;//用来标记倒转计时开启	
		}
		
	}
}


/**
  * @brief  获取电机角度
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void Revolver_UpdateMotorAngle( int16_t angle )
{
  	Revolver_Angle_Measure = angle;
}

/**
  * @brief  获取电机转速
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void Revolver_UpdateMotorSpeed( int16_t speed )
{
	Revolver_Speed_Measure = speed;
}


/**
  * @brief  获取电机转矩电流
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void Revolver_UpdateMotorCurrent( int16_t current )
{
	Revolver_Current_Measure = current;
}



/**
  * @brief  发送拨盘电流值
  * @param  void
  * @retval void
  * @attention 
  */
void Revolver_CANSend(void)
{	 	
	CAN_CMD_Revolver(Revolver_Final_Output, 0);
}

