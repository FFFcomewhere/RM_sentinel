#include "fric.h"
#include "friction.h"
#include "main.h"
#include "Remote_Control.h"
#include "rc.h"
#include "delay.h"
#include "judge.h"
#include "stdbool.h"
#include "chassis_task.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h" 
#include "task.h"

extern  RC_ctrl_t rc_ctrl;
extern float Chassis_Speed_Target[4];//ID

/**************************摩擦轮控制***********************************/
float Friction_PWM_Output[3]     = {0, 2000, 3000};//关闭  低速 200 哨兵  500

//摩擦轮不同pwm下对应的热量增加值(射速),最好比实际值高5
uint16_t Friction_PWM_HeatInc[5] = {0,  20,  26,  34,  36};//测试时随便定的速度,后面测试更改


//速度等级选择
uint16_t Fric_Speed_Level;

//遥控模式下的开启标志位
uint8_t Friction_Switch = 0;//与弹仓开关判断类似

//摩擦轮目标速度
float Friction_Speed_Target;

//摩擦轮等级目标转速
float Frict_Speed_Level_Target;

//摩擦轮实际输出速度,用来做斜坡输出
float Friction_Speed_Real;


/*********************************************************************************************************/


/**
  * @brief  摩擦轮失控保护
  * @param  void
  * @retval void
  * @attention 缓慢减小PWM输出,否则会报警
  */
void FRICTION_StopMotor(void)
{
	Friction_Speed_Target = 0;

	if (Friction_Speed_Real > 0)
	{
		Friction_Speed_Real -= 1;//3
	}

	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}


	Chassis_Speed_Target[FRIC_LEFT] = 0;
	Chassis_Speed_Target[FRIC_RIGHT] = 0;


}

/************************摩擦轮总控制*****************************/

/**
  * @brief  摩擦轮控制函数
  * @param  void
  * @retval void
  * @attention 摩擦轮停止转动,红外关闭,摩擦轮从关闭到开启,云台抬头归中
  */

void friction_Init(void)
{
	 Fric_Speed_Level = FRI_OFF;
	 Friction_Speed_Target = 0;
	 Friction_Speed_Real   = 0;
}





/**
  * @brief  摩擦轮遥控控制函数
  * @param  void
  * @retval void
  * @attention 
  */
void friction_RC_Ctrl(void)
{
	Fric_Speed_Level = FRI_SENTRY;//遥控模式下的速度选择，低射速，方便检录发光弹
	
	if (FRIC_RcSwitch() == TRUE)//判断状态切换
	{	//切换为关
	 if (Friction_Speed_Target > Friction_PWM_Output[FRI_OFF])
	 {
		 Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];	
	 }
	 else//切换为开
	 {
		 Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];//摩擦轮目标值大于0,标志着遥控模式下开启,告诉pitch要抬头
	 }
  }
	
	else
  {
	 if(Friction_Speed_Target > Friction_PWM_Output[Fric_Speed_Level])
	 {
		 Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];
 	 }
  }
			
	//摩擦轮输出斜坡,注意要先抬头才能进入斜坡
	Chassis_Speed_Target[FRIC_LEFT] = -Friction_Speed_Target;
	Chassis_Speed_Target[FRIC_RIGHT] = Friction_Speed_Target;

}


void Fric_mode(uint16_t speed)
{
	
if (FRIC_RcSwitch() == TRUE)//判断状态切换
{
	if(speed == FRI_LOW)
	{
	  Friction_Speed_Target = Friction_PWM_Output[FRI_LOW];
	}
	
	else if(speed == FRI_SENTRY)
	{
		Friction_Speed_Target = Friction_PWM_Output[FRI_SENTRY];
	}
}
	if(speed == FRI_OFF)
	{
		Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];
	}
	
	 
	//摩擦轮输出斜坡,注意要先抬头才能进入斜坡
	Chassis_Speed_Target[FRIC_LEFT] = -Friction_Speed_Target;
	Chassis_Speed_Target[FRIC_RIGHT] = Friction_Speed_Target;
}

extern  uint8_t game_begin ; //用于防止开始前裁判杀死哨兵导致摩擦轮失效
/**
  * @brief  摩擦轮哨兵模式控制函数
  * @param  void
  * @retval void
  * @attention 
  */
void friction_AUTO_Ctrl(void)
{
	if(game_begin == FALSE && Vision_If_Update()==TRUE) 	
	{
		friction_Init();
	  game_begin = TRUE;
	}
	else if(game_begin == TRUE)
	{
		//初始状态为低速,识别到目标摩擦轮转速设置为高速
	
		Friction_Speed_Target = Friction_PWM_Output[FRI_LOW];
		if (Vision_If_Update())
		Friction_Speed_Target = Friction_PWM_Output[FRI_SENTRY];
	}

	//摩擦轮输出斜坡,注意要先抬头才能进入斜坡
	Chassis_Speed_Target[FRIC_LEFT] = -Friction_Speed_Target;
	Chassis_Speed_Target[FRIC_RIGHT] = Friction_Speed_Target;
}



/***********摩擦轮启动云台抬头判断函数*************/

/**
  * @brief  摩擦轮是否已经开启
  * @param  void
  * @retval TRUE已开启   FALSE未开启
  * @attention 
  */
uint8_t FRIC_IfWait( void )
{
    if (Friction_Speed_Target > 0)//通过改变摩擦轮目标值来标志遥控模式下pitch是否要抬头
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  摩擦轮是否正在开启
  * @param  void
  * @retval TRUE正在开启   FALSE开启到目标速度
  * @attention 正在开启不准改变云台PITCH
  */
uint8_t FRIC_IfOpen( void )
{
	static  uint8_t  status = FALSE;
	static uint32_t  ulWait = 0;

	static portTickType ulCurrent = 0;

	ulCurrent = xTaskGetTickCount( );
	
    if (Friction_Speed_Real > 0)
	{
		if (ulCurrent >= ulWait + 1500)//抬头时长,1.5S
		{
			status = TRUE;
		}
	}
	else
	{
		ulWait = ulCurrent;
		
		status = FALSE;
	}
	
	return status;
}

/**
  * @brief  摩擦轮遥控控制
  * @param  void
  * @retval 是否转换当前状态
  * @attention 跟弹仓开关逻辑相同
  */
bool FRIC_RcSwitch(void)
{

		if (IF_RC_SW1_DOWN) //开启摩擦轮条件1
		{
			if (Friction_Switch == FRIC_STEP1)
			{
				Friction_Switch = FRIC_STEP2;
			}
			else if (Friction_Switch == FRIC_STEP2)
			{
				Friction_Switch = FRIC_STEP0; //切断联系
			}
		}
		else //标志SW1是否有复位的情况,在复位的情况下才能再次进入STERP2
		{
			Friction_Switch = FRIC_STEP1; //保障SW1在下次变换之前一直不能用
		}
	

	if (Friction_Switch == FRIC_STEP2)
	{
		return TRUE; //只有SW1重新变换的时候才为TRUE
	}
	else
	{
		return FALSE;
	}
}


/*************摩擦轮辅助函数****************/

/**
  * @brief  摩擦轮输出斜坡
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//开启
	{
		Friction_Speed_Real += 2;
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//关闭
	{
		Friction_Speed_Real -= 2;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
//	Friction_Speed_Real = 0;
}

/**
  * @brief  获取当前摩擦轮PWM输出值
  * @param  void
  * @retval 实际PWM值
  * @attention 用来禁止摩擦轮速度过低的情况下拨盘的转动
  */
float Fric_GetSpeedReal(void)
{
	return Friction_Speed_Real;
}


void if_death_of_judgy()
{
	Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];	
	Friction_Ramp();
	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
}
