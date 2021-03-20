#include "Friction.h"
#include "Pwm.h"
#include "remote_control.h"
#include "start_task.h"

#define High_Friction_Speed    800
#define Mid_Friction_Speed     600
#define Low_Friction_Speed     400
#define Stop_Friction_Speed    0


extern  RC_ctrl_t rc_ctrl;

extern eRemoteMode remoteMode;

bool FRICTION_FLAG = TRUE;


//摩擦轮目标速度
float Speed_Mode;
float Friction_Speed_Target;

//摩擦轮等级目标转速
float Frict_Speed_Level_Target;

//摩擦轮实际输出速度,用来做斜坡输出
float Friction_Speed_Real;

float kRc_Friction = 0.7 ;


void FRICTION_Ctrl( void )
{

	Friction_Rc_Ctrl();
	
	Friction_Ramp();
	
	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
	TIM5_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
	TIM8_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real,Friction_Speed_Real,Friction_Speed_Real);
	
}







void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//开启
	{
		Friction_Speed_Real += 5;
		
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//关闭
	{
		Friction_Speed_Real -= 5;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
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
