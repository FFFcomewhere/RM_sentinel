#include "fric.h"


/**
  * @brief  摩擦轮配置（初始化+解锁）
  * @param  void
  * @retval void
  * @attention PWM1\2直接定义为CCR寄存器,PD14->CH3,PD15->CH4
  */
void TIM4_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4    , ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&gpio);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_TIM4);    
	
	
	
	tim.TIM_Prescaler = 90-1;//1MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;		
	tim.TIM_Period =2500-1;      //  1MHz/2500Hz  400Hz
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM4,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;    		
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;		
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC3Init(TIM4,&oc);		
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM4,&oc);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	
	PWM1 = 1000;		//解锁摩擦轮，大于1000
	PWM2 = 1000;    //
}




void TIM8_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOI,&gpio);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_TIM8);	
	
	
	
	tim.TIM_Prescaler = 90-1;//1MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;		
	tim.TIM_Period =2500-1;      //  1MHz/2500Hz  400Hz
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM8,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;    		
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;		
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC1Init(TIM8,&oc);		
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM8,&oc);		
	TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM8,&oc);		
	TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM8,&oc);
	TIM_OC4PreloadConfig(TIM8,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM8,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
	
	PWM5 = 1000;		//解锁摩擦轮，大于1000
	PWM6 = 1000;    //
	PWM7 = 1000;		//解锁摩擦轮，大于1000
	PWM8 = 1000;    //
}


/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正，以后接线的时候注意三条线的接法
  */
void TIM8_FrictionPwmOutp(int16_t pwm5,int16_t pwm6,int16_t pwm7,int16_t pwm8)//8,9
{ 
	
	
	PWM5 = pwm5+1000;	//持续输出时要大于1ms
	PWM6 = pwm6+1000;
	PWM7 = pwm7+1000;	//持续输出时要大于1ms
	PWM8 = pwm8+1000;
}

/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正，以后接线的时候要注意三条线的接法
  */
void TIM4_FrictionPwmOutp(int16_t pwm1, int16_t pwm2)//8,9
{ 
	
	
	PWM1 = pwm1+1000;	//持续输出时要大于1ms
	PWM2 = pwm2+1000;
	
}

void TIM5_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&gpio);
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource13, GPIO_AF_TIM5);    
	
	
	
	tim.TIM_Prescaler = 90-1;//1MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;		
	tim.TIM_Period =2500-1;      //  1MHz/2500Hz  400Hz
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM5,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;    		
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;		
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC1Init(TIM5,&oc);		
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM5,&oc);
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM5,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
	TIM_CtrlPWMOutputs(TIM5,ENABLE);
	
	
	PWM3 = 1000;		//解锁摩擦轮，大于1000
	PWM4 = 1000;    //
}




/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正，以后接线要注意三条线的接法
  */
void TIM5_FrictionPwmOutp(int16_t pwm3,int16_t pwm4)//8,9
{
	PWM3 = pwm3+1000;	//持续输出时要大于1ms
	PWM4 = pwm4+1000;
}


