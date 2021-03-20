#ifndef _PWM_H
#define _PWM_H

#include "stm32f4xx.h"
#include "main.h"


#define PWM1  TIM4->CCR3     //Ä¦²ÁÂÖ,PD14
#define PWM2  TIM4->CCR4     //Ä¦²ÁÂÖ,PD15
#define PWM3  TIM5->CCR1     //Ä¦²ÁÂÖ,PH10
#define PWM4  TIM5->CCR2     //Ä¦²ÁÂÖ,PD11
#define PWM5  TIM4->CCR1     //Ä¦²ÁÂÖ,PD14
#define PWM6  TIM4->CCR2     //Ä¦²ÁÂÖ,PD15
#define PWM7  TIM5->CCR3     //Ä¦²ÁÂÖ,PH10
#define PWM8  TIM5->CCR4     //Ä¦²ÁÂÖ,PD11


void TIM4_Init(void);
void TIM5_Init(void);
void TIM8_Init(void);
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);
void TIM5_FrictionPwmOutp(int16_t pwm3,int16_t pwm4);
void TIM8_FrictionPwmOutp(int16_t pwm5,int16_t pwm6,int16_t pwm7,int16_t pwm8);
#endif
