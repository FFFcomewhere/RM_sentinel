/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "key.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "start_task.h"
#include "usart6.h"
#include "uart7.h"

#include "MPU_Temperature.h"
#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

void BSP_init(void);
/****************************************************************
 * Function:    Flash_DisableReadProtection
 * Description: Disable the read protection of user flash area.
 * Input:
 * Output:
 * Return:      1: Read Protection successfully disable
 *              2: Error: Flash read unprotection failed
*****************************************************************/

//�������Լ���
bool pass_flag=1;


int main(void)
{
			BSP_init();
    delay_ms(100);
    startTast();
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
		static portTickType ulCurrentTime = 0;
  	static portTickType ulLoopTime    = 0;
	  static int16_t  	sTimeCnt      = 0;
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ��
    delay_init(configTICK_RATE_HZ);
	  key_Init();
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();	  
    //stm32 �����¶ȴ�������ʼ��
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 �������������ʼ��
    RNG_init();
#endif
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ��
    TIM4_Init();
		TIM5_Init();
    //��������ʼ��
 //   buzzer_init(30000, 90);
    //����IO��ʼ��
    laser_configuration();
    //��ʱ��6 ��ʼ��
    TIM6_Init(60000, 90);
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
       power_ctrl_on(i);
       delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //ң������ʼ��
    remote_control_init();
		usart6_init(115200);
		uart7_init(115200);
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    MPU_Init();
		while (mpu_dmp_init( )) 
	  {
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS��ʱ */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms�����Լ� */
					if (sTimeCnt >= 2) 
					{
							pass_flag = 0;
							sTimeCnt  = 0;//10;
					}
					else
					{
							sTimeCnt++;
					}
			}
		}
		

}

//�޷�
int constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

float constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						now     += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}

void abs_limit_num(fp32 num, fp32 Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
}

