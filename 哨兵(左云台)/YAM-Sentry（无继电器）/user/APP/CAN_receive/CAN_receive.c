/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
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

#include "CAN_Receive.h"
#include "chassis_task.h"
#include "stm32f4xx.h"
#include "rng.h"
#include "can.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "Revolver_task.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "Start_Task.h"
//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4];

static CanTxMsg GIMBAL_TxMessage;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

		
extern int16_t Sensor_data[2];
float Revolver_Final_Output_right;
float Pitch_right;
float Yaw_right;

/**
  * @brief  CAN1接收中断
  * @param  void
  * @retval void
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	volatile float speed_measure_L,rota_measure_L,current_measure_L,speed_measure_R,rota_measure_R,current_measure_R;
	volatile float Cap_Inputvot,Cap_Capvot,Cap_Test_current,Cap_Target_Power;	

   // CAN_Receive(CAN1, 0, &RxMessage);
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
	    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	   CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	}
	
	//云台电机机械角度读取
	if(RxMessage.StdId == CAN_PIT_MOTOR_IDR)//云台pitch轴电机
	{
		rota_measure_L  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(PITCH, rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateSpeed(PITCH, speed_measure_L);	
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateCurrent(PITCH, current_measure_L);	
	}
	
	if(RxMessage.StdId == CAN_YAW_MOTOR_IDR)//云台yaw轴电机
	{
		rota_measure_L  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(YAW, rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateSpeed(YAW, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateCurrent(YAW, current_measure_L);
	}
	
	if(RxMessage.StdId  == CAN_2006_M1_ID)//拨盘电机
	{
		rota_measure_L = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		Revolver_UpdateMotorAngle(rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		Revolver_UpdateMotorSpeed(speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		Revolver_UpdateMotorCurrent(current_measure_L);
	}

   
	
	
	//底盘电机转速读取,机械角度暂时没用
	if(RxMessage.StdId == CAN_3508_M1_ID)//前
	{
		rota_measure_L   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(FRON, rota_measure_L);
		
		speed_measure_L  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(FRON, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(FRON, current_measure_L);
	}
	
	if(RxMessage.StdId == CAN_3508_M2_ID)//后
	{
        rota_measure_L   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(BACK, rota_measure_L);
		
		speed_measure_L  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(BACK, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(BACK, current_measure_L);
	}
	
    //摩擦轮电机 M3508
	if(RxMessage.StdId == CAN_3508_M3_ID)//左摩擦轮
	{
		rota_measure_L   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(FRIC_LEFT, rota_measure_L);
		
		speed_measure_L  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(FRIC_LEFT, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(FRIC_LEFT, current_measure_L);
	}
	
	if(RxMessage.StdId == CAN_3508_M4_ID)//右摩擦轮
	{
        rota_measure_L   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(FRIC_RIGHT, rota_measure_L);
		
		speed_measure_L  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(FRIC_RIGHT, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(FRIC_RIGHT, current_measure_L);
	}
	
}


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_slove(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif


//can2中断
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	  volatile float speed_measure,rota_measure,current_measure;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
    }
		
	
	
}


//Chassis   0X2FF   4个3508   编号 1 2 3 4      //CAN1
//Gimbal    0x1FF   2个6020   编号 5 6    //CAN1
//Revolver  0X200   1个2006   编号 5      //CAN1            2020.11.24修改



//发送云台控制命令，其中rev为保留字节
void CAN_CMD_GIMBAL(int16_t pitchl, int16_t yawl, int16_t pitchr, int16_t yawr)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (pitchl >> 8);
    GIMBAL_TxMessage.Data[1] = pitchl;
    GIMBAL_TxMessage.Data[2] = (yawl >> 8);
    GIMBAL_TxMessage.Data[3] = yawl;
    GIMBAL_TxMessage.Data[4] = (pitchr >> 8);
    GIMBAL_TxMessage.Data[5] = pitchr;
    GIMBAL_TxMessage.Data[6] = (yawr>>8);
    GIMBAL_TxMessage.Data[7] = yawr;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif

}


void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}
//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

void CAN_CMD_Revolver(int16_t motor1, int16_t motor2 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_SHOOT_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = 0;
	TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(Revolver_CAN, &TxMessage);
}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

