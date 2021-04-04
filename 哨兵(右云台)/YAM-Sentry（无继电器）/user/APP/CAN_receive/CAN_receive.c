/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
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
//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//��̨������ݶ�ȡ
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


//�����������
static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4];

static CanTxMsg GIMBAL_TxMessage;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

/**
  * @brief  CAN1�����ж�
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
	
	//��̨�����е�Ƕȶ�ȡ
	if(RxMessage.StdId == CAN_PIT_MOTOR_IDR)//��̨pitch����
	{
		rota_measure_L  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(PITCH, rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateSpeed(PITCH, speed_measure_L);	
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateCurrent(PITCH, current_measure_L);	
	}
	
	if(RxMessage.StdId == CAN_YAW_MOTOR_IDR)//��̨yaw����
	{
		rota_measure_L  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(YAW, rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateSpeed(YAW, speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateCurrent(YAW, current_measure_L);
	}
	
	
	if(RxMessage.StdId  == CAN_2006_M2_ID)//���̵��
	{
		rota_measure_L = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		Revolver_UpdateMotorAngle(rota_measure_L);
		
		speed_measure_L = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		Revolver_UpdateMotorSpeed(speed_measure_L);
		
		current_measure_L = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		Revolver_UpdateMotorCurrent(current_measure_L);
	}
	
}


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_slove(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif

//can2�ж�
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


void CAN_CMD_Transfer(int16_t sensor_left,int16_t senser_right, int16_t Pitch_right, int16_t Yaw_right, int16_t Revolver_Final_Output_right)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_SIGNAL_TRANSFER_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = sensor_left;
    TxMessage.Data[1] = senser_right;
   //TxMessage.Data[2] = 0;
	  //TxMessage.Data[3] =0;

	  TxMessage.Data[2] = Pitch_right >> 8;
	  TxMessage.Data[3] = Pitch_right;
//	
	
	
    TxMessage.Data[4] = Yaw_right >> 8;
    TxMessage.Data[5] = Yaw_right;
	
	
    TxMessage.Data[6] = Revolver_Final_Output_right >> 8;
    TxMessage.Data[7] = Revolver_Final_Output_right;

    CAN_Transmit(TRANSFER_CAN, &TxMessage);
}



//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

