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

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define CAP_CAN CAN1
#define Revolver_CAN CAN1
#define TRANSFER_CAN CAN1
#define Send_Mode_CAN CAN1
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,

    CAN_REVOLVER_ALL_ID = 0x200,
		CAN_2006_M2_ID = 0x204, 

		CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_PIT_MOTOR_IDR= 0x207,   //IDΪ3
		CAN_YAW_MOTOR_IDR= 0x208,   //IDΪ4
		
    CAN_SIGNAL_TRANSFER_ALL_ID = 0X217,  //����̨��������ֵ����̨
    CAN_SEND_MODE_ID = 0X250    //��������̨���͵�ģʽ����
		
	  

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{	
    uint16_t ecd;   //����ֵ
    int16_t speed_rpm;    //ת��
    int16_t given_current;  //����ֵ
    uint8_t temperate;
    int16_t last_ecd;  //��һ�α���ֵ
} motor_measure_t;



//���ʹ���������̨����ź�
void CAN_CMD_Transfer(int16_t sensor_left,int16_t senser_right, int16_t Pitch_right, int16_t Yaw_right, int16_t Revolver_Final_Output_right);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
