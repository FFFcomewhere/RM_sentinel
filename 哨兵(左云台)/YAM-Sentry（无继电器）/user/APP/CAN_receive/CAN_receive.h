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
#define SENSOR_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
		      
    CAN_REVOLVER_ALL_ID = 0x200,
		CAN_2006_M1_ID = 0x203,
	
		CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_PIT_MOTOR_IDR= 0x205,   //IDΪ1
		CAN_YAW_MOTOR_IDR= 0x206,   //IDΪ2 
		
		 
    CAN_SIGNAL_TRANSFER_ALL_ID = 0X300,
 
		
	  CAN_CAP_ID=0x211,

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

//void GIMBAL_MPU_Update(void);

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t pitchr, int16_t pitchl, int16_t yawr, int16_t yawl);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//���Ͳ��̵����������
void CAN_CMD_Revolver(int16_t motor1,int16_t motor2);
//���ʹ������ź�
void CAN_CMD_Sensor(int16_t left,int16_t right);

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
