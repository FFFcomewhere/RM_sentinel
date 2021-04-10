/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
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
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357


extern void chassis_task(void *pvParameters);


//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 5000.0f//15000
#define M3505_MOTOR_SPEED_PID_KI 0.0f  //10
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#define CHASSIS_ACCEL_X_NUM 0.1666666667f    //һ�׵�ͨ�˲�����
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f



//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
 



//Ѳ���µ��ٶȲ���
#define AUTO_PARA 1.3

#define CHASSIS_SPEED_LOW 0
#define CHASSIS_SPEED_NORMAL 1
#define CHASSIS_SPEED_HIGH 2

  
//���̵������ٶ�
#define MAX_WHEEL_SPEED 1.6f*2  

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1.6f*AUTO_PARA //1.5

//����ģʽ�˶����̵�ǰ���ٶ�
#define AUTO_MOVE_SPEED 1.7f*AUTO_PARA




//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f

#define CHASSIS_RC_DEADLINE 10




//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002



/*--------------------myself-------------*/
#define Omni_Speed_Max 9000  //�ٶ��޷�

#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //���������

#define LIMIT_CHASSIS_MAX         9000     //������������µ��̵������������

//���̵����޷�
#define iTermChassis_Max          3000     //�����޷�




typedef enum
{
	
	CHASSIS_MECH_MODE = 0,//��е
	CHASSIS_STOP_MODE = 1,//�ڱ�ģʽ
	CHASSIS_R_MODE = 2,//�ڱ������˶�
	CHASSIS_L_MODE = 3,//�ڱ������˶�
	
} ChassisCtrlMode;


//���̵��ID
typedef enum
{
	
	FRON = 0,  // ǰ
	BACK = 1,  // ��
	
} ChassisWheel;

//������
typedef enum
{
	
	LEFT = 0,  // ��
  RIGHT = 1,  // ��
	
} SensorDirection;



typedef struct
{
	bool TO_left;
	bool TO_right;
	bool left;
	bool right;
  bool stop;
}Flag;


void sensor_update(void);
void chassis_feedback_update(void);
void Chassis_Motor_Angle_PID( ChassisWheel Wheel );
void Chassis_MotorOutput(void);
void CHASSIS_REST(void);
void Chassis_Omni_Move_Calculate(void);
void Chassis_Angle_Move_Calculate(void);
void Chassis_Rc_Control(void);
void CHASSIS_CANSend(void);
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed );
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current );
void Chassis_Init(void);
void Chassis_AUTO_Ctrl(void);
void Chassis_Set_Mode(void);
void Chassis_Set_Contorl(void);
void Chassis_Motor_Speed_PID(void);
#endif
