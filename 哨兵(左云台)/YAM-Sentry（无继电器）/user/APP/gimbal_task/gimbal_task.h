/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"


//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201

#define EEE   99999999999     //�������뾯���õ�

#define PALST_COMPS_YAW        (-7)     //������YAW���ٶȲ���
#define PALST_COMPS_PITCH      (41)      //������PITCH���ٶȲ���

//#define max_relative_angle 8191
//#define min_relative_angle -8191


// TEST �Լ�����  REAL װ�����
//PID
#define TEST  1
#define REAL  -1
#define MODE   TEST

#if MODE ==	TEST
/*----------------------------------------�ٶȻ�--------------------------------------------*/
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP 8000.0f
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 8.5f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP 40.0f  //32
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 3.0f
#define YAW_SPEED_PID_MAX_OUT 10000.0f
#define YAW_SPEED_PID_MAX_IOUT 1000.0f

//pitch �ٶȻ� PID�����Լ� PID���������������
//#define PITCH_GYRO_SPEED_PID_KP_ 2000.0f   //5500 ����pid�л������е��¼�����ֵ�仯�����Ը��ĳ�ʼPID
#define PITCH_GYRO_SPEED_PID_KP 3000.0f   //5500
#define PITCH_GYRO_SPEED_PID_KI 0.0f       //1.8
#define PITCH_GYRO_SPEED_PID_KD 0.0f      //0.9

#define PITCH_GYRO_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_GYRO_SPEED_PID_MAX_IOUT 5000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_GYRO_SPEED_PID_KP 80.0f  //32
#define YAW_GYRO_SPEED_PID_KI 0.5f
#define YAW_GYRO_SPEED_PID_KD 4.0f

#define YAW_GYRO_SPEED_PID_MAX_OUT 10000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT 1000.0f

/*----------------------------------------�ǶȻ�---------------------------	-----------------*/
////**������ģʽ**//
//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP_ 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_KP 10.0f //15
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.01f
#define PITCH_GYRO_ABSOLUTE_PID_KD 1.5f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 30.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP 600.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.1f
#define YAW_GYRO_ABSOLUTE_PID_KD 1.2f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 800.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//**��еģʽ**//
//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f    //15
#define PITCH_ENCODE_RELATIVE_PID_KI 0.2f
#define PITCH_ENCODE_RELATIVE_PID_KD 5.5f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 50.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 10.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 5000.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f

#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 6000.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//ע��˴������װ��maxʵ�ʶ�Ӧ��Сֵ������pitch_max ʵ��Ϊ��������޷�

#define max_yaw_relative_angle      4*PI
#define min_yaw_relative_angle     	-4*PI
#define max_pitch_relative_angle     -2.1
#define min_pitch_relative_angle     -2.65

#define auto_yaw_ccw                2.2
#define auto_yaw_cw                 -0.1
#define auto_pitch_up               -2.1 
#define auto_pitch_down              -2.66


#define mid_yaw_angle               1.05                   
#define mid_pitch_angle             -2.1

#define glancing_angle              -PI




#endif

//��ͷ��̨�ٶ�
#define TurnSpeed 0.04f

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 10


//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN -0.000006f //0.005



//��̨����������ʱ��ʹ�õı���
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f


//��̨��������
#define GIMBAL_CONTROL_TIME 1


//�������ֵ����Լ���ֵ
#define Half_ecd_range 4096
#define ecd_range 8191


//�������ֵת���ɽǶ�ֵ
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;




void GIMBAL_task(void *pvParameters);
/*----------------------------------myself---------------------------------------*/

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);   //��̨PID��ʼ�������ڽǶȻ�
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);//��̨PID���㣬���ڽǶȻ�
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);//��̨PID����


void RC_Set_Mode(void);  //ң����ѡ�����ģʽ

void GIMBAL_Set_Control(void); //ң����ģʽ������̨��Ŀ��ֵ
void GIMBAL_Set_Key_Control(void); //����ģʽ������̨��Ŀ��ֵ

void GIMBAL_Behaviour_Control_Set(fp32 add_yaw_mech, fp32 add_yaw_gyro, fp32 add_pitch_mech , fp32 add_pitch_gyro);//��̨���ݲ�ͬ״̬���ò�ͬ�ĺ���
void gimbal_absolute_angle_control(fp32 yaw, fp32 pitch);//��̨������ģʽ�Ŀ��ƺ�������Ҫд��180���ͷ

#define PITCH 0
#define YAW 1


#define MECH 0
#define GYRO 1
#define ROLL 2


#define NOW  0
#define LAST 1

//��̨ģʽѡ��
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_CRUISE_MODE = 1,
} GimbalCtrlMode;



/* ��̨����ģʽ:
   
   ��ͨ             	NORMAL
   ��ͷ180��             AROUND
   ���             	BUFF
   ����,pitchˮƽ   	LEVEL
   ��еģʽpitcḩͷ	HIGH
   ����Ťͷ90��          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//����ģʽ,����ģʽѡ��
	GIMBAL_AROUND  = 1,//180���ͷ
	GIMBAL_TURN    = 2,//90��Ťͷ
	GIMBAL_AUTO    = 3,//����
	
}eGimbalAction;



typedef struct
{
  bool pitch_up;
	bool pitch_down;
	bool yaw_cw;
	bool yaw_ccw;
}Auto_Mode;



typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;

/*       �ٽ�ֵ����ṹ��        */
typedef struct 
{

	float LastAngle;       //��һ�ζ�ȡ�ĽǶ�
	float CurAngle;	       //��ǰ��ȡ�ĽǶ�
	float AngleSum;	       //�Ƕ��ۼ�ֵ
	
}Critical_t;

	

void GIMBAL_InitCtrl(void);
void GIMBAL_Rc_Ctrl( void ); //ң����ģʽ
void GIMBAL_AUTO_Ctrl(void); //�ڱ�Ѳ��ģʽ
void GIMBAL_AUTO_Mode_Ctrl(void);
void GIMBAL_AUTO_PREDICT_Mode_Ctrl(void);
void vPitch_Mech_PositionLoop(void);  //error_delta �����ǽ��ٶ�
void vPitch_Gyro_PositionLoop(void);
void vYaw_Gyro_PositionLoop(void);
void vYaw_Mech_PositionLoop(void);

void GIMBAL_PositionLoop(void);
void GIMBAL_kPID_Init(void);
void GIMBAL_CanSend(void);
void GIMBAL_UpdateCurrent( char ID, int16_t current );
void GIMBAL_UpdateSpeed( char ID, int16_t speed );
void GIMBAL_UpdateAngle( char ID, int16_t angle );
void Gimbal_Error_Read(void); //��ȡ���ֵ
void GIMBAL_MPU_Update(void);
void MPU_Update_last(void);

void Critical_Handle_Init(Critical_t *critical, float get);
float Gimbal_Yaw_Gryo_AngleSum(Critical_t *critical, float get);
void Gimbal_Chass_Separ_Limit(void);
int16_t GIMBAL_GetOffsetAngle(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);

void GIMBAL_NORMAL_Mode_Ctrl(void);//��̨������Ӧģʽ
void GIMBAL_LEVEL_Mode_Ctrl(void);//����ģʽ
void GIMBAL_AUTO_Mode_Ctrl(void);//����ģʽ
void	GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);//��̨���������ͷ����̨
void GIMBAL_BASE_Mode_Ctrl(void);///��ͷ����ģʽ
	

bool GIMBAL_IfAuto_MobPre_Yaw(void);//����yaw��Ԥ���Ƿ��Ѿ�����
bool GIMBAL_MOBPRE_YAW_FIRE(void);//yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ
bool GIMBAL_BUFF_YAW_READY(void);//���YAW���Ƿ��ƶ���λ
bool GIMBAL_BUFF_PITCH_READY(void);//���PITCH�Ƿ��ƶ���λ
bool GIMBAL_AUTO_PITCH_SB(void);//�Ƿ��������ڱ�
bool GIMBAL_AUTO_PITCH_SB_SK(void);//�Ƿ����еȾ��������ڱ�

#endif
