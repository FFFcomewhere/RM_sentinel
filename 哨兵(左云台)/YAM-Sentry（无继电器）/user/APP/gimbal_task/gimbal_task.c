/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       AV  DV���ҵ�ͷ ���ֿ��� ��̨����ģʽ δ����   �������˲�����Ҫ�˽⣬���飬������ƻ���Ҫ�˽⡣
  * @history    ���ӳ��ڳ���   ���������볬��   �����Ҹ�    ������ڳ���  R�� �����Ҹ�    ���������Ҹ�       �����װ�Ļ�  ������   ����������
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "gimbal_task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "main.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "chassis_task.h"
#include "stdio.h"


#define gimbal_total_pid_clear(void)               \
    {                                              \
        Gimbal_PID_clear(&Gimbal_Yaw_Mech_PID);    \
				PID_clear(&gimbal_yaw_motor_mech_pid);     \
			  Gimbal_PID_clear(&Gimbal_Pitch_Mech_PID);  \
			  PID_clear(&gimbal_pitch_motor_mech_pid);   \
    }

		
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }	
		
/*---------------------------------------------------�޷�-------------------------------------------------------------------*/

		
/*--------------------------------------myself-------------------------------*/
extern  RC_ctrl_t rc_ctrl;    //����ң�����ṹ�����
GimbalCtrlMode  modeGimbal;   //������̨����ģʽ    ��е/������
eGimbalAction  actGimbal;     //������̨�˶�ģʽ  ��ͷ ���� �����
Critical_t Yaw_Gyro_Angle;    

extern VisionRecvData_t VisionRecvData; //�����Ӿ����յ����ݽṹ��

Gimbal_PID_t Gimbal_Yaw_Mech_PID;      //PIDһϵ�нṹ��
Gimbal_PID_t Gimbal_Yaw_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_Mech_PID;
Gimbal_PID_t Gimbal_Pitch_Gyro_PID;
PidTypeDef gimbal_yaw_motor_gyro_pid;
PidTypeDef gimbal_pitch_motor_gyro_pid;
PidTypeDef gimbal_yaw_motor_mech_pid;
PidTypeDef gimbal_pitch_motor_mech_pid;
/*-----------------------------------------------------PID����----------------------------------------------------------------------------*/

//�����ǲ���
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//�����ǽǶ�ֵ
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//�����ǽ��ٶ�ֵ
float angleMpu[3][2];

//��е�Ƕ��м����,��CAN�ж�ȡ����
int16_t  angleMotorPit,  angleMotorYaw; 
int16_t  speedMotorPit,  speedMotorYaw; 
int16_t  currentMotorPit,  currentMotorYaw; 

float PitchAngle, YawAngle;
//�����Ƕ�
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro
extern float Cloud_Angle_Target_GD[2][2];   //  pitch/yaw    mech/gyro  ������key_control.c��

//�����Ƕ�
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

//�������ת��
float Cloud_Speed_Measure[2][2];//  pitch/yaw    mech/gyro

//�����������ֵ
float Cloud_Current_Measure[2][2];//  pitch/yaw    mech/gyro

//�������ٶ�
float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro


float motor_gyro_set[2][2];  //PID�����⻷��������ٶ��趨ֵ  pitch/yaw    mech/gyro
float motor_gyro_set[2][2]; 

float current_set[2][2];      //PID�����ڻ���������  pitch/yaw    mech/gyro
float current_set[2][2];

float given_current[2][2];     //PID���ո�ֵ����  pitch/yaw    mech/gyro
float given_current[2][2];
float fMotorOutput[4] = {0};

float Error[2][2]; //���ֵ��ȡ

//����j-scpoe���ԣ�j-scpoe��֧�ֶ�����
#define jscpoe_filter  1000

int32_t yaw_mech_measure;
int32_t yaw_gyro_measure;
int32_t pitch_mech_measure;
int32_t pitch_gyro_measure;	
int32_t yaw_mech_target;
int32_t yaw_gyro_target;
int32_t pitch_mech_target;
int32_t pitch_gyro_target;	


static void J_scope_gimbal_test(void)
{
	yaw_mech_measure = (int32_t)(Cloud_Angle_Measure[YAW][MECH] * jscpoe_filter);
	yaw_gyro_measure = (int32_t)(Cloud_Angle_Measure[YAW][GYRO] * jscpoe_filter);
	pitch_mech_measure = (int32_t)(Cloud_Angle_Measure[PITCH][MECH] * jscpoe_filter);
	pitch_gyro_measure = (int32_t)(Cloud_Angle_Measure[PITCH][GYRO] * jscpoe_filter);

	yaw_mech_target = (int32_t)(Cloud_Angle_Target[YAW][MECH] * jscpoe_filter);
	yaw_gyro_target = (int32_t)(Cloud_Angle_Target[YAW][GYRO] * jscpoe_filter);
	pitch_mech_target = (int32_t)(Cloud_Angle_Target[PITCH][MECH] * jscpoe_filter);
	pitch_gyro_target = (int32_t)(Cloud_Angle_Target[PITCH][GYRO] * jscpoe_filter);

}



/*---------------------------------------------------*����*----------------------------------------------------------------------------*/
Auto_Mode auto_mode;
bool Auto_Mode_falg = 0;
uint32_t Auto_Mode_Count = 0;

float pitch_angle_raw;
float yaw_angle_raw;
float Auto_Error_Pitch[2];
float Auto_Error_Yaw[2];
float Auto_Distance;
uint32_t Gimbal_Vision_Time[2];

//�²�����Ԥ����
float Auto_Distance;//Ԥ�����
float vision_time_update_time;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ
float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
float yaw_speed_k = 0;//yaw�ٶ�Ԥ�����
float kf_yaw_angcon = 0;//yawԤ������޷�
float pitch_speed_k = 0;//pitch�ٶ�Ԥ�����
float kf_pitch_angcon = 0;//pitchԤ������޷�
float debug_kf_y_angle;//yawԤ���ݴ�
float debug_kf_p_angle;//pitchԤ���ݴ�
float debug_kf_angle_temp;//Ԥ��Ƕ�б���ݴ���
float debug_kf_angle_ramp = 20;//Ԥ��Ƕ�б�±仯��
float kf_speed_yl = 0;//�ٶȹ��͹ر�Ԥ��
uint16_t Auto_KF_Delay = 0;//����ͻȻ����,�������˲�������ʱ
float debug_y_sk;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_y_sb_sk;//�ڱ�Ԥ��ϵ��
float debug_y_sb_brig_sk;//��ͷ�ڱ�
float debug_p_sk;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_auto_err_y=120;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��              ����ֵ�ڳ����л���Ҫ�޸�               
float debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
float debug_kf_delay=80;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����              ����ֵ�ڳ����л���Ҫ�޸�
float debug_kf_speed_yl;//yaw�ٶȹ��͹ر�Ԥ��
float debug_kf_speed_yl_sb;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
float debug_kf_speed_yh;//yaw�ٶȹ��߹ر�Ԥ��
float debug_kf_speed_pl;//pitch�ٶȹ��͹ر�Ԥ��
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
float debug_kf_p_angcon;//pitchԤ�����޷�


/*�Զ����õ�һЩ��־λ*/
bool Mobility_Prediction_Yaw = FALSE;//Ԥ���Ƿ�����־λ
bool Mobi_Pre_Yaw_Fire = FALSE;//Ĭ��Ԥ��û��λ����ֹ��ǹ

uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������


/*���׿�����*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2
extKalman_t Vision_Distance_Kalman;                     //�����Ӿ����뿨�����˲��ṹ��
speed_calc_data_t Vision_Yaw_speed_Struct;              //�����Ӿ�yaw�ٶȲ��ٽṹ��
speed_calc_data_t Vision_Pitch_speed_Struct;            //�����Ӿ�pitch�ٶȲ��ٽṹ��
kalman_filter_t yaw_kalman_filter;                      //����yaw�������˲����ṹ��
kalman_filter_t pitch_kalman_filter;                    //����pitch�������˲����ṹ��

/*----------------------------------------------------------ң������ر���-------------------------------------------------------------------*/

//�ϵ�б�±���
float Slope_Begin_Pitch = 0.005 ;  //���ϵ�ʱ�ƶ�����
float Slope_Begin_Yaw = 0.005 ;

float rc_add_yaw, rc_add_pit;       //ң��������
int16_t yaw_channel, pitch_channel; //ң�����м����                                                  
extern uint8_t Vision_Get_New_Data;

bool op=0;

//ÿ2msִ��һ��������
void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		
		/* ����� */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//��ʼ��ģʽ  ����pitch�ϵ��˦�����Լ����ж�
       GIMBAL_InitCtrl();
		
		else
		{
			//����j-scope����
			J_scope_gimbal_test();

			if(SYSTEM_GetRemoteMode() == RC)
				
			{
        RC_Set_Mode();
			  GIMBAL_Set_Control();
			  actGimbal = GIMBAL_NORMAL;
			}
			if(SYSTEM_GetRemoteMode() == AUTO)
			{
				uint32_t 	Auto_Mode_Count_Past;		
				if(op==0)
				{
					modeGimbal = CLOUD_MECH_MODE;   //��ʼpidĬ�ϻ�еģʽ��ֹ��ͷ
					op=1;
				}
				if(Vision_If_Update()==TRUE )    //���ݸ���
				{		
				

						modeGimbal = CLOUD_CRUISE_MODE;
						GIMBAL_AUTO_Mode_Ctrl();
				}
				else                            //�Ӿ������ݸ���
				{
					if(Vision_If_Update()==TRUE){
						Auto_Mode_Count++;
						Auto_Mode_Count_Past = Auto_Mode_Count;
					}
					else{
						if (Auto_Mode_Count_Past > 0){
								delay_ms(300);
						}
						else
						{
						Auto_Mode_Count=0;
						Auto_Mode_Count_Past = 0;
						}
					}
					modeGimbal = CLOUD_MECH_MODE;
					GIMBAL_AUTO_Ctrl();
				}
			}

		}
		
		
		
		GIMBAL_PositionLoop();
		GIMBAL_CanSend();			
		CAN_CMD_Send_Mode(SYSTEM_GetRemoteMode());  //����ģʽ����
		
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_4MS);//������ʱ
	}
}


/**
  * @brief  ��̨��ʼ��,��Ҫ��PID��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_InitCtrl(void)
{	
		static bool bAngleRecord  = FALSE;
	  static portTickType  ulTimeCurrent = 0;
	  auto_mode.pitch_up = TRUE;
  	auto_mode.pitch_down = FALSE;
	  auto_mode.yaw_cw = TRUE;
	  auto_mode.yaw_ccw = FALSE;
	
	  if (xTaskGetTickCount( ) - ulTimeCurrent > TIME_STAMP_100MS)//��֤���ϵ�������´ο���
		   bAngleRecord = FALSE;

		 
		ulTimeCurrent = xTaskGetTickCount( );
		 
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]   = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		static const fp32 Pitch_gyro_speed_pid[3] = {PITCH_GYRO_SPEED_PID_KP, PITCH_GYRO_SPEED_PID_KI, PITCH_GYRO_SPEED_PID_KD};
    static const fp32 Yaw_gyro_speed_pid[3]   = {YAW_GYRO_SPEED_PID_KP, YAW_GYRO_SPEED_PID_KI, YAW_GYRO_SPEED_PID_KD};
		modeGimbal = CLOUD_MECH_MODE;
    //��ʼ��yaw���pid		
		GIMBAL_PID_Init(&Gimbal_Yaw_Gyro_PID,YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Yaw_Mech_PID,YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_yaw_motor_gyro_pid, PID_POSITION, Yaw_gyro_speed_pid, YAW_GYRO_SPEED_PID_MAX_OUT, YAW_GYRO_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_yaw_motor_mech_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);		
    //��ʼ��pitch���pid_
		GIMBAL_PID_Init(&Gimbal_Pitch_Gyro_PID, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_Mech_PID, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		PID_Init(&gimbal_pitch_motor_gyro_pid, PID_POSITION, Pitch_gyro_speed_pid, PITCH_GYRO_SPEED_PID_MAX_OUT, PITCH_GYRO_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_pitch_motor_mech_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);		
		
		gimbal_total_pid_clear();		

		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		
			//��¼�ϵ�ʱ��̨��е�Ƕ�
	  if (bAngleRecord == FALSE)
	  {
	  	bAngleRecord = TRUE;
			
		  Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		  Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	  }
		
			//ƽ��������̨�ƶ����м�,��ֹ���ϵ��˦
	  Cloud_Angle_Target[PITCH][MECH] = RAMP_float( mid_pitch_angle, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
	  Cloud_Angle_Target[YAW][MECH]   = RAMP_float( mid_yaw_angle, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);


}
/*-----------------------------------------------��̨ң��������ģʽѡ���ң����Ŀ��ֵ����--------------------------------------------------*/
/**
  * @brief  ��̨ң��������ģʽ
  * @param  void
  * @retval void
  * @attention 
  */
void RC_Set_Mode(void)
{
	if(IF_RC_SW2_DOWN)
		modeGimbal = CLOUD_CRUISE_MODE;
	
	else
		modeGimbal = CLOUD_MECH_MODE;
}

/**
  * @brief  ������̨��Ŀ��ֵ
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_Set_Control(void)
{
		Auto_Mode_falg = 1;
	  //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    rc_deadline_limit(RC_CH0_RLR_OFFSET, yaw_channel, RC_deadband);
    rc_deadline_limit(RC_CH1_RUD_OFFSET, pitch_channel, RC_deadband);	
	
    rc_add_yaw = yaw_channel * Yaw_RC_SEN ;
    rc_add_pit = pitch_channel * Pitch_RC_SEN ;		
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
   	
#if YAW_POSITION ==	YAW_DOWN
    if (Cloud_Angle_Target[YAW][MECH] > min_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
		
    else if (Cloud_Angle_Target[YAW][MECH] < max_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
		
#else
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
		
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
		
#endif	
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		Cloud_Angle_Target[YAW][MECH]-= rc_add_yaw;		
	
//		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
//		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
	}
	
	Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
	Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][MECH];
}



/*---------------------------------�ڱ�ģʽ��pitch������ת����yaw��360��ת----------------------------------*/
float erro_pitch = 0.0f;
float erro_yaw = 0.0f;
/*---˦ͷ---*/
int glancing = 0;//��������
float glancing_Angle_Target;


void GIMBAL_AUTO_Ctrl(void)
{
	

	Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
	
	if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
			Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
	else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
			Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;

/*---------------------yaw��----------------------*///װ�ϵ绬���Ժ�360��ת�����ڲ���

	if(auto_mode.yaw_cw == TRUE)
	{			
		if (glancing > 6){
			Cloud_Angle_Target[YAW][MECH] = RAMP_float( glancing_angle, Cloud_Angle_Target[YAW][MECH], 0.2 );
			erro_yaw = glancing_angle - Cloud_Angle_Measure[YAW][MECH];
		}else{
			Cloud_Angle_Target[YAW][MECH] = RAMP_float( auto_yaw_cw, Cloud_Angle_Target[YAW][MECH], 0.03 );
			erro_yaw = auto_yaw_cw - Cloud_Angle_Measure[YAW][MECH];
		}
		if(erro_yaw < 0.15f && erro_yaw > -0.15f)
		{
			auto_mode.yaw_cw = FALSE;
			auto_mode.yaw_ccw = TRUE;
			glancing++;//������1
			if (glancing > 7){
				glancing = 0;
			}
		}
	}
	else if(auto_mode.yaw_ccw == TRUE)
	{			
		if (glancing > 6){
			Cloud_Angle_Target[YAW][MECH] = RAMP_float( glancing_angle, Cloud_Angle_Target[YAW][MECH], 0.2 );
			erro_yaw = glancing_angle - Cloud_Angle_Measure[YAW][MECH];
		}else{
			erro_yaw = auto_yaw_ccw - Cloud_Angle_Measure[YAW][MECH];
			Cloud_Angle_Target[YAW][MECH] = RAMP_float( auto_yaw_ccw, Cloud_Angle_Target[YAW][MECH], 0.03 );
		}
		if(erro_yaw < 0.15f && erro_yaw > -0.15f)
		{
			auto_mode.yaw_cw = TRUE;
			auto_mode.yaw_ccw = FALSE;
			glancing++;//������1
			if (glancing > 7){
				glancing = 0;
			}			
		}
	}
		
	
	
/*---------------------pitch��--------------------*/	
	if(auto_mode.pitch_up == TRUE)
	{			
		Cloud_Angle_Target[PITCH][MECH] = RAMP_float( auto_pitch_up, Cloud_Angle_Target[PITCH][MECH], 0.015);
		
		//Cloud_Angle_Target[PITCH][MECH]  = auto_pitch_up;
		
		erro_pitch =  Cloud_Angle_Measure[PITCH][MECH] - auto_pitch_up;
		if(erro_pitch  < 0.05f && erro_pitch > -0.05f)
		{
			auto_mode.pitch_up = FALSE;
			auto_mode.pitch_down = TRUE;
		}
	}
	
	else if(auto_mode.pitch_down == TRUE)
	{		
    Cloud_Angle_Target[PITCH][MECH] = RAMP_float( auto_pitch_down, Cloud_Angle_Target[PITCH][MECH], 0.015);		
		
			//Cloud_Angle_Target[PITCH][MECH]  = auto_pitch_down;

		erro_pitch =   Cloud_Angle_Measure[PITCH][MECH] - auto_pitch_down;
		if(erro_pitch  < 0.05f && erro_pitch > -0.05f )
		{
			auto_mode.pitch_up = TRUE;
			auto_mode.pitch_down = FALSE;
		}
	}
	
	
}



/*---------------------------------����----------------------------------*/
/**
  * @brief  ��̨�ڱ�����ģʽ
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_AUTO_Mode_Ctrl(void)
{
	
	

		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Target[YAW][MECH];
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Target[PITCH][MECH];

	
		if(Vision_If_Update()==TRUE)
		{  
		static float pitch_angle_ref;
		static float yaw_angle_ref;
		Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
		Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
		Vision_Get_Distance(&Auto_Distance);
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][MECH]+Auto_Error_Pitch[NOW]);
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][MECH]-Auto_Error_Yaw[NOW]);
		//Vision_Clean_Update_Flag();//����,�����һֱִ��
		Auto_Error_Pitch[NOW] = 0;
		Auto_Error_Yaw[NOW]= 0 ;
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//��ȡ�����ݵ�����ʱ��
		
			
			
		if(VisionRecvData.identify_target == TRUE)                     //ʶ����Ŀ��
		{
			Cloud_Angle_Target[YAW][GYRO] = yaw_angle_ref;
			Cloud_Angle_Target[PITCH][GYRO] = pitch_angle_ref;
		}

		
		if (Cloud_Angle_Target[PITCH][GYRO] > max_pitch_relative_angle)
				Cloud_Angle_Target[PITCH][GYRO] = max_pitch_relative_angle;

		else if (Cloud_Angle_Target[PITCH][GYRO] < min_pitch_relative_angle)
				Cloud_Angle_Target[PITCH][GYRO] = min_pitch_relative_angle;

		if (Cloud_Angle_Target[YAW][GYRO] > max_yaw_relative_angle)
				Cloud_Angle_Target[YAW][GYRO] = max_yaw_relative_angle;

		else if (Cloud_Angle_Target[YAW][GYRO] < min_yaw_relative_angle)
				Cloud_Angle_Target[YAW][GYRO] = min_yaw_relative_angle;
		
		
		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
		Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
//		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Target[YAW][GYRO];
//		Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Target[PITCH][GYRO];
//		
		
	}
}
/*----------------------------����Ԥ��----------------------------------*/

void GIMBAL_AUTO_PREDICT_Mode_Ctrl(void)
{	
	static float yaw_angle_raw, pitch_angle_raw;//�������˲��ǶȲ���ֵ
	static float yaw_angle_ref;//��¼Ŀ��Ƕ�
	static float pitch_angle_ref;//��¼Ŀ��Ƕ�
	
	Mobility_Prediction_Yaw = FALSE;
	Mobi_Pre_Yaw_Fire = FALSE;
	
	//��ȡ�Ƕ�ƫ����,�Ѿ�ת��Ϊ��������
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman,Auto_Distance);
	
	
	if(Vision_If_Update() == TRUE)                    //���ݸ���
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][GYRO]+Auto_Error_Pitch[NOW]);//�õ��ĽǶ������������Ҫ�Ŵ���߼��ϲ���
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);//�õ��ĽǶ������������Ҫ�Ŵ���߼��ϲ���
		//Vision_Clean_Update_Flag();//����,�����һֱִ��
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//��ȡ�����ݵ�����ʱ��
	}
	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //���¿������˲�����ֵ
	{
		vision_time_update_time = Gimbal_Vision_Time[NOW] - Gimbal_Vision_Time[LAST];//�����Ӿ��ӳ�
		pitch_angle_raw = pitch_angle_ref;//���¶��׿������˲�����ֵ
		yaw_angle_raw = yaw_angle_ref;
		Gimbal_Vision_Time[NOW] = Gimbal_Vision_Time[LAST];
	}
	
	if(VisionRecvData.identify_target == TRUE)                     //ʶ����Ŀ��
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct,Gimbal_Vision_Time[NOW],yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct,Gimbal_Vision_Time[NOW],pitch_angle_raw);
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,yaw_angle_raw,Vision_Angle_Speed_Yaw);//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,pitch_angle_raw,Vision_Angle_Speed_Pitch);
		
		Auto_KF_Delay++;//�˲��ӳٿ���
		
		//Ŀ��������ʱ��СԤ��                              ����ûд!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		yaw_speed_k = debug_y_sk; //yaw�ٶ�Ԥ�����
		kf_yaw_angcon = debug_kf_y_angcon; //yawԤ������޷�
		kf_speed_yl = debug_kf_speed_yl; //�ٶȹ��͹ر�Ԥ��
		
		
	  if(fabs(Auto_Error_Yaw[NOW])<debug_auto_err_y && fabs(yaw_kf_result[KF_SPEED]) > kf_speed_yl && fabs(pitch_kf_result[KF_SPEED]) < debug_kf_speed_yh)//Ԥ�⿪������
	  {
			if(yaw_kf_result[KF_SPEED] >= 0)
	  		debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1;
			
			else if(yaw_kf_result[KF_SPEED] <0)
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1;
			
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp,-debug_kf_y_angcon,debug_kf_y_angcon);//Ԥ���ݴ����޷�
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp,debug_kf_y_angle,debug_kf_angle_ramp);//Ԥ���������仯
			debug_kf_y_angle = constrain_float(debug_kf_y_angle,-debug_kf_y_angcon,debug_kf_y_angcon);
		  Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE]+debug_kf_y_angle;
			
			if((yaw_kf_result[KF_SPEED] > 0) && (Auto_Error_Yaw[NOW] < 0.3f)) //���������Ƚϻ�����Ҫdebug��
			{
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_left_delay++;
				
				if(mobpre_yaw_left_delay > 0)//������ʱʱ��
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ,���Կ���

				else 
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
			}
			
			else if((yaw_kf_result[KF_SPEED] < 0) && (Auto_Error_Yaw[NOW] > -0.3f))//���������Ƚϻ�����Ҫdebug��
			{
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay++;
				
				if(mobpre_yaw_right_delay > 0)//������ʱʱ���������
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ,���Կ���

				else 
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
			}
			
	  	else
		  {
				Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
				
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
	  	}
			
			Mobility_Prediction_Yaw = TRUE;
			mobpre_yaw_stop_delay = 0;
			

//				pitch_speed_k = debug_p_sk/2.f;
//		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
//        debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[1] + debug_kf_speed_pl);
//			
//    		Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	  }
		
		if((Auto_KF_Delay > debug_kf_delay) && (fabs(Auto_Error_Pitch[NOW]) > debug_auto_err_p) && (fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl) && (VisionRecvData.distance/100 < 4.f))
		{
		  if(VisionRecvData.auto_too_close == TRUE)
		  {
		  	pitch_speed_k = debug_p_sk/2.f;
		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
		  }
			
			if(pitch_kf_result[KF_SPEED]>=0)
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			
			else 
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);
			}
			
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);//Pitch�޷�
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		else
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	}
	
	else  //δʶ��Ŀ��
	{
//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
		
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,Cloud_Angle_Measure[YAW][GYRO],0);
    pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,Cloud_Angle_Measure[PITCH][MECH],0);		

		GIMBAL_AUTO_Ctrl();	
		Auto_KF_Delay = 0;
		
		//Ԥ���ӳ�����
	}
}


/*-------------------------PID�ܼ���������------------------------------*/
/**
  * @brief  pid����
  * @param  void
  * @retval void
  * @attention �˴����ܸı�Ŀ��Ƕ�,ֻ���������޷��͵���PID���㺯��
  */
void GIMBAL_PositionLoop(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_Yaw_Mech_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
		

		
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
		
		
		
	}
	else
	{
		
	
		Cloud_Palstance_Measure[YAW][MECH] = 0;
		Cloud_Palstance_Measure[PITCH][MECH] = 0;
		
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID, Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][GYRO], Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][GYRO] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID, Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][GYRO], Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid, Cloud_Palstance_Measure[YAW][MECH], motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][GYRO] = PID_Calc(&gimbal_pitch_motor_gyro_pid, Cloud_Palstance_Measure[PITCH][MECH], motor_gyro_set[PITCH][GYRO]);	
    
		
		
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO];
    given_current[PITCH][GYRO]	=	current_set[PITCH][GYRO];
	}
}

extern float Chassis_Final_Output[4];
extern float Pitch_right;
extern float Yaw_right;


/*-------------------------------------------------------�������ͺ���������---------------------------------------------------------------*/
void GIMBAL_CanSend(void)
{

	
	//������̨�����װ����������ķ��͵��������Ӹ���
		
	if(modeGimbal == CLOUD_MECH_MODE)
	{
		fMotorOutput[YAW]   = -given_current[YAW][MECH];
		fMotorOutput[PITCH] = -given_current[PITCH][MECH];
	}
	//������ģʽ��̨�����仯
	else
	{
		fMotorOutput[YAW]   = -given_current[YAW][GYRO];
		fMotorOutput[PITCH] = -given_current[PITCH][GYRO];
	}
	
	
		
	CAN_CMD_GIMBAL(fMotorOutput[PITCH], fMotorOutput[YAW], Pitch_right , Yaw_right);
	//����
	//CAN_CMD_GIMBAL(0, 0, 0 , 0);

	
}





/*--------------------------��������------------------------------------------------------------*/
/*        �ٽ�ֵ�ṹ���ʼ��    ��ȡ�����ǽǶȣ����ٶ�    Ŀ���ٶȼ��㺯��   YAW��ƫ�����ĽǶ�    ������̨����̷���Ƕ�  ����          */
/**
  * @brief �ٽ�ֵ�ṹ���ʼ��
  * @param  critical:�ٽ�ֵ�ṹ��ָ��
  *    get:��ǰ��ȡ���ĽǶȣ������ǽǻ��е�Ƕȣ�
  * @retval void 
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	
	critical->AngleSum = get;//0;
	critical->CurAngle = get;
	critical->LastAngle = get;
	
	Cloud_Angle_Target[YAW][GYRO] = get; 

}


float speed_threshold = 5.f;//�ٶȹ���
float debug_speed;//�����Ҹ�,һ�㶼��1����,debug��
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�
		
		
//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//�ٶ�б�±仯
//		}                                                                                           //DEBUG��
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//�ٶ�б�±仯
//		}
		
		
		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���

	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}

/**
  * @brief  ����YAWƫ�����ĽǶ�,���̸���ģʽ��
  * @param  void
  * @retval sAngleError,ƫ��Ƕ�ֵ,CAN�����Ļ�е�Ƕ�
  */
int16_t GIMBAL_GetOffsetAngle(void)
{
	return FALSE;
}


/*----------------------���飬Ԥ��yaw��ĸ�������------------------------------------------------------------------------------*/
/**
  * @brief  ����yaw��Ԥ���Ƿ��Ѿ�����
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
	return FALSE;
}

/**
  * @brief  yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention ���Ҹ����ӳ٣�����ʱ�ǵ����㷴��;�ֹʱ���ӳ�
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
	return FALSE;
}




/*------------------------�ڱ��������Ԥ�⸨������-----------------------------------------------------------------*/
/**
  * @brief  �Ƿ��������ڱ�
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
bool GIMBAL_AUTO_PITCH_SB(void)
{
	return FALSE;
}


/**
  * @brief  �Ƿ����еȾ��������ڱ�,�Ӵ�Ԥ��
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
	return FALSE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*----------------���½ǶȵĻ�е�ǶȺ������ǽǶȵĺ���------------------------------------------------------------*/

//���������̨��ֵ�ĽǶ�
static fp32 motor_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd = ecd - 4096;
    if (relative_ecd > Half_ecd_range)
        relative_ecd -= ecd_range;
		
    else if (relative_ecd < -Half_ecd_range)
        relative_ecd += ecd_range;

    return relative_ecd * Motor_Ecd_to_Rad;
}

/**
  * @brief  ������̨��е�Ƕ�,���ٶ�,����ֵ,can1�ж��е���
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_UpdateAngle( char ID, int16_t angle )
{
	if (ID == PITCH)
	{
		angleMotorPit = angle;
		Cloud_Angle_Measure[PITCH][MECH]  = -motor_ecd_to_angle_change(angleMotorPit);
		PitchAngle = angleMotorPit;
	}
	else if (ID == YAW)
	{
		angleMotorYaw = angle;
		Cloud_Angle_Measure[YAW][MECH]  = -motor_ecd_to_angle_change(angleMotorYaw);
		YawAngle = angleMotorYaw;
	}
}

void GIMBAL_UpdateSpeed( char ID, int16_t speed )
{
	if (ID == PITCH)
	{
		speedMotorPit = speed;
		Cloud_Speed_Measure[PITCH][MECH]  = speedMotorPit;
	}
	else if (ID == YAW)
	{
		speedMotorYaw = speed;
		Cloud_Speed_Measure[YAW][MECH]  = speedMotorYaw;
	}
}

void GIMBAL_UpdateCurrent( char ID, int16_t current )
{
	if (ID == PITCH)
	{
		currentMotorPit = current;
		Cloud_Current_Measure[PITCH][MECH]  = currentMotorPit;
	}
	else if (ID == YAW)
	{
		currentMotorYaw = current;
		Cloud_Current_Measure[YAW][MECH]  = currentMotorYaw;
	}
}


void Gimbal_Error_Read(void)
{
	Error[YAW][MECH]  = Cloud_Angle_Target[YAW][MECH] -Cloud_Angle_Measure[YAW][MECH];
	Error[YAW][GYRO]  = Cloud_Angle_Target[YAW][GYRO] -Cloud_Angle_Measure[YAW][GYRO];
	Error[PITCH][MECH]= Cloud_Angle_Target[PITCH][MECH] -Cloud_Angle_Measure[PITCH][MECH];
	Error[PITCH][GYRO]= Cloud_Angle_Target[PITCH][GYRO] -Cloud_Angle_Measure[PITCH][GYRO];
}


static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;

}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
        return 0.0f;
		
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid��������
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
        return;
		
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
  * @brief  ������̨��̬,500HZ,loop�е���
  * @param  void
  * @retval void
  * @attention �Ƕ��ʶȷŴ�
  */
float AngleMpuYaw[2];
float AngleMpuPitch[2];
float AngleMpuRoll[2];

void GIMBAL_MPU_Update(void)
{
	//��ȡ������  �Ƕ�   ���ٶ�   
	mpu_dmp_get_data( &angleMpuPitch, &angleMpuRoll, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	AngleMpuYaw[NOW]   = angleMpuYaw-AngleMpuYaw[LAST];
	AngleMpuPitch[NOW] = angleMpuPitch-AngleMpuPitch[LAST];
	AngleMpuRoll[NOW]  = angleMpuRoll-AngleMpuRoll[LAST];
	
			//�������ǽǶȷŴ�,���Ŵ���������ģʽ�ڻ�PҪ���ܴ�,�᲻�õ�
		Cloud_Angle_Measure[PITCH][GYRO]  =  (AngleMpuPitch[NOW]*PI)/180;
	  Cloud_Angle_Measure[YAW][GYRO] = (AngleMpuYaw[NOW]*PI)/180 ;
	  //theta_format(Cloud_Angle_Measure[YAW][GYRO]);
	
		//���ٶȸ���
		Cloud_Palstance_Measure[PITCH][MECH] = ((palstanceMpuPitch + PALST_COMPS_PITCH)*PI)/180;
		Cloud_Palstance_Measure[YAW][MECH]   = ((palstanceMpuYaw+PALST_COMPS_YAW)*PI)/180;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH)/10;   //ֱ�Ӷ������ǽ��ٶ�
		Cloud_Palstance_Measure[YAW][GYRO]   = (palstanceMpuYaw+PALST_COMPS_YAW)/10;  //��������ó��Ľ��ٶ�
}


void MPU_Update_last(void)
{
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	AngleMpuYaw[LAST] = angleMpuYaw;
	AngleMpuPitch[LAST] = angleMpuPitch;
	AngleMpuRoll[LAST] = angleMpuRoll;
}