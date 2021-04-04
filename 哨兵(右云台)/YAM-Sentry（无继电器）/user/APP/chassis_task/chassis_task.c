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
#include "chassis_task.h"
#include "start_task.h"
#include "rc.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "key.h"
#include "stdio.h"
#include "arm_math.h"

#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include "Remote_Control.h"
#include "usart6.h"
#include "chassis_behaviour.h"
#include "delay.h"

#include "kalman.h"
#include "judge.h"
#include "user_lib.h"


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
		

extern  RC_ctrl_t rc_ctrl;
ChassisCtrlMode Chassis_Mode;
extKalman_t Chassis_Error_Kalman; //����һ��kalmanָ��
		
PidTypeDef motor_pid[4];
PidTypeDef chassis_angle_pid;

first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;
		
#define abs(x) ((x)>0? (x):(-(x)))
		
extern int16_t Sensor_data[2];

bool remote_change = FALSE;	
bool if_beat = FALSE;              //�ڱ�������
uint16_t last_remain_HP;
uint16_t now_remain_Hp;

		
extern float Revolver_Final_Output;
extern  float fMotorOutput[4] ;
		
	

//������
void chassis_task(void *pvParameters)
{
//    //����һ��ʱ��
      vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
				{
            Chassis_Init();					
				}
				else
				{
					
					if (SYSTEM_GetRemoteMode() == RC) //ң��ģʽ
					{													
						Chassis_Set_Mode();    //�л�ģʽ  
            chassis_feedback_update();	//��������			  				
            Chassis_Rc_Control(); //ң��������   
						Chassis_Set_Contorl();  //��ͬģʽ��ͬ����	  					
					}				
					else
					{
						sensor_update();    //���´�������Ϣ
						chassis_feedback_update();
						Chassis_AUTO_Ctrl();
						Chassis_Set_Contorl();  //��ͬģʽ��ͬ����
					}
				}
				Chassis_Omni_Move_Calculate();  //����ȫ���˶�����
				Chassis_Motor_Speed_PID(); //PID����				
				
				//����̨����
				
				CAN_CMD_Transfer(Sensor_data[LEFT], Sensor_data[RIGHT],fMotorOutput[PITCH] , fMotorOutput[YAW] ,Revolver_Final_Output);
				
				vTaskDelay(TIME_STAMP_2MS);
			}
}

/*----------------------------------------------***�������ȫ���ƶ�����***-------------------------------------------------*/

float Chassis_Move_X;//ǰ��

fp32 motor_chassis_speed[4];     //�����ĸ����ӵ��ٶȣ����ڸ���   m/s
/*----------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------***������***-------------------------------------------------------------*/

//��еģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Mech_Chassis_Standard; //�����еģʽƽ��ң������Ӧ

/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***�ٶ��޷�***-------------------------------------------------------------*/


fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
fp32 vz_max_speed;  //���ҷ�������ٶ� ��λm/s
fp32 vz_min_speed;  //���ҷ�����С�ٶ� ��λm/s

float Chassis_Standard_Move_Max;//����ǰ������ƽ������
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯


/*-----------------------------------------------***б�²���***-------------------------------------------------------------*/

float Slope_Chassis_Move_Z;//б�¼�������ƶ�����,����Ŀ���������ʵʱб��ֵ

uint16_t timeInc;//б�����ӱ仯ʱ��
uint16_t timeInc_Saltation;//ǰ����ͻ���µ�б��������,�����������ҪС

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//����ģʽ��Ťͷб��,��Ҫ����Ťƨ��ģʽ��
float Slope_Chassis_Revolve_Move;

/*-----------------------------------------------***��翪��***--------------------------------------------------------*/
#define Change_TurnBack_Time   100       //ʶ�𵽵�ʱ�䣬�������ʱ�����ʶ��        
fp32 CJ_L;
fp32 CJ_R;
bool flag=TRUE;
Flag change;
/*-----------------------------------------------***����PID����***--------------------------------------------------------------*/

//���������ٶ�
float Chassis_Speed_Target[4];//ID

//�����ٶ����
float Chassis_Speed_Error[4];//ID

//���������Ƕ�
float Chassis_Angle_Target[4];//ID

//���̽Ƕ����
float Chassis_Angle_Error[4];//ID

//���̲����Ƕ�
float Chassis_Angle_Measure[4];

//���̲����ٶ�
int16_t Chassis_Speed_Measure[4];

//���̲�������ֵ
int16_t Chassis_Current_Measure[4];

//�������ź�
int16_t Sensor_data[2];


fp32 Speed_Measure[2];
fp32 current_measure[2];
//�����ٶ�����
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//���̽Ƕ�����
float Chassis_Angle_Error_Sum[4];//ID
float Chassis_Angle_Error_NOW[4], Chassis_Angle_Error_LAST[4];

//����PID����
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	  pidTermChassis[4];//ID,���������

//���̵�������
float Chassis_Final_Output[4];
/*----------------------------------------------------------------------------------------------------------------------*/






//���̵�������
float Chassis_Final_Output[4];

/**
  * @brief  ���̳�ʼ��  ��Ҫ��PID �޷���ʼ��
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Init(void)
{
   	flag = TRUE;
	  remote_change = TRUE;
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    uint8_t i;
		
    //��ʼ��PID �˶�
    for (i = 0; i < 4; i++)
    {

        PID_Init(&motor_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
			
    }		
			
	 //��һ���˲�����б����������
    first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
		
		vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
		vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
		
		
    /**************PID����*************************/	
	  Chassis_Speed_kpid[FRON][KP] = 11;
	  Chassis_Speed_kpid[FRON][KI] = 0.08;//0.08;
	  Chassis_Speed_kpid[FRON][KD] = 0;
	
	  Chassis_Speed_kpid[BACK][KP] = 18;
	  Chassis_Speed_kpid[BACK][KI] = 0.08;//0.08;
	  Chassis_Speed_kpid[BACK][KD] = 0;
		
    chassis_feedback_update();
    change.TO_left = FALSE;
    change.TO_right = FALSE;	
    change.left = FALSE;	
    change.right = FALSE;		
}




/**
  * @brief  ���µ��̵������   �ٶ�  ����̨�ĵ��ӽǶ�
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void chassis_feedback_update(void)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
      motor_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN*Chassis_Speed_Measure[i];  //ת����m/s��ת��			
    }	
}


/**
  * @brief  ң�����л�ģʽ�����޸���Ӧ����
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Mode(void)
{
	if(IF_RC_SW2_DOWN)
	{
    Chassis_AUTO_Ctrl();
		flow_led_on(3);   
		flow_led_off(2);
	}
	else
	{
		Chassis_Mode=CHASSIS_MECH_MODE;  //������еģʽ	
		flow_led_on(2);
		flow_led_off(3);
	}
}



/*********************************************************************************************************/

/**
  * @brief  ң�������ƣ��ó�����ȫ���˶��ٶ�
  * @param  void
  * @retval void
  * @attention  Chassis_Move_Z Chassis_Move_X  Chassis_Move_Y
  *              
  */
	
void Chassis_Rc_Control(void)
{
    //ң����ԭʼͨ��ֵ
    int16_t vx_channel;
    fp32 vx_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	  if(Chassis_Mode == CHASSIS_MECH_MODE)
		{
      rc_deadline_limit(rc_ctrl.rc.ch[2], vx_channel, CHASSIS_RC_DEADLINE);
			vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	  }
		else if(Chassis_Mode == CHASSIS_R_MODE)
		{
      vx_set_channel = AUTO_MOVE_SPEED;	
		}
		else if(Chassis_Mode == CHASSIS_L_MODE)
		{
			vx_set_channel = -AUTO_MOVE_SPEED;	
		}
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
	
    //ֹͣ�źţ�����Ҫ���������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
       chassis_cmd_slow_set_vx.out = 0.0f;
    }
    Chassis_Move_X = chassis_cmd_slow_set_vx.out;
}




/*-------------------------------------***��翪���ж�(0�жϵ���1û�жϵ�)***----------------------------------------*/

void sensor_update(void)
{	

	//����̨����
	CJ_L=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10); //D
	CJ_R=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11); //C
	
	
	Sensor_data[LEFT] = CJ_L;
	Sensor_data[RIGHT] = CJ_R;
	
	


	static uint16_t change_time = 0;
	


	if(remote_change == TRUE) //�ӻ�еģʽ�й���
	{
		remote_change = FALSE;
		CHASSIS_REST();
		Chassis_Mode = CHASSIS_L_MODE;
	}
	if(CJ_L==1 && CJ_R==0 && Chassis_Mode==CHASSIS_R_MODE)  //��ߵ�û��ʶ�𵽣��ұߵ�ʶ���ˣ����������Ҷ���������߶�
	{
		change_time++;
		if(change_time > Change_TurnBack_Time)
		{
			change_time = 0;
			change.TO_left = TRUE;
			change.TO_right = FALSE;
			flag = TRUE;
		}
	}
	else if(CJ_L==0 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)  //��ߵ�ʶ�𵽣��ұߵ�û��ʶ�𵽣����������󶯣������ұ߶�
	{
		change_time++;
		if(change_time > Change_TurnBack_Time) 
		{
		  change_time = 0;
		  change.TO_right = TRUE;
			change.TO_left = FALSE;
			flag = TRUE;
		}
	}
	else if(((CJ_R==0 && CJ_L==0 )|| if_beat)&& Chassis_Mode==CHASSIS_R_MODE && flag == TRUE)  //����ܵ����򣬻������Ҷ�ʶ���ϰ���1s���ϣ��Ͱ������� 
	{
		if(if_beat)
			if_beat = 0;
		
		change_time++;		
		if(change_time > 500)
		{
		  change_time = 0;
		  change.TO_left = TRUE;
			change.TO_right = FALSE;
			flag = FALSE;
		}
	}
	else if(((CJ_R==0 && CJ_L==0 )|| if_beat) && Chassis_Mode==CHASSIS_L_MODE && flag == TRUE)
	{
		if(if_beat)
			if_beat = 0;
		
		change_time++;		
		if(change_time > 500)
		{
		  change_time = 0;
		  change.TO_right = TRUE;
			change.TO_left = FALSE;
			flag = FALSE;
		}
	}
}

/**
  * @brief  �ڱ�����
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X
  *              
  */
void Chassis_AUTO_Ctrl(void)
{
	sensor_update();
  if(change.TO_left == TRUE)  //��ߵ�û��ʶ�𵽣��ұߵ�ʶ���ˣ����������Ҷ���������߶�
	  Chassis_Mode = CHASSIS_L_MODE;  //����
		
	else if (change.TO_right == TRUE)	//��ߵ�ʶ�𵽣��ұߵ�û��ʶ�𵽣����������󶯣������ұ߶�		
	  Chassis_Mode = CHASSIS_R_MODE;  //����
		
	else if (CJ_L==1 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)  //��ʼ����ʱ��
	{
		Chassis_Mode = CHASSIS_L_MODE;
		flag = TRUE;
	}
	
	
	
//	else if(CJ_L==1 && CJ_R==1 && Chassis_Mode==CHASSIS_L_MODE)
//	{
//		Chassis_Mode = CHASSIS_R_MODE;
//		flag = TRUE;
//	}
}



/**
  * @brief  ��ͬģʽ��ͬ����ʽ
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Contorl(void)
{
	static fp32 change_time = 0;
	if(Chassis_Mode == CHASSIS_MECH_MODE)			
	{    
		remote_change = TRUE;
    Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);		
	}	
	else if(Chassis_Mode == CHASSIS_R_MODE)
	{		
	  change.left = FALSE;
		if(change.right == FALSE)
		{
			Chassis_Move_X = fp32_constrain( 0, -vx_max_speed, vx_max_speed);//ǰ��
			change_time++;
			if(change_time > 300)
			{      
				change_time = 0;
				change.right = TRUE;			
			}
	  }
		else if(change.right == TRUE)
		{
		  Chassis_Move_X = fp32_constrain( AUTO_MOVE_SPEED, -vx_max_speed, vx_max_speed);//ǰ��
		}
	}
	else if(Chassis_Mode == CHASSIS_L_MODE)
	{
		change.right = FALSE;
		if(change.left == FALSE)
		{
			Chassis_Move_X = fp32_constrain( 0, -vx_max_speed, vx_max_speed);//ǰ��
			change_time++;
			if(change_time > 300)
			{      
				change_time = 0;
				change.left = TRUE;
			}
	  }
		else if(change.left == TRUE)
		{
		  Chassis_Move_X = fp32_constrain( -AUTO_MOVE_SPEED, -vx_max_speed, vx_max_speed);//ǰ��
		}
	}
}
/**
  * @brief  ����ȫ���㷨,��������ת��
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  *              	Xǰ(+)��(-)     Y��(-)��(+)     ZŤͷ
  */
void Chassis_Omni_Move_Calculate(void)
{	
		Chassis_Speed_Target[FRON] = Chassis_Move_X;
		Chassis_Speed_Target[BACK] = -Chassis_Move_X;
}



/**
  * @brief  ���̵��PID�������
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID(void) 
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;
		
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        temp = fabs(Chassis_Speed_Target[i]);
        if (max_vector < temp)
        {
             max_vector = temp;
        }
    }	

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            Chassis_Speed_Target[i] *= vector_rate;
        }
    }

    //����pid

    for (i = 0; i < 4; i++)
    {
        PID_Calc(&motor_pid[i],motor_chassis_speed[i], Chassis_Speed_Target[i]);
    }

    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        Chassis_Final_Output[i] = motor_pid[i].out;
    }		
		
}




/**
  * @brief  ��ȡ����Ƕ�
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle )
{
    Chassis_Angle_Measure[Wheel] = angle;
}


/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed )
{
	Chassis_Speed_Measure[ Wheel ] = speed;
	Speed_Measure[ Wheel ] = speed;
}


/**
  * @brief  ��ȡ���ת�ص���
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current )
{
	Chassis_Current_Measure[ Wheel ] = current;
	current_measure[ Wheel ] = current;
}

/**
  * @brief  ��ȡ��紫�����ź�
  * @param  ID,CAN����
  * @retval void
  * @attention  1Ϊδ��Ӧ�� �� 2Ϊ��Ӧ��
  */
void Sensor_UpdateMotorCur(SensorDirection direction, int16_t data)
{
	Sensor_data[direction] = data;
}





/**
  * @brief  ��ʼ���������
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X Chassis_Move_Y Chassis_Move_Z ��0
  */
void CHASSIS_REST(void)
{
	Chassis_Move_X = 0;
}
