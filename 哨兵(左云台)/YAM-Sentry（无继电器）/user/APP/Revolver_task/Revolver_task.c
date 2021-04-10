#include "main.h"
#include "Revolver_task.h"
#include "start_task.h"
#include "friction.h"
#include "fric.h"
#include "friction.h"
#include "vision.h"
#include "led.h"
#include "laser.h"
#include "stdio.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include "Remote_Control.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
extern  RC_ctrl_t rc_ctrl;
extern VisionRecvData_t VisionRecvData;
#define abs(x) ((x)>0? (x):(-(x)))

/******����,�����߼�����̨����*********/

//���̵��ģʽ,λ�û����ٶȻ�
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//����ģʽѡ��
//���̵��ģʽ,λ�û����ٶȻ�


/*********����**********/
#define Revolver_Angle_Mid  4092
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_STOP_MODE = 2,
}eRevolverCtrlMode;
eRevolverCtrlMode Revolver_mode;

/*****************����Ƶ���뽵�����٣����������Ħ�������***********************/


#define REVOL_SPEED_RATIO   2160       //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��
#define Revolver_Speed    1620
/*       �����ͺ�Ԥ����       */
#define SEVEN_REVOLVER    0    	//7����
#define EIGHT_REVOLVER    1		//8����
#define TEN_REVOLVER      2		//10����
#define TWELVE_REVOLVER   3		//12����

#define REVOLVER_CHOOSE  EIGHT_REVOLVER	//ѡ�����ͺ�

#if REVOLVER_CHOOSE == EIGHT_REVOLVER			
	#define		REVOL_SPEED_GRID      8				//���̸���
	#define 	AN_BULLET         (36864.0f)		//�����ӵ����λ������ֵ

#elif REVOLVER_CHOOSE == SEVEN_REVOLVER
	#define 	REVOL_SPEED_GRID      7				//���̸���
	#define   	AN_BULLET     	  (42130.2857f)		//�����ӵ����λ������ֵ

#elif REVOLVER_CHOOSE == TEN_REVOLVER
	#define 	REVOL_SPEED_GRID  	  10			//���̸���
	#define    	AN_BULLET     	  (29491.2f)		//�����ӵ����λ������ֵ

#elif REVOLVER_CHOOSE == TWELVE_REVOLVER
	#define 	REVOL_SPEED_GRID      12			//���̸���
	#define    	AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ

#endif	



/*------------------------------------------------***λ��PID***---------------------------------------------------------*/
float Revolver_Angle_Measure_Sum;//���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Revolver_Angle_Measure_Prev;//�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�
//���������Ƕ�
float  Revolver_Angle_Target_Sum;
float  Revolver_Buff_Target_Sum;//���ģʽ�µ�Ŀ��ֵ�ݴ棬��ʱĿ��Ƕ���б�´���
float  Revolver_Buff_Ramp = AN_BULLET/40;//40msתһ��,һ�����ܳ���50ms


/********���**********/
//�����ӵ���,��һ�¼�һ��,��һ�ż�һ��
int16_t Key_ShootNum;//����������

//�ۼƺ�
float Revolver_Angle_Measure_Sum;//���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Revolver_Angle_Measure_Prev;//�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�

//���̵�������,������ʱ��
float Revolver_Final_Output;

//���̽Ƕ����
float Revolver_Angle_Error[2];//  inner/outer



float pTermRevolSpeed, iTermRevolSpeed;	
float pTermRevolAngle[2], iTermRevolAngle[2],pidTermRevolAngle[2];//  inner/outer
float Revolver_Speed_kpid[3];//	kp/ki/kd
float Revolver_Angle_kpid[2][3];//  inner/outer    kp/ki/kd
float kRc_Chassis_Standard; //�����еģʽƽ��ң������Ӧ


/************����************/
#define Stuck_Revol_PIDTerm   500      //PID����������������Ϊ�п��ܿ���
#define Stuck_Speed_Low       60       //�����ٶȵ��������,����Ϊ�п��ܿ���

#define Stuck_SpeedPID_Time   100       //�ٶ����� ms��С,PID����  ms����
#define Stuck_TurnBack_Time   500       //��תʱ��,ʱ��Խ������Խ��
uint32_t Stuck_Speed_Sum = 0;//���㿨������,�ٶȻ�
uint32_t Stuck_Posit_Sum = 0;//���㿨������,λ�û�

portTickType posishoot_time;//�����ʱ����


/**********�޷�*************/
//��������޷�
float Revolver_Output_Max;
float Revolve_Move_Max;
float Revolve_Posi_Max;
float kRc_Mech_Standard; //�����еģʽƽ��ң������Ӧ
float kRc_Mech_Revolve;  //�����еģʽ��תң������Ӧ
float iTermRevolPosiMax;//λ�û������޷�
float iTermRevolSpeedMax;//�ٶȻ������޷�


/*******************���̲���**********************/

//���̲����Ƕ�
float Revolver_Angle_Measure;

//���̲����ٶ�
float Revolver_Speed_Measure;

//���̲�������
float Revolver_Current_Measure;

//�����ٶ����
float Revolver_Speed_Error;

//����Ŀ��ת��
float  Revolver_Speed_Target;//ת�ٹ������׿���,������ת����6000




//�����ٶȻ���Ƶ
int16_t Revolver_Freq;

#define  Revolver_Speed_Low 2.5
#define  Revolver_Speed_Mid 3   //5
#define  Revolver_Speed_High 10

/*********************************************Ħ����*********************************************************/

#define    REVOL_CAN_OPEN    340  //Ħ����ʵ���ٶȳ������ֵ��������ת��,����Ħ������СĿ���ٶ����ı�


uint8_t revol_remot_change = TRUE;
void Revolver_task(void *pvParameters)
{
//    //����һ��ʱ��
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
				{
				   REVOLVER_Rest();
           Revolver_Init();	
					 friction_Init();
					 
				}
				else
				{
					if (SYSTEM_GetRemoteMode() == RC) //ң��ģʽ
					{										
						Revolver_RC_Ctrl();					
						friction_RC_Ctrl();
					}
					else
					{
						Revolver_AUTO_Ctrl();
						friction_AUTO_Ctrl();
						revol_remot_change = TRUE;
					}
				}
				
				if (Revolver_mode==REVOL_SPEED_MODE)//�ٶȻ�
				{
					REVOL_SpeedLoop();
					Fric_mode(FRI_LOW);
				}

				else if(Revolver_mode==REVOL_POSI_MODE)//λ�û�
				{
					REVOL_PositionLoop();
					Fric_mode(FRI_LOW);
				}
				
				else if(Revolver_mode==REVOL_STOP_MODE)
				{
				  REVOL_SpeedLoop();
					Fric_mode(FRI_OFF);
				}
				
				
				
//				if(Fric_GetSpeedReal() > REVOL_CAN_OPEN)//Ħ���ֿ���
//		    {
//					laser_on();
//					Revolver_CANSend();
//				}
//				else
//				{
//					Revolver_Speed_Target = 0;//Ħ���ֹر�,���̲�����
//					Revolver_Angle_Rest();//Ħ���ֹرգ��������ڼ�Ĵ�ָ��
//					REVOL_SpeedLoop();
//					laser_off();
//					Revolver_CANSend();
//				}
				
				vTaskDelay(TIME_STAMP_2MS);
			}
}


/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention ǹ�ڳ���������
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//λ�û���������
	Revolver_Speed_Target = 0;//�ٶȻ�ֹͣת��
	
	
	
	//�ٶȻ�λ������
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Measure;
	
	
	//PID��������
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

void Revolver_Init(void)
{
	/* Ŀ��ֵ */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;
	Key_ShootNum = 0;
	/**************ң����������*************************/	
  kRc_Mech_Standard = 14.f;  //����ҡ��������,�����ٶ�
  kRc_Mech_Revolve  = 11.4f; //���ڻ�еģʽҡ��Ťͷ������(̫С��Ӱ������ٶ�)
	
	
/********************************************/
/**************PID����*************************/	
	
//�Ƕ�	
	Revolver_Angle_kpid[OUTER][KP] = 0.08;//0.005
	Revolver_Angle_kpid[OUTER][KI] = 0;
	Revolver_Angle_kpid[OUTER][KD] = 0;
	Revolver_Angle_kpid[INNER][KP] = 6;//1.8
	Revolver_Angle_kpid[INNER][KI] = 0;
	Revolver_Angle_kpid[INNER][KD] = 0;
	
		/* PID���� */
	  //�ٶȻ�
	Revolver_Speed_kpid[KP] = 14;
	Revolver_Speed_kpid[KI] = 0.0f;
	Revolver_Speed_kpid[KD] = 0;
	

	/* λ�û�Ŀ��Ƕ� */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;//������0,�����ϵ�ᷴת
	Revolver_Buff_Target_Sum  = Revolver_Angle_Measure;	
	
	
		
/********************************************/
/**************�޷�����*************************/
  Revolve_Move_Max = 10000;
	iTermRevolSpeedMax  = 250;
	iTermRevolPosiMax   = 2500;
	Revolver_Output_Max = 9999;  
	
/********************************************/
}


/**
  * @brief  ���̽Ƕ�����
  * @param  void
  * @retval void
  * @attention ģʽ�л�ʱ��,��ֹ�´��л�ȥ��ͻȻ��һ��
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Buff_Target_Sum 	= Revolver_Angle_Target_Sum;
}



/**
  * @brief  ���̵�ң��ģʽ
  * @param  void
  * @retval void
  * @attention  ң�����ٶȻ�
  *              
  */


void Revolver_RC_Ctrl(void)
{	
	
	/*******************�����**********************/
	
	if(IF_RC_SW2_UP)//��еģʽ�µ�����������Ե���
	{
		Revolver_mode = REVOL_POSI_MODE;//λ�û�
		if(REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++;//��һ��
		}
		
		if(revol_remot_change == TRUE)//�մӼ���ģʽ�л���������շ�������
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest();//��ֹͻȻ�Ӽ����е�ң�ؿ�ת
		}
		
		if(Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Buff_Target_Sum += AN_BULLET;
		}
		
		if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//����ת��ȥ
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
		}
		
		if(IF_RC_SW1_UP)
		{
     Reset_Fric();
		}
		else
		{
     Fric_mode(FRI_MID);
		}
		
		REVOL_PositStuck();//�����жϼ���ת
	}
	/**************������*******************/
	else if(IF_RC_SW2_MID)//�Զ�ģʽ
	{
		Revolver_mode = REVOL_SPEED_MODE;//�ٶȻ�
		
		if(IF_RC_SW1_DOWN)//sw1�´�
		{
			Revolver_Freq = Revolver_Speed_Mid;//��Ƶѡ��
			//�ٶȻ�ת������
			Revolver_Speed_Target = REVOL_SPEED_RATIO/REVOL_SPEED_GRID*Revolver_Freq;
		}
		else	//ң��ģʽ�رղ���
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
		}
		if(IF_RC_SW1_UP)  //SW1�Ϲر�Ħ����
		{
     Reset_Fric();
		}
		else
		{
    Fric_mode(FRI_MID);
		}
			
		REVOL_SpeedStuck();//�����жϼ���ת
	}
	/********************************************/
}


/**
  * @brief  stop Fric
  * @param  void
  * @retval void
  * @attention 
  */

extern uint16_t Fric_Speed_Level;
extern float Friction_Speed_Target;
extern float Friction_Speed_Real;	

void Reset_Fric(void)
{
	 Fric_Speed_Level = FRI_OFF;
	 Friction_Speed_Target = 0;
	 Friction_Speed_Real   = 0;
}


/*****************************************ҡ�ڲ���***************************************/

//long key,key0;
//void Revolver_RC_Ctrl(void)
//{	

//  if(IF_RC_SW1_DOWN)
//	{
//		Revolver_mode=REVOL_POSI_MODE;		
//		key0=key%2;		
//		if(revol_remot_change == TRUE)//�մӼ���ģʽ�л���������շ�������
//		{
//			revol_remot_change = FALSE;
//			Revolver_Angle_Rest();//��ֹͻȻ�Ӽ����е�ң�ؿ�ת
//		}		
//    if(key0 == 0)
//		{
//			Revolver_Buff_Target_Sum = -120000;
//		}
//		else
//		{
//			Revolver_Buff_Target_Sum = 120000;
//		}	
//		if(Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum)//����ת��ȥ
//		{
//			Revolver_Angle_Target_Sum = RAMP_float  (Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, 500);
//		}	
//		if(abs(Revolver_Buff_Target_Sum - Revolver_Angle_Target_Sum)< 200)
//		{
//			key=key+1;
//		}	
//	}
//}

/*****************************************************************************************/

void Revolver_AUTO_Ctrl(void)
{
	

	//���ʶ��װ�װ���ʶ�𵽰��� ����ת��	
	if (Vision_If_Update() && VisionRecvData.centre_lock)
	{
		Revolver_mode = REVOL_SPEED_MODE;  //�ٶ�
		Revolver_Speed_Target = constrain_float(Revolver_Speed, -Revolve_Move_Max, Revolve_Move_Max);
		REVOL_SpeedStuck();//�����жϼ���ת
		Vision_Clean_Update_Flag();
	}
	
}




/**
  * @brief  �ٶȻ�PID����
  * @param  void
  * @retval void
  * @attention  ң��ֻ���ٶȻ�
  */
void REVOL_SpeedLoop(void)
{  

	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//���͵���PID�㷨
	pTermRevolSpeed   = Revolver_Speed_Error * Revolver_Speed_kpid[KP];
	iTermRevolSpeed  += Revolver_Speed_Error * Revolver_Speed_kpid[KI];
	iTermRevolSpeed   = constrain( iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax );

	Revolver_Final_Output = constrain_float( pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max );
//	
	
	
	//fzj
	
//	//��ȡת�����ܽǶ�ֵ
//	REVOL_UpdateMotorAngleSum( );
//	
//	//�⻷����
//	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
//	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];

//	//�ڻ�����
//	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
//	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];		
//	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
//	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );
//  
//	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);
}


/**
  * @brief  λ�û�PID����
  * @param  void
  * @retval void
  * @attention  ����ģʽ
  */
void REVOL_PositionLoop(void)
{
	//��ȡת�����ܽǶ�ֵ
	REVOL_UpdateMotorAngleSum();
	
	//�⻷����
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];
  
		//�ڻ�����
	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];		
	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );

	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);

}



void REVOL_UpdateMotorAngleSum(void)
{		 
	//�ٽ�ֵ�жϷ�
	if (abs(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//����Ȧ�ҹ����
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;

		else
			//������һȦ
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
	}
	else      
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}




///**
//  * @brief  ����ң�ش�
//  * @param  void
//  * @retval void
//  * @attention 
//  */
//#define REVOL_STEP0    0		//ʧ�ܱ�־
//#define REVOL_STEP1    1		//SW1��λ��־
//#define REVOL_STEP2    2		//���ֿ��ر�־
//uint8_t	Revolver_Switch = 0;//����ң��ģʽ���ر�־λת��
//bool REVOLVER_Rc_Switch(void)
//{
//	if (IF_RC_SW2_UP)//��еģʽ
//	{
//		if (IF_RC_SW1_UP)
//		{
//			if (Revolver_Switch == REVOL_STEP1)
//				Revolver_Switch = REVOL_STEP2;

//			else if (Revolver_Switch == REVOL_STEP2)
//				Revolver_Switch = REVOL_STEP0;
//		}
//		else		
//			Revolver_Switch = REVOL_STEP1;
//	}
//	else
//		Revolver_Switch = REVOL_STEP0;
//	
//	if (Revolver_Switch == REVOL_STEP2)
//		return TRUE;
//	else
//		return FALSE;
//}

/**
  * @brief  ����ң�ش�
  * @param  void
  * @retval void
  * @attention 
  */
#define REVOL_STEP0    0		//ʧ�ܱ�־
#define REVOL_STEP1    1		//SW1��λ��־
#define REVOL_STEP2    2		//���ֿ��ر�־
uint8_t	Revolver_Switch = 0;//����ң��ģʽ���ر�־λת��
bool REVOLVER_Rc_Switch(void)
{
	
	if (IF_RC_SW2_UP )
	{
		if (IF_RC_SW1_DOWN)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else		
		{
			Revolver_Switch = REVOL_STEP1;
		}
	}
	else
	{
		Revolver_Switch = REVOL_STEP0;
	}
	
	if (Revolver_Switch == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*****************************��������**************************************/

/**
  * @brief  �ٶȻ�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */
void REVOL_SpeedStuck(void)
{
	static uint16_t  stuck_time    = 0;//������ʱ
	static uint16_t  turnback_time = 0;//��ת��ʱ
	static bool Revol_Speed_ifStuck = FALSE;//�����ж�

	if (Revol_Speed_ifStuck == TRUE)//��ȷ�Ͽ���,��ʼ��ת��ʱ
	{
		Revolver_Speed_Target = -4000;//��ת
		turnback_time++;//��תһ��ʱ��

		if (turnback_time > Stuck_TurnBack_Time)//��ת���
		{
			turnback_time  = 0;
			Revol_Speed_ifStuck = FALSE;//������ת
		}			
	}
	else
	{
		if ( abs(Revolver_Final_Output) >= Stuck_Revol_PIDTerm //PID�������
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//�ٶȹ���
			stuck_time++;//������ʱ

		else
			stuck_time = 0;//û�г�ʱ�俨��,��ʱ����

		if (stuck_time > Stuck_SpeedPID_Time)//���˳���60ms
		{
			Stuck_Speed_Sum++;//��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Speed_ifStuck = TRUE;//��ǿ��Խ��뵹ת��ʱ
		}
	}
}


/**
  * @brief  λ�û�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */
void REVOL_PositStuck(void)
{
	static uint16_t  stuck_time      = 0;//������ʱ
	static uint16_t  turnback_time   = 0;//��ת��ʱ
	static bool Revol_Posit_ifStuck = FALSE;//�����ж�
	
	if (Revol_Posit_ifStuck == TRUE)//������ʼ��ת��ʱ
	{
		//�����������ж��Ƿ񿨵����ʱ������갴�µ�ָ�����
		Key_ShootNum = 0;
		
		turnback_time++;//��ת��ʱ,1msһ��
		if (turnback_time > Stuck_TurnBack_Time)//��תʱ�乻��
		{
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum;//������ת,��ת�ر�����Ҫ��ת����λ��
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			Revol_Posit_ifStuck = FALSE;//��Ϊ��ʱ���ٿ�����
			turnback_time = 0;//��תʱ������,Ϊ�´ε�ת��׼��	
		}
	}
	else
	{
		if ( abs(Revolver_Final_Output)  >= Stuck_Revol_PIDTerm //PID�������
				&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low  )//�ٶȹ���
			stuck_time++;//ͳ�ƿ��˶೤ʱ��

		else
			stuck_time = 0;//������,ʱ������
		
		if (stuck_time > Stuck_SpeedPID_Time)//��̫����,��ʾҪ��ת
		{
			//��ת���ܷ���Revol_Posit_ifStuck == TRUE��,����Ͳ��Ƕ�һ�ε�ת1/2����
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum - AN_BULLET ;//��ת 1��  
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			
			Stuck_Posit_Sum++;//��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Posit_ifStuck = TRUE;//������ǵ�ת��ʱ����	
		}
		
	}
}


/**
  * @brief  ��ȡ����Ƕ�
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void Revolver_UpdateMotorAngle( int16_t angle )
{
  	Revolver_Angle_Measure = angle;
}

/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void Revolver_UpdateMotorSpeed( int16_t speed )
{
	Revolver_Speed_Measure = speed;
}


/**
  * @brief  ��ȡ���ת�ص���
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void Revolver_UpdateMotorCurrent( int16_t current )
{
	Revolver_Current_Measure = current;
}



/**
  * @brief  ���Ͳ��̵���ֵ
  * @param  void
  * @retval void
  * @attention 
  */
void Revolver_CANSend(void)
{	 	
	CAN_CMD_Revolver(Revolver_Final_Output, 0);
}

