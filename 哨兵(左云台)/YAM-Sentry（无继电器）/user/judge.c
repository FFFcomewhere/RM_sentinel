#include "judge.h"
#include "crc.h"
#include "usart6.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*****************ϵͳ���ݶ���**********************/
ext_game_state_t       				    GameState;					      //0x0001     ����״̬����
ext_game_result_t            	   	GameResult;					      //0x0002     �����������
ext_game_robot_survivors_t        GameRobotSurvivors;			  //0x0003     �����˴������
ext_event_data_t        		  	  EventData;					      //0x0101     �����¼�����
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102     ����վ������ʶ
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103     ���󲹸�վ�����ӵ�
ext_game_robot_state_t			     	GameRobotStat;				    //0x0201     ����������״̬
ext_power_heat_data_t		  	    	PowerHeatData;			    	//0x0202     ʵʱ������������
ext_game_robot_pos_t				      GameRobotPos;				      //0x0203     ������λ��
ext_buff_musk_t					        	BuffMusk;					        //0x0204     ����������
aerial_robot_energy_t			      	AerialRobotEnergy;		  	//0x0205     ���л���������״̬
ext_robot_hurt_t					        RobotHurt;					      //0x0206     �˺�״̬
ext_shoot_data_t					        ShootData;					      //0x0207     ʵʱ�����Ϣ
ext_bullet_remaining_t						BulletRemaining;					//0x0208     �ӵ�ʣ�෢����

xFrameHeader                      FrameHeader;		          //����֡ͷ��Ϣ
ext_SendClientData_t              ShowData;			            //�ͻ�����Ϣ
ext_CommunatianData_t             CommuData;		            //����ͨ����Ϣ
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID

/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������


/**
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);   //��ReadFromUsart + DATA ���� LEN_game_state �� ���ݵ� GameState��
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&SupplyProjectileBooking, (ReadFromUsart + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//������ͳ

					case ID_bullet_remaining:      //0x208
						memcpy(&BulletRemaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	
	return retval_tf;//����������������
}




/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}


/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time;//������ʱ����
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	Shoot_Speed_Now = ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	//shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return PowerHeatData.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_id1_17mm_cooling_rate;
}

/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}

bool Judge_If_Death(void)
{
	if(GameRobotStat.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  ��ǰѪ��
  * @param  void
  * @retval 
  * @attention  
  */
uint16_t JUDGE_remain_HP(void)
{
	return GameRobotStat.remain_HP;
}


/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}


