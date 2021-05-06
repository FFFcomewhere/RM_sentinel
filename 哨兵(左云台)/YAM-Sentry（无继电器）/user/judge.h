#ifndef _JUDGE_H
#define _JUDGE_H

#include "main.h"


#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16



#define BLUE  0
#define RED   1

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         		������������      
	ID: 0x0003  Byte:  2    ���������˴������   		1Hz����  
	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı���� 
	ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ�����      �����ӷ��ͣ�10Hz 
	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz       
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0X0208  Byte:  1    �ӵ�ʣ�෢����              10Hz���ڷ��ͣ����л����˷���
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	
*/


//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       		          	= 0x0001,//����״̬����
	ID_game_result 	   			          	= 0x0002,//�����������
	ID_game_robot_survivors           	= 0x0003,//���������˴������
	ID_event_data  				   						= 0x0101,//�����¼����� 
	ID_supply_projectile_action   			= 0x0102,//���ز���վ������ʶ����
	ID_supply_projectile_booking 				= 0x0103,//���ز���վԤԼ�ӵ�����
	ID_game_robot_state    							= 0x0201,//������״̬����
	ID_power_heat_data    							= 0x0202,//ʵʱ������������
	ID_game_robot_pos        						= 0x0203,//������λ������
	ID_buff_musk												= 0x0204,//��������������
	ID_aerial_robot_energy							= 0x0205,//���л���������״̬����
	ID_robot_hurt												= 0x0206,//�˺�״̬����
	ID_shoot_data												= 0x0207,//ʵʱ�������
	ID_bullet_remaining                                  =0x208,//�ӵ�ʣ�෢����
} CmdID;


//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
	LEN_game_state       										=  3,	//0x0001
	LEN_game_result       									=  1,	//0x0002
	LEN_game_robot_survivors       					=  2,	//0x0003
	LEN_event_data  												=  4,	//0x0101
	LEN_supply_projectile_action      		  =  3,	//0x0102
	LEN_supply_projectile_booking						=  2,	//0x0103
	LEN_game_robot_state    								= 15,	//0x0201
	LEN_power_heat_data   									= 14,	//0x0202
	LEN_game_robot_pos        							= 16,	//0x0203
	LEN_buff_musk        										=  1,	//0x0204
	LEN_aerial_robot_energy   	 		    		=  3,	//0x0205
	LEN_robot_hurt        									=  1,	//0x0206
	LEN_shoot_data       										=  6,	//0x0207
	LEN_bullet_remaining        										=  6,	//0x0208
	
} JudgeDataLength;

/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    ���������˴������ */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ����� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t remain_HP; 
	uint16_t max_HP; 
	uint16_t shooter_id1_17mm_cooling_rate; 
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate; 
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;

	uint16_t shooter_id1_42mm_cooling_rate; 
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit; 
	uint8_t mains_power_gimbal_output : 1; 
	uint8_t mains_power_chassis_output : 1; 
	uint8_t mains_power_shooter_output : 1;

} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;  

} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type;
	uint8_t shooter_id; 
	uint8_t bullet_freq; 
	float bullet_speed;
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    �ӵ�ʣ�෢���� */
typedef __packed struct { 
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num; 
} ext_bullet_remaining_t;


/* 
	
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	����Ƶ�ʣ����� 10Hz


	1.	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	�ֽ�ƫ���� 	��С 	˵�� 				��ע 
	0 			2 		���ݵ����� ID 		0xD180 
	2 			2 		���ߵ� ID 			��ҪУ�鷢���߻����˵� ID ��ȷ�� 
	4 			2 		�ͻ��˵� ID 		ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ��� 
	6 			4 		�Զ��帡������ 1 	 
	10 			4 		�Զ��帡������ 2 	 
	14 			4 		�Զ��帡������ 3 	 
	18 			1 		�Զ��� 8 λ���� 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;

//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//֡ͷ
	uint16_t		 						CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	client_custom_data_t  					clientData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_SendClientData_t;


//�����˽�����Ϣ
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//֡ͷ
	uint16_t								CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 			interactData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_CommunatianData_t;

bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);

bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
float JUDGE_usGetSpeedHeat17(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);
uint16_t JUDGE_remain_HP(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);

#endif
