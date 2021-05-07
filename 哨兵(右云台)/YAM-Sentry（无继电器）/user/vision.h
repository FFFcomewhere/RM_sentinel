#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define ATTACK_NONE    0	//��ʶ��
#define ATTACK_RED     1	//ʶ��췽
#define ATTACK_BLUE    2	//ʶ������


#define    VISION_DATA_ERROR      0          //�Ӿ����ݴ���
#define    VISION_DATA_CORRECT    1          //�Ӿ����ݴ���

#define    VISION_LEN_HEADER      3          //֡ͷ��
#define    VISION_LEN_DATA        17         //���ݶγ���,���Զ���
#define    VISIOV_LEN_TAIL        2	         //֡βCRC16
#define    VISION_LEN_PACKED      22         //���ݰ�����

#define    VISION_OFF         		  (0x00)   //�ر��Ӿ�
#define    VISION_RED           	  (0x01)   //ʶ���ɫ
#define    VISION_BLUE          	  (0x02)   //ʶ����ɫ
#define    VISION_RBUFF_ANTI   	 	  (0x03)   //���� ���
#define    VISION_BBUFF_ANTI   		  (0x04)   //���� ���
#define    VISION_RBUFF_CLOCKWISE   (0x05)   //��˳ ���
#define    VISION_BBUFF_CLOCKWISE   (0x06)   //��˳ ���
#define    VISION_RBUFF_STAND   	  (0x07)   //�� С��
#define    VISION_BBUFF_STAND   	  (0x08)   //�� С��

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF              (0xA5)     //�ɸ��ģ�
#define    VISION_WEI              (0xFF)     //֡β

/*-------�Ӿ��ֱ���Ԥ����--------*/
#define VISION_MID_YAW		444//640
#define VISION_MID_PITCH	500//360

/*------------------����Ԥ����,�Ƕȳ�ʼ������------------------------*/
#define	COMPENSATION_YAW	0
#define	COMPENSATION_PITCH	0
#define COMPENSATION_PITCH_DIST 0



/* 	STM32 -> PC

	CmdID   0x00   �ر��Ӿ�
	CmdID   0x01   ʶ���ɫװ��
	CmdID   0x02   ʶ����ɫװ��
	CmdID   0x03   ���
	CmdID   0x04   ����
*/

/* 	PC -> STM32

	CmdID   0x00   �ر��Ӿ�
	CmdID   0x01   ʶ���ɫװ��
	CmdID   0x02   ʶ����ɫװ��
	CmdID   0x03   С��
	CmdID   0x04   ���
*/

//�������պͷ���ָ������бȽ�,���պͷ���ָ������ͬʱ,���ж�Ϊ���ݿ���

//֡ͷ��CRC8У��,��֤���͵�ָ������ȷ��

//PC�շ���STM32�շ��ɾ����ϵ,���½ṹ��������STM32,PC�������޸�

typedef enum
{
	VISION_MANU =0,
	VISION_BUFF =1,
	VISION_AUTO =2,
}VisionActData_t;      //�Ӿ�ģʽѡ��


typedef __packed struct    //3 Byte
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
}VisionSendHeader_t;


//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct       //17 Byte
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼����
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
	uint8_t   identify_buff;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��
	
	uint8_t	  blank_b;			//Ԥ��
	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
	
	/* β */
	uint16_t  CRC16;     //֡β     
	
}VisionRecvData_t;

//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef struct
{

	
	/* ���� */
	float     pitch_angle;     //��ǰ�Ƕ�
	float     yaw_angle;       //��ǰ�Ƕ�                                              (��е?������?)�)???????????
	float     distance;			   //����
	uint8_t   lock_sentry;	 	 //�Ƿ���̧ͷʶ���ڱ�
	uint8_t   base;				     //����
	
	uint8_t   blank_a;		//Ԥ��
	uint8_t	  blank_b;
	uint8_t	  blank_c;	
	
	/* β */
	uint16_t  CRC16;
	
}VisionSendData_t;


//�������д��CRCУ��ֵ
//���ǿ���ֱ�����ùٷ�����CRC����

//ע��,CRC8��CRC16��ռ�ֽڲ�һ��,8Ϊһ���ֽ�,16Ϊ2���ֽ�

//д��    CRC8 ����    Append_CRC8_Check_Sum( param1, param2)
//���� param1����д����֡ͷ���ݵ�����(֡ͷ������ݻ�ûдû�й�ϵ),
//     param2����CRC8д������ݳ���,���Ƕ������ͷ�����һλ,Ҳ����3

//д��    CRC16 ����   Append_CRC16_Check_Sum( param3, param4)
//���� param3����д����   ֡ͷ + ����  ������(��������ͬһ������)
//     param4����CRC16д������ݳ���,���Ƕ�����������ݳ�����22,������22

/*----------------------------------------------------------*/

//������ID,�����жϽ��յ���ʲô����


void Vision_Read_Data(uint8_t *ReadFormUart7);//�Ӿ���ȡ����
void Vision_Send_Data( uint8_t CmdID );//�Ӿ���������
void Vision_Ctrl(void);//�Ӿ�����
void Vision_Buff_Ctrl(void);//�������
void Vision_Auto_Attack_Ctrl(void);//�������
void Vision_Auto_Attack_Off(void);//�ر�����


bool Get_Vision_distance(void); //�Ӿ��ж�Զ�������ڵ�������Ƶ
void Vision_Get_Distance(float *distance); //��ȡ����

/********�Ӿ���������*********/
uint8_t VISION_isColor(void);
uint8_t VISION_BuffType(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);
bool Vision_If_Armor(void);
void Vision_Clean_Ammor_Flag(void);


/*****�Ӿ�ƫ���ȡ******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Buff_Error_Angle_Yaw(float *error);
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error);
void Vision_Buff_Error_Angle_Pitch(float *error);
void Vision_Base_Yaw_Pixel(float *error);

#endif
