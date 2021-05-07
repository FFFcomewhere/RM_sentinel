#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define ATTACK_NONE    0	//不识别
#define ATTACK_RED     1	//识别红方
#define ATTACK_BLUE    2	//识别蓝方


#define    VISION_DATA_ERROR      0          //视觉数据错误
#define    VISION_DATA_CORRECT    1          //视觉数据错误

#define    VISION_LEN_HEADER      2          //帧头长
#define    VISION_LEN_DATA        15         //数据段长度,可自定义
#define    VISIOV_LEN_TAIL        1	         //帧尾CRC16
#define    VISION_LEN_PACKED      18         //数据包长度

#define    VISION_OFF         		  (0x00)   //关闭视觉
#define    VISION_RED           	  (0x01)   //识别红色
#define    VISION_BLUE          	  (0x02)   //识别蓝色


//起始字节,协议固定为0xA5
#define    VISION_BEGIN              (0xA5)     //可更改
#define    VISION_END              (0xFF)     //帧尾


/*-------视觉分辨率预编译--------*/
#define VISION_MID_YAW		444//640
#define VISION_MID_PITCH	500//360

/*------------------自瞄预编译,角度初始化补偿------------------------*/
#define	COMPENSATION_YAW	0
#define	COMPENSATION_PITCH	0
#define COMPENSATION_PITCH_DIST 0



/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲

*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用


//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef enum
{
	VISION_MANU =0,
	VISION_BUFF =1,
	VISION_AUTO =2,
}VisionActData_t;      //视觉模式选择


typedef __packed struct    //3 Byte
{
	/* 头 */
	uint8_t   BEGIN;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	
}VisionSendHeader_t;



//STM32接收,直接将串口接收到的数据拷贝进结构体 17帧
typedef __packed struct       //18 Byte
{
	/* 头 */
	uint8_t  BEGIN;			//帧头起始位,暂定0xA5
	uint8_t   CmdID; 		//指令
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	
	uint8_t	  identify_buff;	//视野内是否有能量机关/是否识别到了目标   0否  1是	
	
	uint8_t   END;
}VisionRecvData_t;

//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{
	uint8_t SOF;
	uint8_t CmdID;
	uint8_t speed;  //速度
	
	uint8_t END;
}VisionSendData_t;


/*----------------------------------------------------------*/

//命令码ID,用来判断接收的是什么数据


void Vision_Read_Data(uint8_t *ReadFormUart7);//视觉读取数据
void Vision_Send_Data( uint8_t CmdID );//视觉发送数据
void Vision_Ctrl(void);//视觉控制
void Vision_Buff_Ctrl(void);//打符控制
void Vision_Auto_Attack_Ctrl(void);//自瞄控制
void Vision_Auto_Attack_Off(void);//关闭自瞄


bool Get_Vision_distance(void); //视觉判断远近，用于调射速射频
void Vision_Get_Distance(float *distance); //获取距离

/********视觉辅助函数*********/
uint8_t VISION_isColor(void);
uint8_t VISION_BuffType(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);
bool Vision_If_Armor(void);
void Vision_Clean_Ammor_Flag(void);


/*****视觉偏差获取******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Buff_Error_Angle_Yaw(float *error);
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error);
void Vision_Buff_Error_Angle_Pitch(float *error);
void Vision_Base_Yaw_Pixel(float *error);

#endif
