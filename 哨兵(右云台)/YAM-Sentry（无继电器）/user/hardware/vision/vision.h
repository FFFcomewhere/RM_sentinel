#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define ATTACK_NONE    0	//²»Ê¶±ğ
#define ATTACK_RED     1	//Ê¶±ğºì·½
#define ATTACK_BLUE    2	//Ê¶±ğÀ¶·½


#define    VISION_DATA_ERROR      0          //ÊÓ¾õÊı¾İ´íÎó
#define    VISION_DATA_CORRECT    1          //ÊÓ¾õÊı¾İ´íÎó

#define    VISION_LEN_HEADER      3          //Ö¡Í·³¤
#define    VISION_LEN_DATA        17         //Êı¾İ¶Î³¤¶È,¿É×Ô¶¨Òå
#define    VISIOV_LEN_TAIL        2	         //Ö¡Î²CRC16
#define    VISION_LEN_PACKED      22         //Êı¾İ°ü³¤¶È

#define    VISION_OFF         		  (0x00)   //¹Ø±ÕÊÓ¾õ
#define    VISION_RED           	  (0x01)   //Ê¶±ğºìÉ«
#define    VISION_BLUE          	  (0x02)   //Ê¶±ğÀ¶É«
#define    VISION_RBUFF_ANTI   	 	  (0x03)   //ºìÄæ ´ó·û
#define    VISION_BBUFF_ANTI   		  (0x04)   //À¶Äæ ´ó·û
#define    VISION_RBUFF_CLOCKWISE   (0x05)   //ºìË³ ´ó·û
#define    VISION_BBUFF_CLOCKWISE   (0x06)   //À¶Ë³ ´ó·û
#define    VISION_RBUFF_STAND   	  (0x07)   //ºì Ğ¡·û
#define    VISION_BBUFF_STAND   	  (0x08)   //À¶ Ğ¡·û

//ÆğÊ¼×Ö½Ú,Ğ­Òé¹Ì¶¨Îª0xA5
#define    VISION_SOF              (0xA5)     //¿É¸ü¸Ä£¿
#define    VISION_WEI              (0xFF)     //Ö¡Î²

/*-------ÊÓ¾õ·Ö±æÂÊÔ¤±àÒë--------*/
#define VISION_MID_YAW		444//640
#define VISION_MID_PITCH	500//360

/*------------------×ÔÃéÔ¤±àÒë,½Ç¶È³õÊ¼»¯²¹³¥------------------------*/
#define	COMPENSATION_YAW	0
#define	COMPENSATION_PITCH	0
#define COMPENSATION_PITCH_DIST 0



/* 	STM32 -> PC

	CmdID   0x00   ¹Ø±ÕÊÓ¾õ
	CmdID   0x01   Ê¶±ğºìÉ«×°¼×
	CmdID   0x02   Ê¶±ğÀ¶É«×°¼×
	CmdID   0x03   ºì·û
	CmdID   0x04   À¶·û
*/

/* 	PC -> STM32

	CmdID   0x00   ¹Ø±ÕÊÓ¾õ
	CmdID   0x01   Ê¶±ğºìÉ«×°¼×
	CmdID   0x02   Ê¶±ğÀ¶É«×°¼×
	CmdID   0x03   Ğ¡·û
	CmdID   0x04   ´ó·û
*/

//¿ÉÀûÓÃÊÕºÍ·¢µÄÖ¸ÁîÂë½øĞĞ±È½Ï,µ±ÊÕºÍ·¢µÄÖ¸ÁîÂëÏàÍ¬Ê±,¿ÉÅĞ¶¨ÎªÊı¾İ¿ÉÓÃ

//Ö¡Í·¼ÓCRC8Ğ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ

//PCÊÕ·¢ÓëSTM32ÊÕ·¢³É¾µÏñ¹ØÏµ,ÒÔÏÂ½á¹¹ÌåÊÊÓÃÓÚSTM32,PCĞèÉÔ×÷ĞŞ¸Ä

typedef enum
{
	VISION_MANU =0,
	VISION_BUFF =1,
	VISION_AUTO =2,
}VisionActData_t;      //ÊÓ¾õÄ£Ê½Ñ¡Ôñ


typedef __packed struct    //3 Byte
{
	/* Í· */
	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
	uint8_t   CmdID;		//Ö¸Áî
	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
}VisionSendHeader_t;


//STM32½ÓÊÕ,Ö±½Ó½«´®¿Ú½ÓÊÕµ½µÄÊı¾İ¿½±´½ø½á¹¹Ìå
typedef __packed struct       //17 Byte
{
	/* Í· */
	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
	uint8_t   CmdID;		//Ö¸Áî
	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
	/* Êı¾İ */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//¾àÀë
	uint8_t   centre_lock;		//ÊÇ·ñÃé×¼µ½ÁËÖĞ¼ä  0Ã»ÓĞ  1Ãé×¼µ½ÁË
	uint8_t	  identify_target;	//ÊÓÒ°ÄÚÊÇ·ñÓĞÄ¿±ê/ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê   0·ñ  1ÊÇ	
	uint8_t   identify_buff;	//´ò·ûÊ±ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê£¬1ÊÇ£¬2Ê¶±ğµ½ÇĞ»»ÁË×°¼×£¬0Ã»Ê¶±ğµ½
	
	uint8_t	  blank_b;			//Ô¤Áô
	uint8_t	  auto_too_close;   //Ä¿±ê¾àÀëÌ«½ü,ÊÓ¾õ·¢1£¬·ñÔò·¢0
	
	
	/* Î² */
	uint16_t  CRC16;     //Ö¡Î²     
	
}VisionRecvData_t;

//STM32·¢ËÍ,Ö±½Ó½«´ò°üºÃµÄÊı¾İÒ»¸ö×Ö½ÚÒ»¸ö×Ö½ÚµØ·¢ËÍ³öÈ¥
typedef struct
{

	
	/* Êı¾İ */
	float     pitch_angle;     //µ±Ç°½Ç¶È
	float     yaw_angle;       //µ±Ç°½Ç¶È                                              (»úĞµ?ÍÓÂİÒÇ?)¿)???????????
	float     distance;			   //¾àÀë
	uint8_t   lock_sentry;	 	 //ÊÇ·ñÔÚÌ§Í·Ê¶±ğÉÚ±ø
	uint8_t   base;				     //µõÉä
	
	uint8_t   blank_a;		//Ô¤Áô
	uint8_t	  blank_b;
	uint8_t	  blank_c;	
	
	/* Î² */
	uint16_t  CRC16;
	
}VisionSendData_t;


//¹ØÓÚÈçºÎĞ´ÈëCRCĞ£ÑéÖµ
//ÎÒÃÇ¿ÉÒÔÖ±½ÓÀûÓÃ¹Ù·½¸øµÄCRC´úÂë

//×¢Òâ,CRC8ºÍCRC16ËùÕ¼×Ö½Ú²»Ò»Ñù,8ÎªÒ»¸ö×Ö½Ú,16Îª2¸ö×Ö½Ú

//Ğ´Èë    CRC8 µ÷ÓÃ    Append_CRC8_Check_Sum( param1, param2)
//ÆäÖĞ param1´ú±íĞ´ºÃÁËÖ¡Í·Êı¾İµÄÊı×é(Ö¡Í·ºóµÄÊı¾İ»¹Ã»Ğ´Ã»ÓĞ¹ØÏµ),
//     param2´ú±íCRC8Ğ´ÈëºóÊı¾İ³¤¶È,ÎÒÃÇ¶¨ÒåµÄÊÇÍ·µÄ×îºóÒ»Î»,Ò²¾ÍÊÇ3

//Ğ´Èë    CRC16 µ÷ÓÃ   Append_CRC16_Check_Sum( param3, param4)
//ÆäÖĞ param3´ú±íĞ´ºÃÁË   Ö¡Í· + Êı¾İ  µÄÊı×é(¸úÉÏÃæÊÇÍ¬Ò»¸öÊı×é)
//     param4´ú±íCRC16Ğ´ÈëºóÊı¾İ³¤¶È,ÎÒÃÇ¶¨ÒåµÄÕû¸öÊı¾İ³¤¶ÈÊÇ22,ËùÒÔÊÇ22

/*----------------------------------------------------------*/

//ÃüÁîÂëID,ÓÃÀ´ÅĞ¶Ï½ÓÊÕµÄÊÇÊ²Ã´Êı¾İ


void Vision_Read_Data(uint8_t *ReadFormUart7);//ÊÓ¾õ¶ÁÈ¡Êı¾İ
void Vision_Send_Data( uint8_t CmdID );//ÊÓ¾õ·¢ËÍÊı¾İ
void Vision_Ctrl(void);//ÊÓ¾õ¿ØÖÆ
void Vision_Buff_Ctrl(void);//´ò·û¿ØÖÆ
void Vision_Auto_Attack_Ctrl(void);//×ÔÃé¿ØÖÆ
void Vision_Auto_Attack_Off(void);//¹Ø±Õ×ÔÃé


bool Get_Vision_distance(void); //ÊÓ¾õÅĞ¶ÏÔ¶½ü£¬ÓÃÓÚµ÷ÉäËÙÉäÆµ
void Vision_Get_Distance(float *distance); //»ñÈ¡¾àÀë

/********ÊÓ¾õ¸¨Öúº¯Êı*********/
uint8_t VISION_isColor(void);
uint8_t VISION_BuffType(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);
bool Vision_If_Armor(void);
void Vision_Clean_Ammor_Flag(void);


/*****ÊÓ¾õÆ«²î»ñÈ¡******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Buff_Error_Angle_Yaw(float *error);
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error);
void Vision_Buff_Error_Angle_Pitch(float *error);
void Vision_Base_Yaw_Pixel(float *error);

#endif
