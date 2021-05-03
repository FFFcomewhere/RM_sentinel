/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       vision.c/h
  * @brief      完成视觉数据读取与传输。
  * @note       主要有视觉数据读取，视觉数据传输，主要传输自瞄，打符，手瞄的指令。还有一些
	              视觉的辅助函数。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-11-16       XYS              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */


#include <vision.h>
#include "math.h"
#include "remote_control.h"
#include "crc.h"
#include "uart7.h"
#include "gimbal_task.h"
#include "kalman.h"
#include "chassis_task.h"
#include "judge.h"
#include "FreeRTOS.h"
#include "task.h"


extern RC_ctrl_t rc_ctrl;

//角度初始化补偿
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//根据距离补偿

VisionSendHeader_t    VisionSendHeader;        //帧头

VisionActData_t       VisionActData;          //行动模式结构体

VisionRecvData_t      VisionRecvData;        //接收数据结构体

VisionSendData_t      VisionSendData;        //发送数据结构体

uint8_t Attack_Color_Choose = ATTACK_NONE;  //默认不识别


//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;


//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send   = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.f;




/**
  * @brief  读取视觉信息
  * @param  uart7缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint8_t Vision_Time_Test[2] = {0};            //当前数据和上一次数据
uint8_t Vision_Ping = 0;                 //发送时间间隔
void Vision_Read_Data(uint8_t *ReadFormUart7)
{
	//判断帧头数据是否为0xA5
	if(ReadFormUart7[0] == VISION_SOF)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFormUart7, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFormUart7, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				memcpy( &VisionRecvData, ReadFormUart7, VISION_LEN_PACKED);	
				if(VisionRecvData.identify_target == TRUE)
				{
					Vision_Get_New_Data = TRUE;//标记视觉数据更新了
				}
				else
				{
					Vision_Get_New_Data = FALSE;

				}

				//帧计算
				Vision_Time_Test[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
//				if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//标记打符打中了，换装甲了
//				{
//					if(VisionRecvData.identify_buff == 2)//发2说明换装甲板了
//					{
//						Vision_Armor = TRUE;//发2换装甲
//					}
//				}
			}
		}
	}
}


/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲

  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
uint8_t vision_send_pack[50] = {0};//大于22就行
void Vision_Send_Data( uint8_t CmdID )
{
	int i;    //循环发送次数



	VisionSendData.SOF = VISION_SOF;
	VisionSendData.CmdID   = CmdID;
	VisionSendData.speed   = 30;
	VisionSendData.END    = VISION_WEI;

	

	
	memcpy( vision_send_pack + VISION_LEN_HEADER, &VisionSendData, VISION_LEN_DATA);
	
	//将打包好的数据通过串口移位发送到裁判系统
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		Uart7_SendChar( vision_send_pack[i] );
	}
	
	memset(vision_send_pack, 0, 50);
}

/**********************************视觉控制*****************************************/
/**
  * @brief  视觉总控制,指令更新
  * @param  void
  * @retval void
  * @attention  8位,只有键盘模式有视觉
  */
//void Vision_Ctrl(void)
//{
//	if(1)//SYSTEM_GetRemoteMode() == KEY)//键盘模式
//	{
//		if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//自动打符模式
//		{
//			VisionActData = VISION_BUFF;
//		}
//		else if (GIMBAL_IfManulHit() == TRUE)//手动模式
//		{
//			VisionActData = VISION_MANU;
//		}
//		else//默认朋友开挂,常威你还敢说你不会武功
//		{
//			VisionActData = VISION_AUTO;
//		}

//		switch(VisionActData)
//		{
//			/*- 打符 -*/
//			case VISION_BUFF:
//				Vision_Buff_Ctrl();
//			break;
//			
//			/*- 自瞄 -*/
//			case VISION_AUTO:
//				Vision_Auto_Attack_Ctrl();
//			break;
//			
//			/*- 手动 -*/
//			case VISION_MANU:
//				Vision_Auto_Attack_Off();
//			break;
//		}
//	}
//	else
//	{
//		Vision_Auto_Attack_Off();
//	}
//}


/**
  * @brief  自瞄控制
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* 确定敌方颜色 */
	//s1上识别红,下识别蓝,中间不识别
	if(IF_RC_SW1_UP)
	{
		Attack_Color_Choose = ATTACK_RED;
	}
	else if(IF_RC_SW1_DOWN)
	{
		Attack_Color_Choose = ATTACK_BLUE;
	}
	else
	{
		Attack_Color_Choose = ATTACK_NONE;
	}
	
	
	//向小电脑发送颜色识别指令
	if(Attack_Color_Choose == ATTACK_BLUE)
	{
		Vision_Send_Data( VISION_BLUE );
	}
	else if(Attack_Color_Choose == ATTACK_RED)
	{
		Vision_Send_Data( VISION_RED );
	}
	else if(Attack_Color_Choose == ATTACK_NONE)
	{
		Vision_Auto_Attack_Off();
	}
}


/**
  * @brief  关闭自瞄
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
	Vision_Send_Data( VISION_OFF );
}

/*******************************视觉误差获取*************************************/
/**
  * @brief  获取yaw误差像素(x轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点左边,正数表示在右边
  */
void Vision_Error_Yaw(float *error)
{
	if(VisionRecvData.yaw_angle != 0)
	{
		//输出为负时云台右移,为正时左移
		*error = -(VisionRecvData.yaw_angle - VISION_MID_YAW);
	}
	else
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差像素(y轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点上方,正数表示在下方
  */
void Vision_Error_Pitch(float *error)
{	
	if(VisionRecvData.pitch_angle != 0)
	{
		*error = VisionRecvData.pitch_angle - VISION_MID_PITCH;
	}
	else
	{
		*error = 0;
	}
}
/*--------------------------------自瞄专用的yaw,pitch轴误差角度获取-----------------------------*/
/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
			*error = ((VisionRecvData.yaw_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)* PI)/180;//因为pitch是机械模式,所以把欧拉角转换成弧度


	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;//距离很远时开放鼠标补偿
float vision_pitch_dist = 2;//固定距离,超过此距离开启距离补偿
float vision_pitch_dist_far = 4.4f;//超过此距离开放鼠标补偿
void Vision_Error_Angle_Pitch(float *error)
{	
					
	
	*error = ((VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)* PI)/180;//因为pitch是机械模式,所以把欧拉角转换成弧度

	if(VisionRecvData.pitch_angle == 0)
		*error = 0;
}

/*---------------------------------------------------------------打符专用 yaw，pitch轴误差角度获取---------------------------------------------------*/

/**
  * @brief  获取yaw误差角度，打符专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在云台
  */
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = -VisionRecvData.yaw_angle;//取负是为了对应yaw左转机械角增加
	if(VisionRecvData.yaw_angle == 0)//发零
		*error = 0;

}

/**
  * @brief  获取pitch误差角度，打符专用
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
void Vision_Buff_Error_Angle_Pitch(float *error)
{	
	//视觉上负下正,注意云台正负是抬头还是低头(上减下加)
	*error = VisionRecvData.pitch_angle;//pitch云台在右边时，向上转云台机械角减小
	if(VisionRecvData.pitch_angle == 0)
		*error = 0;

}
/*---------------------------------------------------------------吊射基地专用 yaw，pitch轴误差角度获取---------------------------------------------------*/
/**
  * @brief  获取yaw误差像素，桥头吊射基地专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正，摄像头在云台
  */
void Vision_Base_Yaw_Pixel(float *error)
{
	if(VisionRecvData.yaw_angle != 0)
		//输出为负时云台右移,为正时左移
		*error = -(VisionRecvData.yaw_angle - 640);

	else
		*error = 0;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/


bool Get_Vision_distance(void)     //TRUE 远    FALSE 近 
{
	if(VisionRecvData.distance>400)
		return TRUE;

	else
		return FALSE;
}

/**
  * @brief  获取距离
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance < 0)
		*distance = 0;
}

/*----------------------------------------------------------------------------------视觉辅助函数--------------------------------------------------------------------------------------*/

/**
  * @brief  自瞄颜色判断
  * @param  void
  * @retval
  * @attention  裁判系统左到右第二个灯红为识别红,绿为识别蓝,不识别红绿闪烁
  */
uint8_t VISION_isColor(void)
{
	return Attack_Color_Choose;
}



/**
  * @brief  判断发送的指令与视觉接收到的指令是否相同
  * @param  void
  * @retval TRUE指令一样    FALSE指令不一样
  * @attention  视觉收到什么指令,就发同样的指令回来
  */
bool VISION_IfCmdID_Identical(void)
{
	if (VisionRecvData.CmdID == VisionSendHeader.CmdID)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool Vision_If_Armor(void)
{
	return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Ammor_Flag(void)
{
	Vision_Armor = FALSE;
}

