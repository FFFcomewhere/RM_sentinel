#include "stm32f4xx.h"


#include "cap_task.h"
#include "start_task.h"
#include "can.h"
#include "Remote_Control.h"
#include "buzzer.h"
#include "led.h"
#include "CAN_receive.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

float Target_Power;
u16 Power;


//遥控器控制变量
extern RC_ctrl_t rc_ctrl;


void CAN1_Cap_Send(u16 temPower)
{	
  CanTxMsg TxMessage;
	
	TxMessage.StdId=0x210;
	TxMessage.IDE=CAN_ID_STD; //0x0000
	TxMessage.RTR=CAN_RTR_DATA; //0x0000
	TxMessage.DLC=8;
	
	TxMessage.Data[0]=(temPower >> 8);
	TxMessage.Data[1]=(temPower);
	
	CAN_Transmit(CAN1,&TxMessage);
}




void Cap_UpdateTarget_Power( int16_t power )
{
    Target_Power = power;			//获取目标功率
}




//主任务
void CapTask(void *pvParameters)
{
//    //空闲一段时间
    vTaskDelay(CAP_TASK_INIT_TIME);

			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)
				{
           Cap_Init();
				}
				else
				{
					if (SYSTEM_GetRemoteMode() == RC) //遥控模式
					{										
						Cap_RC_Ctrl();					
					}
					
				}
				vTaskDelay(TIME_STAMP_2MS);
			}
}

void Cap_Init()
{
	CAN1_Cap_Send(3500);
	led_green_off();
	led_red_off();
}


void Cap_RC_Ctrl()
{
		if (rc_ctrl.rc.s[0] == RC_SW_UP)//s1拨到上,CAN发送数据，将超电控制功率设置到38W
	 {
		 CAN1_Cap_Send(3800);
     GPIO_SetBits(GPIOG, GPIO_Pin_1);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);
	 }
	 
	 if(rc_ctrl.rc.s[0] == RC_SW_MID)//s1拨到中间,CAN发送数据，将超电控制功率设置到80W
	 {
		 CAN1_Cap_Send(8000);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
		 GPIO_SetBits(GPIOG, GPIO_Pin_4);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);

	 }	
   if(rc_ctrl.rc.s[0] == RC_SW_DOWN)//s1到下,CAN发送数据，将超电控制功率设置到100W
	 {
     CAN1_Cap_Send(10000);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
		 GPIO_SetBits(GPIOG, GPIO_Pin_7);
	 }	
}

//void Cap_RC_Ctrl()
//{
//		if (Chass_Realtime_RemainEnergy <= 40 )//当前底盘最大功率限制<50 
//	 {
//		 CAN1_Cap_Send(3800);
//     GPIO_SetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);
//	 }
//	 
//	 if(Chass_Realtime_RemainEnergy > 40 &&Chass_Realtime_RemainEnergy <= 60)//s1拨到中间,CAN发送数据，将超电控制功率设置到80W
//	 {
//		 CAN1_Cap_Send(5800);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);

//	 }	
//	 
//   if(Chass_Realtime_RemainEnergy >60 &&Chass_Realtime_RemainEnergy <= 80)//s1到下,CAN发送数据，将超电控制功率设置到100W
//	 {
//     CAN1_Cap_Send(7800);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_7);
//	 }

//   if(Chass_Realtime_RemainEnergy >80 &&Chass_Realtime_RemainEnergy < 120)//s1到下,CAN发送数据，将超电控制功率设置到100W
//	 {
//     CAN1_Cap_Send(11800);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_7);
//	 }	 
//}


