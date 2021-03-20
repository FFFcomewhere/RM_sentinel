/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "User_Task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"

#include "Detect_Task.h"
#include "INS_Task.h"

#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

angle_measure_t angle_measure;

//姿态角 单位度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
fp32 angle_speed[3] = {0.0f, 0.0f, 0.0f};
fp32 angle_accel[3] = {0.0f, 0.0f, 0.0f};
float yaw;
float pit;
void UserTask(void *pvParameters)
{

    const volatile fp32 *angle;
	  const volatile fp32 *speed;
	  const volatile fp32 *accel;	
    //获取姿态角指针
    angle = get_INS_angle_point();
	  speed = get_MPU6500_Gyro_Data_Point();
	  accel = get_MPU6500_Accel_Data_Point();
    while (1)
    {
        yaw+=0.0007f;
			  pit+=0.0005f;
        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;
			
        angle_measure.angleMpuYaw=(angle_degree[0]+yaw)*20;
			  angle_measure.angleMpuPitch=angle_degree[1]*20;
			
			  angle_speed[0] = (*(speed + INS_GYRO_X_ADDRESS_OFFSET));    //roll角速度
		  	angle_speed[1] = (*(speed + INS_GYRO_Y_ADDRESS_OFFSET));     //pitch角速度
		  	angle_speed[2] = (*(speed + INS_GYRO_Z_ADDRESS_OFFSET));    //yaw角速度

			  angle_measure.palstanceMpuPitch=angle_speed[1];
			  angle_measure.palstanceMpuYaw=angle_speed[2];
			
        vTaskDelay(2);
			
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

