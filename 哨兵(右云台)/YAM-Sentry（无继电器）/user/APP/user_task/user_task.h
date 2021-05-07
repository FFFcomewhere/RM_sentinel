#ifndef USER_TASK_H
#define USER_TASK_H

extern void UserTask(void *pvParameters);

typedef struct
{
	float angleMpuPitch;
	float angleMpuYaw;
	float palstanceMpuYaw;
	float palstanceMpuPitch;
} angle_measure_t;
#endif
