#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "main.h"

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 700
//��������ӳټ��
#define GIMBAL_CONTROL_TIME_MS 2
void gimbal_task(void const *pvParameters);
extern fp32 Find_MIN_ANGLE(float set,float feed);

#endif


