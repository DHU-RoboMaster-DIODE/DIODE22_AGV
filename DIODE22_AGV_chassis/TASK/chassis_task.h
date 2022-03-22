#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "main.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 5
//chassis task control time 0.002s
//����������Ƽ�� 0.002s

void chassis_task(void const *pvParameters);
typedef struct
{
  float vx;                      //chassis set vertical speed,positive means forward,unit m/s.����ڵ����趨�ٶ� ǰ������ ǰΪ������λ cm/s
  float vy;                      //chassis set horizontal speed,positive means left,unit m/s.����ڵ����趨�ٶ� ���ҷ��� ��Ϊ������λ cm/s
  float wz;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� Ȧ/s
} chassis_speed_t;
extern uint16_t RC_FLAG;
#endif


