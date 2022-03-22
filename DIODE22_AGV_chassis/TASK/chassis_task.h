#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "main.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 5
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s

void chassis_task(void const *pvParameters);
typedef struct
{
  float vx;                      //chassis set vertical speed,positive means forward,unit m/s.相对于底盘设定速度 前进方向 前为正，单位 cm/s
  float vy;                      //chassis set horizontal speed,positive means left,unit m/s.相对于底盘设定速度 左右方向 左为正，单位 cm/s
  float wz;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 圈/s
} chassis_speed_t;
extern uint16_t RC_FLAG;
#endif


