#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "main.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 1200
//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s

void chassis_task(void const *pvParameters);

extern uint16_t RC_FLAG;

typedef struct
{

//  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
//  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
	
//  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
//  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.相对于云台设定速度 前进方向 前为正，单位 cm/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.相对于云台设定速度 左右方向 左为正，单位 cm/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 圈/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
  uint16_t chassis_init_angle_set;  //the set relative angle.设置相对云台初始前方角度             

  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
  fp32 setSpeed[4];

} chassis_move_t;

typedef struct
{
  fp32 vx;                      //chassis set vertical speed,positive means forward,unit m/s.相对于底盘设定速度 前进方向 前为正，单位 cm/s
  fp32 vy;                      //chassis set horizontal speed,positive means left,unit m/s.相对于底盘设定速度 左右方向 左为正，单位 cm/s
  fp32 wz;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 圈/s
} chassis_speed_t;

typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 0, //底盘跟随云盘行走
    CHASSIS_GYROSCOPE = 1,	   //小陀螺模式
    CHASSIS_NORMAL   = 2,      //底盘不跟随云台行走
    CHASSIS_ROSHAN   = 3,      //打符模式
    CHASSIS_SLOW     = 4,      //补弹低速模式
    CHASSIS_SZUPUP   = 5,      //爬坡模式
} eChassisAction;


extern eChassisAction actChassis;
extern chassis_speed_t absolute_chassis;
#endif


