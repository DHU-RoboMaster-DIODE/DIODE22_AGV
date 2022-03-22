#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "main.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 1200
//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s

void chassis_task(void const *pvParameters);

extern uint16_t RC_FLAG;

typedef struct
{

//  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
//  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
	
//  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
//  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�������̨�趨�ٶ� ǰ������ ǰΪ������λ cm/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�������̨�趨�ٶ� ���ҷ��� ��Ϊ������λ cm/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ Ȧ/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  uint16_t chassis_init_angle_set;  //the set relative angle.���������̨��ʼǰ���Ƕ�             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 setSpeed[4];

} chassis_move_t;

typedef struct
{
  fp32 vx;                      //chassis set vertical speed,positive means forward,unit m/s.����ڵ����趨�ٶ� ǰ������ ǰΪ������λ cm/s
  fp32 vy;                      //chassis set horizontal speed,positive means left,unit m/s.����ڵ����趨�ٶ� ���ҷ��� ��Ϊ������λ cm/s
  fp32 wz;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� Ȧ/s
} chassis_speed_t;

typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 0, //���̸�����������
    CHASSIS_GYROSCOPE = 1,	   //С����ģʽ
    CHASSIS_NORMAL   = 2,      //���̲�������̨����
    CHASSIS_ROSHAN   = 3,      //���ģʽ
    CHASSIS_SLOW     = 4,      //��������ģʽ
    CHASSIS_SZUPUP   = 5,      //����ģʽ
} eChassisAction;


extern eChassisAction actChassis;
extern chassis_speed_t absolute_chassis;
#endif


