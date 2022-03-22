/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       pid.c
 * @brief     
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "pid.h"
#include "math.h"
#include "bsp_can.h"
#include "user_lib.h"


float angle0=0,yaw0=0;
extern uint8_t spin_flag;
float temp_n;

PID_TypeDef PID_GM6020[6];	// ����GM6020 PID�ṹ��
PID_TypeDef PID_GM6020_speed[2];
PID_TypeDef PID_M2006[2];		// ����M2006 PID�ṹ��
PID_TypeDef PID_M2006_ANGLE[2];// ����M2006 �Ƕ�PID�ṹ��
PID_TypeDef PID_M3508[7];	// ����M3508 PID�ṹ�� ǰ�ĸ�Ϊ���̵�� ������ΪĦ���ֵ��
PID_TypeDef PID_M3508_ANGLE[7];// ����M2006 �Ƕ�PID�ṹ��
PID_TypeDef PID_M3508_Follow;// ����M3508���� PID�ṹ��
PID_TypeDef PID_HEAT_PWM;// ���������Ǽ��� PID�ṹ��

void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
/**
  * @brief  ��ʼ��PID�ṹ��
  * @param  PID�ṹ��ָ��
    @param  ����ϵ��
		@param  ����ϵ��
		@param  ΢��ϵ��
		@param  �������ֵ
		@param  ��������ֵ
  * @retval None
  */
void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd)
{
    pid->max_iout = intergral_limit;
    pid->max_out = maxout;
    pid->mode = mode;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

}

float PID_Calculate(PID_TypeDef *pid, float set, float ref)
{
	    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == POSITION_PID)	 //λ��ʽPID
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        abs_limit(&pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        abs_limit(&pid->out, pid->max_out);
    }
    else if (pid->mode ==DELTA_PID)	//����ʽPID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        abs_limit(&pid->out, pid->max_out);
    }
    return pid->out;
}

void M3508_follow(PID_TypeDef *pid_6020,float target,int v[])//���̸���
{
    float n;
	
//    if(CAN_GM6020[1].angle>4095)
//        angle=-8191+CAN_GM6020[1].angle;
//	else
//	float angle=CAN_GM6020[0].total_angle/8191*360;
	 
	PID_Calculate(pid_6020,target,CAN_GM6020[0].total_angle);
	n=-pid_6020->out;
	if(fabs(CAN_GM6020[0].total_angle-target)<300)
	{
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],0+v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],0+v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],0+v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],0+v[3],CAN_M3508[3].speed);
	}
	else
	{

		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n+v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n+v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n+v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n+v[3],CAN_M3508[3].speed);
	}
}



