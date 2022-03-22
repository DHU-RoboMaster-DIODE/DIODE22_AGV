#ifndef _PID_H
#define _PID_H
#include "main.h"

#define ABS(x)		(((x)>0)? (x): -(x))
#define pi 3.1415926535898
enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
extern uint8_t motionflag;
typedef struct _PID_Typedef
{
    float Kp;
    float Ki;
    float Kd;

    float target[3];					//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float feedback[3];					//����ֵ
    float err[3];							//���

    float pout;								//p���
    float iout;								//i���
    float dout;								//d���

    float pos_out;						//����λ��ʽ���
    float last_pos_out;					//�ϴ�λ��ʽ���
    float pos_out0;						//λ��ʽ�����Сֵ

    float delta_u;						//��������ֵ
    float last_delta_out;				//�ϴ�����ʽ���
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u

    float max_err;
    float deadband;						//err < deadband return

    uint32_t pid_mode;
    int MaxOutput;				//����޷�
    uint32_t IntegralLimit;		    //�����޷�
} PID_TypeDef;

extern PID_TypeDef PID_GM6020[4];	// ����GM6020 PID�ṹ��
extern PID_TypeDef PID_GM6020_speed[4];
extern PID_TypeDef PID_M2006[];		// ����M2006 PID�ṹ��
extern PID_TypeDef PID_M2006_ANGLE[];	// ����M2006 �Ƕ�PID�ṹ��
extern PID_TypeDef PID_M3508[7];	// ����M3508 PID�ṹ��
extern PID_TypeDef PID_M3508_ANGLE[];  // ����M3508 �Ƕ�PID�ṹ��
extern PID_TypeDef PID_M3508_Follow[];
extern PID_TypeDef PID_HEAT_PWM;

void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd
);
void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd);

float PID_Calculate(PID_TypeDef *pid, float target, float feedback);
void M3508_follow(PID_TypeDef *pid_6020,float target,int v[]);
void M3508_follow_2(PID_TypeDef *pid_6020,float target,float angle,float speed,float target2);
	
#endif



