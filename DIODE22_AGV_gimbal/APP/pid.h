#ifndef _PID_H
#define _PID_H
#include "main.h"

#define ABS(x)		(((x)>0)? (x): -(x))
#define pi 3.1415926535898f
enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
typedef struct _PID_Typedef
{
	  uint8_t mode;
    float Kp;
    float Ki;
    float Kd;
    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;  //�趨ֵ
    fp32 fdb;  //����ֵ

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
    float I_Separation; //���ַ�����ֵ����ƫ�������ֵʱ���������ֽ���PD����
    float gama;			//΢�������˲�ϵ��
    float lastdout;		//��һ��΢�����
} PID_TypeDef;

extern PID_TypeDef PID_GM6020[6];	// ����GM6020 PID�ṹ��
extern PID_TypeDef PID_GM6020_speed[2];
extern PID_TypeDef PID_M2006[];		// ����M2006 PID�ṹ��
extern PID_TypeDef PID_M2006_ANGLE[];	// ����M2006 �Ƕ�PID�ṹ��
extern PID_TypeDef PID_M3508[7];	// ����M3508 PID�ṹ��
extern PID_TypeDef PID_M3508_ANGLE[];  // ����M3508 �Ƕ�PID�ṹ��
extern PID_TypeDef PID_M3508_Follow;
extern PID_TypeDef PID_HEAT_PWM;

void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd,
	  float I_Separation,float gama
);
void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd);

float PID_Calculate(PID_TypeDef *pid, float target, float feedback);

	
#endif



