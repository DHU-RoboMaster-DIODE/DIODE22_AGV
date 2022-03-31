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
    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;  //设定值
    fp32 fdb;  //反馈值

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
    float I_Separation; //积分分离阈值，当偏差大于阈值时，消除积分仅用PD控制
    float gama;			//微分先行滤波系数
    float lastdout;		//上一次微分输出
} PID_TypeDef;

extern PID_TypeDef PID_GM6020[6];	// 定义GM6020 PID结构体
extern PID_TypeDef PID_GM6020_speed[2];
extern PID_TypeDef PID_M2006[];		// 定义M2006 PID结构体
extern PID_TypeDef PID_M2006_ANGLE[];	// 定义M2006 角度PID结构体
extern PID_TypeDef PID_M3508[7];	// 定义M3508 PID结构体
extern PID_TypeDef PID_M3508_ANGLE[];  // 定义M3508 角度PID结构体
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



