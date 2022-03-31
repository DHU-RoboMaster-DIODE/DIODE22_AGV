/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  */
#include "chassis_task.h"
#include "rc_control.h"
#include "INS_task.h"
#include "arm_math.h"

uint16_t time_chassis;

#define WHEEL_RADIUS  15//轮子半径cm
#define LENGTH_A  40  //轮距
#define LENGTH_B  36  //轴距
#define CHASSIS_DECELE_RATIO  19 //减速比

//底盘运动 
uint16_t classis_speed,spin_speed;   //底盘基本速度,小陀螺速度根据裁判系统改变
uint8_t SpinStartFlag,IsSpinFlag;//小陀螺标志
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;

eChassisAction actChassis=CHASSIS_FOLLOW_GIMBAL;   //默认底盘跟随云台行走
eChassisAction actChassis_last = CHASSIS_FOLLOW_GIMBAL;
//底盘运动数据
chassis_move_t chassis_move;
chassis_speed_t absolute_chassis;

uint8_t flag_upup;
/**
  * @brief          底盘控制函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_rc_ctrl(void);
void chassis_pc_ctrl(void);
void chassis_control_loop(void); 
/**
  * @brief          寻找最小角度（编码器版本）用于标准化-pi~pi
  * @param[in]      pvParameters: 设置角度和
  * @retval         none
  */	
fp32 Find_MIN_ANGLE_Enconder(float set,float feed);

/**
  * @brief          解算为麦轮四个电机的速度值
  * @param[in]      pvParameters: 底盘坐标速度结构体，输出结构体
  * @retval         none
  */	
void mecanum_calc(chassis_speed_t *speed,chassis_move_t *out);
	
float logistic(int* x);

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(CHASSIS_TASK_INIT_TIME);
		classis_speed=1000;//移动速度
		spin_speed=1000;//小陀螺速度
    static portTickType lastWakeTime;  
    for(int i=0; i<4; i++)	// 四个底盘电机
    {
        PID_Init(&PID_M3508[i],DELTA_PID,16000,3000,3,0.02,0,0,0);//5,0.01
    }
		PID_Init(&PID_M3508_Follow,POSITION_PID,1000,1000,3,0,5,0,0);
		chassis_move.chassis_init_angle_set=7066;
    while (1)
    {
			  lastWakeTime = xTaskGetTickCount(); 
				time_chassis++;
				if(rc_flag)
				{
				  switch(rc.sw2){
						case 1:
							chassis_pc_ctrl();
							break;
						case 3:
							actChassis=CHASSIS_NORMAL;	//底盘跟随云台
							chassis_rc_ctrl();
						  break;
						case 2:
							actChassis=CHASSIS_GYROSCOPE;	//底盘小陀螺
							chassis_rc_ctrl();
						  break;
						default:
              break;
					}
          chassis_control_loop(); //底盘速度分解及pid计算
				}
				else    //若遥控器失控关闭清空电流值
				{

				}
				//CAN通信向底盘四个电机发送电流值
        CAN_TA();
        //系统绝对延时
        osDelayUntil(&lastWakeTime,CHASSIS_CONTROL_TIME_MS);
    }
}


void chassis_rc_ctrl(void)
{  
	  flag_upup=0;
    switch(actChassis)
    {
    case CHASSIS_FOLLOW_GIMBAL://跟随云台
        chassis_move.vx_set =(fp32)rc.ch3/6.0f; //前后计算
        chassis_move.vy_set =(fp32)rc.ch2/6.0f; //左右计算
        chassis_move.wz_set=rc.ch0/800.0f+PID_Calculate(&PID_M3508_Follow,
		Find_MIN_ANGLE_Enconder(chassis_move.chassis_init_angle_set,CAN_GM6020[0].angle),0)/1000.0f;//PID使底盘跟随云台速度
        break;
    case CHASSIS_NORMAL://不跟随云台
        chassis_move.vx_set=(fp32)rc.ch3/6.0f;
        chassis_move.vy_set=(fp32)rc.ch2/6.0f;
        chassis_move.wz_set=0;
        break;
    case CHASSIS_GYROSCOPE:		//小陀螺模式
        chassis_move.vx_set=(fp32)rc.ch3/6.0f;
        chassis_move.vy_set=(fp32)rc.ch2/6.0f;
        chassis_move.wz_set=-3;
        break;
		case CHASSIS_SLOW:		//驻留模式
        chassis_move.vx_set=0;
        chassis_move.vy_set=0;
        chassis_move.wz_set=0;
        break;
    default:
        break;
    }
}
//pc控制全局变量，便于调试后期改为静态结构体
	float speedx,speedy;
  float speedxx,speedyy;
  int x1,x2,y1,y2;

void chassis_pc_ctrl(void)
{  
	  if(rc.key[5]) 
			actChassis=CHASSIS_GYROSCOPE;
		else 
			actChassis=CHASSIS_NORMAL;		
	  if(rc.key[4])flag_upup=1;
		else flag_upup=0;
		if(rc.key[0]) speedx=logistic(&x1);  //w
		if(rc.key[1]) speedx=-logistic(&x2); //s
		if(rc.key[2]) speedy=-logistic(&y1); //a
		if(rc.key[3]) speedy=logistic(&y2);  //d
		if(speedx>=660) speedx=660;
		if(speedx<=-660) speedx=-660;
		if(speedy>=660) speedy=660;
		if(speedy<=-660) speedy=-660;
		if(!(rc.key[0]||rc.key[1])) {
			speedx=0;
			x1=x2=0;
		}
		if(!(rc.key[2]||rc.key[3])) {
			speedy=0;
			y1=y2=0;
		}
    switch(actChassis)
    {
    case CHASSIS_FOLLOW_GIMBAL://跟随云台
        chassis_move.vx_set =(fp32)speedx/6.0f; //前后计算
        chassis_move.vy_set =(fp32)speedy/6.0f; //左右计算
        chassis_move.wz_set=rc.mouse_x/800.0f+PID_Calculate(&PID_M3508_Follow,
		Find_MIN_ANGLE_Enconder(chassis_move.chassis_init_angle_set,CAN_GM6020[0].angle),0)/1000.0f;//PID使底盘跟随云台速度
        break;
    case CHASSIS_NORMAL://不跟随云台
        chassis_move.vx_set=(fp32)speedx/6.0f;
        chassis_move.vy_set=(fp32)speedy/6.0f;
        chassis_move.wz_set=0;
        break;
    case CHASSIS_GYROSCOPE:		//小陀螺模式
        chassis_move.vx_set=(fp32)speedx/6.0f;
        chassis_move.vy_set=(fp32)speedy/6.0f;
        chassis_move.wz_set=-3;
        break;
		case CHASSIS_SLOW:		//驻留模式
        chassis_move.vx_set=0;
        chassis_move.vy_set=0;
        chassis_move.wz_set=0;
        break;
    default:
        break;
    }
}

void Gimbal2Chassis(chassis_move_t *gimbal, chassis_speed_t *chassis,float angle)
{
    float angle_hd = angle * PI / 180;
    chassis->wz = gimbal->wz_set;
    chassis->vx = gimbal->vx_set * cos(angle_hd) + gimbal->vy_set * sin(angle_hd);
    chassis->vy = gimbal->vx_set * sin(angle_hd) - gimbal->vy_set * cos(angle_hd);
	
}

void chassis_control_loop(void){
	  Gimbal2Chassis(&chassis_move,&absolute_chassis,
	  ((fp32)(CAN_GM6020[0].angle-chassis_move.chassis_init_angle_set))*0.043945f);			
}



void mecanum_calc(chassis_speed_t *speed,chassis_move_t *out)
{
	
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 1/ (WHEEL_RADIUS * 3.14159f) * CHASSIS_DECELE_RATIO*60;
	
    out->setSpeed[0] = ( speed->vx - speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[1] = (-speed->vx - speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[2] = ( speed->vx + speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    out->setSpeed[3] = (-speed->vx + speed->vy + speed->wz * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

}


fp32 Find_MIN_ANGLE_Enconder(float set,float feed)
{
    fp32 temp = set - feed;
    if(temp >=4096)
        temp -= 8192;
    else if(temp < -4096)
        temp += 8192;
		if(temp>2048)  temp-=4096;
		else if(temp<-2048)  temp+=4096;
    return temp;
}
float logistic(int* x)
{
	float speed;
	(*x)++;
	if((*x)>=60) (*x)=60;
	speed=300/(1+exp(-(0.3*(*x)-10)));
	return speed;
}
