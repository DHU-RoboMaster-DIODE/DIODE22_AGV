/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       shoot.c
 * @brief
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */

#include "shoot_task.h"
#include "JudgeTask.h"

uint16_t time_shoot;
extern uint16_t RC_FLAG;
uint8_t ShotFlag;  //记录发射模式选择枪管
uint8_t shootflag2=1; //记录摩擦轮开关
uint16_t shoot_speed,dial_speed_set;   //发射弹速，发射频率
int16_t dial_speed;
shoot_control_t shoot_control;          //射击数据
shoot_state_t shoot_flag;
#define BLOCK_TIME 100
#define BLOCK_TRIGGER_SPEED 300.0f
#define REVERSE_TIME 100
/**
  * @brief          发射遥控器控制函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_rc_ctrl(void);
void shoot_pc_ctrl(void);
void shoot_control_loop(void);

/**
  * @brief          底盘任务，间隔 SHOOT_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(SHOOT_CONTROL_INIT_TIME);
    shoot_speed=4200;//摩擦轮速度
    dial_speed_set=1500;//拨弹速度4350
    PID_Init(&PID_M2006[0],POSITION_PID,16000,5000,8,0,0.1);		// 发射左拨盘电机 速度闭环
    for(int i=1; i<3; i++)	// 两个摩擦轮电机
    {
        PID_Init(&PID_M3508[i],POSITION_PID,16000,16000,2.5,0,0);// 0.01//5 0.1 0.1
    }	
    static portTickType lastWakeTime;
    lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        time_shoot++;
        if(rc_flag)
        {
					if(rc.sw2==1){
						 shoot_pc_ctrl();
					}
					else{
						 shoot_rc_ctrl();	
					}
					shoot_control_loop(); //底盘速度分解及pid计算
        }
        else    //若遥控器失控关闭清空电流值
        {
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
					  CAN_M2006[0].set_current =0;
        }
        CAN_Shoot_SendCurrent();
				memcpy(&rc_last,&rc,sizeof(rc_info_t));
        //系统绝对延时
        osDelayUntil(&lastWakeTime,SHOOT_CONTROL_TIME_MS);
    }
}

void shoot_pc_ctrl(void){

	if(rc.key[6] &&  shoot_flag.friction==0)//首次按q打开摩擦轮
		shoot_flag.friction=1;
	else if(rc.key[7] && shoot_flag.friction==1)//再按q关闭摩擦轮
		shoot_flag.friction=0;
  if(rc.mouse_press_l==1 && shoot_flag.shoot_continuous==0 && abs(CAN_M3508[1].speed)>500){
		  shoot_flag.shoot_continuous=1;
	}
	if 	(rc.mouse_press_l==0)shoot_flag.shoot_continuous=0;
}

void shoot_rc_ctrl(void) {
    if(rc.sw1!=1){shoot_flag.friction=1;}//按CTRL打开摩擦轮
	  else{shoot_flag.friction=0;}//按G关闭摩擦轮

    if(rc.sw1==2 && shoot_flag.shoot_continuous==0 && abs(CAN_M3508[1].speed)>500){
			shoot_flag.shoot_continuous=1;
	  }
	  if(rc.sw1==3) {
		  shoot_flag.shoot_continuous=0;
    }
}
void shoot_control_loop(void){
	if(game_robot_states.shooter_id1_17mm_cooling_limit-power_heat_data.shooter_id1_17mm_cooling_heat <17)
		shoot_flag.shoot_continuous = 0;//热量限制
	  //摩擦轮输出
		if(!shoot_flag.friction)
	  {
      laser_off();
      CAN_M3508[1].set_current =  PID_Calculate(&PID_M3508[1], 0, CAN_M3508[1].speed);
      CAN_M3508[2].set_current =  PID_Calculate(&PID_M3508[2], 0, CAN_M3508[2].speed);
	  }	
	  else
	  {
      laser_on();
      CAN_M3508[1].set_current =  PID_Calculate(&PID_M3508[1], shoot_speed, CAN_M3508[1].speed);
      CAN_M3508[2].set_current =  PID_Calculate(&PID_M3508[2], -shoot_speed, CAN_M3508[2].speed);
	  }
		if( shoot_control.block_time < BLOCK_TIME)
    {
        dial_speed = dial_speed_set;
    }
// 如果卡弹时间大于等于我设定的时间，电机反转。
    else if(shoot_control.block_time >= BLOCK_TIME)
    {
        dial_speed = -dial_speed_set;
    }
//	如果2006的速度小于设定的速度（即发生卡弹），
//	同时卡弹的时间小于我规定的时间，
//	则卡弹的时间计数继续增加，反转的时间清零。
    if((abs(CAN_M2006[0].speed) < BLOCK_TRIGGER_SPEED )
		&& shoot_control.block_time < BLOCK_TIME )
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
		if(shoot_control.block_time < BLOCK_TIME && abs(CAN_M2006[0].speed)>500)
		   shoot_control.block_time = 0;
// 如果卡弹的时间等于我规定的时间，同时电机反转的时间小于我设定的反转时间，则反转的时间计数继续增加。
    if(shoot_control.block_time >= BLOCK_TIME)
    {
        shoot_control.reverse_time++;
    }

		if(shoot_control.reverse_time>=REVERSE_TIME||(!shoot_flag.shoot_continuous) )//停止
		{
				shoot_control.block_time = 0;
		}
		if(shoot_flag.shoot_continuous){
			CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);	
		}	
		else
			CAN_M2006[0].set_current = 0;	
}
