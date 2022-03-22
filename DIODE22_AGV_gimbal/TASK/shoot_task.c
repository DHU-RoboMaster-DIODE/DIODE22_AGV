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
uint8_t ShotFlag;  //��¼����ģʽѡ��ǹ��
uint8_t shootflag2=1; //��¼Ħ���ֿ���
uint16_t shoot_speed,dial_speed_set;   //���䵯�٣�����Ƶ��
int16_t dial_speed;
shoot_control_t shoot_control;          //�������
shoot_state_t shoot_flag;
#define BLOCK_TIME 100
#define BLOCK_TRIGGER_SPEED 300.0f
#define REVERSE_TIME 100
/**
  * @brief          ����ң�������ƺ���
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void shoot_rc_ctrl(void);
void shoot_pc_ctrl(void);
void shoot_control_loop(void);

/**
  * @brief          �������񣬼�� SHOOT_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(SHOOT_CONTROL_INIT_TIME);
    shoot_speed=4200;//Ħ�����ٶ�
    dial_speed_set=1500;//�����ٶ�4350
    PID_Init(&PID_M2006[0],POSITION_PID,16000,5000,8,0,0.1);		// �������̵�� �ٶȱջ�
    for(int i=1; i<3; i++)	// ����Ħ���ֵ��
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
					shoot_control_loop(); //�����ٶȷֽ⼰pid����
        }
        else    //��ң����ʧ�عر���յ���ֵ
        {
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
					  CAN_M2006[0].set_current =0;
        }
        CAN_Shoot_SendCurrent();
				memcpy(&rc_last,&rc,sizeof(rc_info_t));
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,SHOOT_CONTROL_TIME_MS);
    }
}

void shoot_pc_ctrl(void){

	if(rc.key[6] &&  shoot_flag.friction==0)//�״ΰ�q��Ħ����
		shoot_flag.friction=1;
	else if(rc.key[7] && shoot_flag.friction==1)//�ٰ�q�ر�Ħ����
		shoot_flag.friction=0;
  if(rc.mouse_press_l==1 && shoot_flag.shoot_continuous==0 && abs(CAN_M3508[1].speed)>500){
		  shoot_flag.shoot_continuous=1;
	}
	if 	(rc.mouse_press_l==0)shoot_flag.shoot_continuous=0;
}

void shoot_rc_ctrl(void) {
    if(rc.sw1!=1){shoot_flag.friction=1;}//��CTRL��Ħ����
	  else{shoot_flag.friction=0;}//��G�ر�Ħ����

    if(rc.sw1==2 && shoot_flag.shoot_continuous==0 && abs(CAN_M3508[1].speed)>500){
			shoot_flag.shoot_continuous=1;
	  }
	  if(rc.sw1==3) {
		  shoot_flag.shoot_continuous=0;
    }
}
void shoot_control_loop(void){
	if(game_robot_states.shooter_id1_17mm_cooling_limit-power_heat_data.shooter_id1_17mm_cooling_heat <17)
		shoot_flag.shoot_continuous = 0;//��������
	  //Ħ�������
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
// �������ʱ����ڵ������趨��ʱ�䣬�����ת��
    else if(shoot_control.block_time >= BLOCK_TIME)
    {
        dial_speed = -dial_speed_set;
    }
//	���2006���ٶ�С���趨���ٶȣ���������������
//	ͬʱ������ʱ��С���ҹ涨��ʱ�䣬
//	�򿨵���ʱ������������ӣ���ת��ʱ�����㡣
    if((abs(CAN_M2006[0].speed) < BLOCK_TRIGGER_SPEED )
		&& shoot_control.block_time < BLOCK_TIME )
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
		if(shoot_control.block_time < BLOCK_TIME && abs(CAN_M2006[0].speed)>500)
		   shoot_control.block_time = 0;
// ���������ʱ������ҹ涨��ʱ�䣬ͬʱ�����ת��ʱ��С�����趨�ķ�תʱ�䣬��ת��ʱ������������ӡ�
    if(shoot_control.block_time >= BLOCK_TIME)
    {
        shoot_control.reverse_time++;
    }

		if(shoot_control.reverse_time>=REVERSE_TIME||(!shoot_flag.shoot_continuous) )//ֹͣ
		{
				shoot_control.block_time = 0;
		}
		if(shoot_flag.shoot_continuous){
			CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);	
		}	
		else
			CAN_M2006[0].set_current = 0;	
}
