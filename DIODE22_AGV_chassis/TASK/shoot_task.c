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
uint16_t time_shoot;
extern uint16_t RC_FLAG;

uint8_t ShotFlag;  //��¼����ģʽѡ��ǹ��
uint8_t shootflag2=1; //��¼Ħ���ֿ���
uint16_t shoot_speed,dial_speed;   //���䵯�٣�����Ƶ��

/**
  * @brief          ����ң�������ƺ���
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void shoot_rc_ctrl(void);


/**
  * @brief          �������񣬼�� SHOOT_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(SHOOT_CONTROL_INIT_TIME);
	  shoot_speed=5000;//Ħ�����ٶ�
	  dial_speed=500;//�����ٶ�4350
    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 
    while (1)
    {
			  time_shoot++;
			  if(rc_flag<3)
			  {
          shoot_rc_ctrl();
			  }
				else    //��ң����ʧ�عر���յ���ֵ
			  {
            CAN_M3508[4].set_current=0;
            CAN_M3508[5].set_current=0;					
			  }
        //CAN_Shoot_SendCurrent();
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,SHOOT_CONTROL_TIME_MS);
    }
}

void shoot_rc_ctrl(void){
if(rc.sw2!=1) {
# if RACEFLAG
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], -shoot_speed, CAN_M3508[5].speed);
		CAN_M3508[6].set_current =  PID_Calculate(&PID_M3508[6], shoot_speed, CAN_M3508[6].speed);
    CAN_M3508[7].set_current =  PID_Calculate(&PID_M3508[7], -shoot_speed,CAN_M3508[7].speed);
# else

	if(rc.sw1!=1){shootflag2=0;}//��CTRL��Ħ����
	else{shootflag2=1;}//��G�ر�Ħ����
	
	if(shootflag2)
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
	}
	
	else
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], shoot_speed, CAN_M3508[5].speed);
	}
#endif
		/*************************��f�л�����*******************************/

		ShotFlag = 0;
//	ShotFlag = 2;
	if(rc.sw1==2){
		if(ShotFlag==0)
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],dial_speed, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);
		}
		else if(ShotFlag==1)
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);

		}
		else
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);
		}
	}
	else{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);

	}
}
}
