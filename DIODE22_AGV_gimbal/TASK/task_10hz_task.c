/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       shoot.c
 * @brief      
 * @note       
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
 
#include "task_10hz_task.h"

int16_t warebuf[5]={0};
uint16_t time_10;
/**
  * @brief          10hz���񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void task_10hz_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(TASK_10HZ_TASK_INIT_TIME);
	
    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 
    while (1)
    {
			  time_10++;
			  if(game_robot_states.chassis_power_limit<45)  game_robot_states.chassis_power_limit=45;
			  SuperCapacitance(game_robot_states.chassis_power_limit);
			  //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PID_Calculate(&PID_HEAT_PWM,45,imu.temp)*100);
			  warebuf[0]=(int16_t)1;
			  warebuf[1]=(int16_t)(pitch_angle_set*100);
        warebuf[2]=(int16_t)(CAN_GM6020[1].set_angle_speed*100);
		    warebuf[3]=(int16_t)(CAN_GM6020[1].set_voltage*10);	
		    //warebuf[4]=(int16_t)(imu.wz*100);
        //VCAN_SendWare(&huart7,(uint8_t *)warebuf,sizeof(warebuf));	
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,TASK_10HZ_TASK_TIME_MS);
    }
}
