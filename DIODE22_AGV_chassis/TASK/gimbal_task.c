/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       gimbal_task.c/h
  * @brief      云台控制任务
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
#include "gimbal_task.h"
#include "pid.h"
#include "bsp_can.h"

#define pitch_angle_min	-20
#define pitch_angle_max	27

extern uint16_t RC_FLAG;
uint16_t time_gimbal;
float Kp=70,Ki=0,Kd=20,Kps=30,Kis=0,Kds=50;




extern float GM6020speed;
extern float GM6020speed_pit;
extern double imu_last_anglesum;
double last_imu_yaw;
//extKalman_t imu_speed;
//extKalman_t pit_data_speed;

void angle_sum(void);
void gimbal_rc_ctrl(void);

/**
  * @brief          云台任务，绝对延时 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void gimbal_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(GIMBAL_TASK_INIT_TIME);

    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 

    imu.angle_sum=imu.yaw;
    while (1)
    { 
       //IMU_Get();
       time_gimbal++;
			 if(rc_flag<3)
			 {
				  
					  gimbal_rc_ctrl();  
			 }
				else    //若遥控器失控关闭清空电流值
			 {
        		CAN_GM6020[0].set_voltage=0;
            CAN_GM6020[1].set_voltage=0;
			 }
       //CAN_Gimbal_SendVoltage();
       //系统绝对延时
       osDelayUntil(&lastWakeTime,GIMBAL_CONTROL_TIME_MS);
    }
}

void gimbal_rc_ctrl(void)
{
	switch(rc.sw2){
		case 3:		
		break;
		case 2:

		  break;		
		
  }
}

