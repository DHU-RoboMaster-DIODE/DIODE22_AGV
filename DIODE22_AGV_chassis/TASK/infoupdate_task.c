/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             ���̿�������
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
#include "infoupdate_task.h"
#include "gimbal.h"
#include "kalman.h"
#include "chassis_task.h"
#include "bsp_usart.h"

int time_info;

uint16_t dialspd=2000,Startflag=0;
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void info_update_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(INFO_UPDATE_INIT_TIME);

    while (1)
    {
			
      time_info++;
				if(rc_flag<3)
				{
					  if(!Startflag)
							dialspd=2000,
							CAN_GM6020[0].total_angle=CAN_GM6020[0].angle,Startflag=1;
				}
      //ϵͳ������ʱ
      osDelay(10);
    }
}

