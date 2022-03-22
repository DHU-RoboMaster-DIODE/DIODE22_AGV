/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       led_task.c/h
  * @brief      led task,
  *             led灯任务
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
#include "outctl_task.h"
extern uint16_t RC_FLAG;
uint16_t time_crl;
uint8_t rc_flag=0,rc_out_time=20;  //num记录离线时间
bool_t rc_unable_flag=0;


//控制模式
eRemoteMode remoteMode = RC;

//系统状态
eSystemState systemState = SYSTEM_STARTING;

/**
  * @brief          失控保护任务，间隔 OUTCTL_TASK_TIME_MS 100ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void outctl_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(OUTCTL_TASK_INIT_TIME);

    while (1)
    {
			  time_crl++;
        rc_out_time++;
			  if(rc_out_time>20) rc_out_time=20;
				if(rc_out_time>10) rc_flag=0;
			  else rc_flag=1;
			  if(rc_unable_flag) rc_flag=0;
			
        //系统相对延时
        osDelay(OUTCTL_TASK_TIME_MS);
    }
}
void RC_unable(void){
	rc_unable_flag=1;
}
void RC_restart(void){
	rc_unable_flag=0;
}
/**
  * @brief  拨杆模式选择
  * @param  void
  * @retval void
  * @attention 键盘或鼠标,可在此自定义模式选择方式,1ms执行一次
  */
void SYSTEM_UpdateRemoteMode( void )
{ 

  if (rc.sw2==1  || rc.sw2==3)
	{
		remoteMode = RC;
	}
	else
	{
		remoteMode = PC;
	}
}


//返回控制模式
eRemoteMode SYSTEM_GetRemoteMode(void)
{
	return remoteMode;
}

//返回系统状态
eSystemState SYSTEM_GetSystemState(void)
{
	return systemState;
}


