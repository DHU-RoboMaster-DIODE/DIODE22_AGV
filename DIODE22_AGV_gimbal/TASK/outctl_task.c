/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       led_task.c/h
  * @brief      led task,
  *             led������
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
uint8_t rc_flag=0,rc_out_time=20;  //num��¼����ʱ��
bool_t rc_unable_flag=0;


//����ģʽ
eRemoteMode remoteMode = RC;

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;

/**
  * @brief          ʧ�ر������񣬼�� OUTCTL_TASK_TIME_MS 100ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void outctl_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(OUTCTL_TASK_INIT_TIME);

    while (1)
    {
			  time_crl++;
        rc_out_time++;
			  if(rc_out_time>20) rc_out_time=20;
				if(rc_out_time>10) rc_flag=0;
			  else rc_flag=1;
			  if(rc_unable_flag) rc_flag=0;
			
        //ϵͳ�����ʱ
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
  * @brief  ����ģʽѡ��
  * @param  void
  * @retval void
  * @attention ���̻����,���ڴ��Զ���ģʽѡ��ʽ,1msִ��һ��
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


//���ؿ���ģʽ
eRemoteMode SYSTEM_GetRemoteMode(void)
{
	return remoteMode;
}

//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState(void)
{
	return systemState;
}


