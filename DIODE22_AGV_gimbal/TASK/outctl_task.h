/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis.c/h
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
#ifndef __OUTCTL_TASK_H
#define __OUTCTL_TASK_H
#include "main.h"



#define OUTCTL_TASK_INIT_TIME   500
#define OUTCTL_TASK_TIME_MS     20



typedef enum
{
    RC   = 0,  
    PC  = 1,  

} eRemoteMode;  // 遥控方式


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;
extern eRemoteMode SYSTEM_GetRemoteMode(void);
extern void outctl_task(void const *pvParameters);
extern void RC_unable(void);
extern void RC_restart(void);

#endif


