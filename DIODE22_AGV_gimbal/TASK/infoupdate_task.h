/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis.c/h
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
#ifndef __INFOUPDATE_TASK_H
#define __INFOUPDATE_TASK_H
#include "main.h"

#define INFO_UPDATE_INIT_TIME  100
#define INFO_UPDATE_TIME_MS    2

#define USART_RX_BUF_LENGHT     128
#define REFEREE_FIFO_BUF_LENGTH 256

void info_update_task(void const *pvParameters);

#endif


