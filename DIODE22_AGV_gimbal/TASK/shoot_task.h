/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       shoot.c
 * @brief
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */

#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H
#include "main.h"

void shoot_task(void const *pvParameters);

typedef struct
{
		double speed;
		double speed_set;
		double trigger_speed_set;
		uint16_t block_time;
		uint16_t reverse_time;
}shoot_control_t;

typedef struct
{
	uint8_t friction;     //¼ÇÂ¼Ä¦²ÁÂÖ¿ª¹Ø
	uint8_t shoot_single;	
	uint8_t shoot_continuous;		
} shoot_state_t;

#define SHOOT_CONTROL_INIT_TIME  700
#define SHOOT_CONTROL_TIME_MS   10
#endif


