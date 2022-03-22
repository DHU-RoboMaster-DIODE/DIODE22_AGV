#ifndef __RC_CONTROL_H
#define __RC_CONTROL_H
#include "main.h"



extern uint8_t rc_flag;

void RC_PC(float classis_speed,float spin_speed,float dial_speed,float shoot_speed);
void Level_Up_System(void);
void RC_Shoot(float fri_speed,float dial_speed);
void RC_Vision_aiming(void);
void RC_Singleshot(float fri_speed,uint8_t pattern);

#endif

