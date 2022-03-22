#ifndef __BSP_BEEP_H
#define __BSP_BEEP_H
#include "main.h"


void Beep_On(void);
void Beep_Off(void);
void Beep(uint16_t t);
void buzzer_on(uint16_t psc, uint16_t pwm);
extern void laser_on(void);
extern void laser_off(void);
#endif


