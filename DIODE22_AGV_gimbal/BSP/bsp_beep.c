/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       beep.c
 * @brief      
 * @note       TIM12 CH1 (PH6)
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_beep.h"

/* ·äÃùÆ÷Ãù½Ð */
void Beep_On(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,15000);
}
/* ·äÃùÆ÷Ï¨»ð */
void Beep_Off(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,0);
}
/* ·äÃùÆ÷Ãù½ÐÒ»¶ÎÊ±¼ä ms */
void Beep(uint16_t t)
{

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,15000);
	HAL_Delay(t);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
/* ·äÃùÆ÷Ãù½ÐµÄÉèÖÃÆµÂÊºÍÇ¿¶È*/
void buzzer_on(uint16_t psc, uint16_t pwm)
{
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}


extern TIM_HandleTypeDef htim3;
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
