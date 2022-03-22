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

/* ���������� */
void Beep_On(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
/* ������Ϩ�� */
void Beep_Off(void)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
/* ����������һ��ʱ�� ms */
void Beep(uint16_t t)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,20000);
	HAL_Delay(t);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
