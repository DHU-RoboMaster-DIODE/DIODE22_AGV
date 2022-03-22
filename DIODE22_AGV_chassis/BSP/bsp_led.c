/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       led.c
 * @brief      
 * @note       PE11 PF14 
 * @Version    V1.0.0
 * @Date       2021.5     
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_led.h"

/* �����̵� */
//void LED_G_On(void)
//{
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
//}
/* ������� */
void LED_R_Off(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}
/* Ϩ���̵� */
//void LED_G_Off(void)
//{
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//}
/* Ϩ���� */
void LED_R_On(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}
/* ��ת�̵� */
//void LED_G_Toggle(void)
//{
//	HAL_GPIO_TogglePin (LED_G_GPIO_Port, LED_G_Pin);
//}
/* ��ת��� */
void LED_R_Toggle(void)
{
	HAL_GPIO_TogglePin (LED_R_GPIO_Port, LED_R_Pin);
}
/* ����̵���˸ */
//void LED_RG_Twinkle(uint32_t t)
//{
//	LED_G_On();
//	LED_R_On();
//	HAL_Delay(t);
//	LED_G_Off();
//	LED_R_Off();
//	HAL_Delay(t);
//}

void LASER_On(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}
// �رռ���
void LASER_Off(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}
