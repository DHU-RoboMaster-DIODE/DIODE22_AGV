/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_usart.c
 * @brief      
 * @note       
*
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_usart.h"

// ɽ�⴮�ڵ������ֵײ㺯��
void UART_PutBuff (UART_HandleTypeDef *huart,uint8_t *buff, uint32_t len)
{
    while(len)
    {
        HAL_UART_Transmit(huart,buff,1,1);
        buff++;
        len--;
    }
}
void VCAN_SendWare(UART_HandleTypeDef *huart,void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3	// ���ڵ������� ����ʾ����������
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    UART_PutBuff(huart,cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    UART_PutBuff(huart,(uint8_t *)wareaddr, waresize);    //��������
    UART_PutBuff(huart,cmdr, sizeof(cmdr));    //���ͺ�����
}

// �ض��򴮿�2��printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
uint8_t aRxBuffer[1];		// ����2�Ľ��ջ����������ڳ�ʼ����Ȼ��ֵһ��һ��������USART2_RX_BUF��
uint8_t USART2_RX_BUF[200];	//������200�ֽ�
uint16_t USART_RX_STA=0;       //����״̬���
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR & (__FLAG__)) == (__FLAG__))
#define __HAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->SR = ~(__FLAG__))
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        uint32_t isrflags   = READ_REG(huart->Instance->SR);//�ֲ����н��������Ҫ�ȶ�SR
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//PE���־���ڶ�����DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);//���־
        }
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//FE���־���ڶ�����DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
        }
        
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//NE���־���ڶ�����DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
        }        
        
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
        {
                READ_REG(huart->Instance->CR1);//ORE���־���ڶ�����CR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
        }        
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


//	Beep(200);
//	printf("%s","11   ");
    if (huart == &huart1)
    {
//		OLED_Printf(1,1,"12");
//		OLED_RefreshGram();
		VisionUartRxCpltCallback();
//		printf("%s","33   ");
//		VisionUartRxCpltCallback_2();
	
    }
	if(huart == &huart6)
	{	//
//		LED_R_On();
//		HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer, 1);
//		HAL_UART_Transmit_IT(&huart6, "12121", 10);
////		printf("%d",1);
//		BT_SendWave();
	}
    if (huart == &JUDGE_UART)
    {
        judgeUartRxCpltCallback();
    }
	HAL_UART_ErrorCallback(huart);
}
