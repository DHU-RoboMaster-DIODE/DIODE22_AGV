#ifndef __BSP_USART
#define __BSP_USART
#include "main.h"

#define JUDGE_UART		huart6   //C��˿ӡ��Ӧuart1
#define VISION_UART		huart1   //C��˿ӡ��Ӧuart2

extern uint8_t aRxBuffer[];		// ����2�Ľ��ջ����������ڳ�ʼ����Ȼ��ֵһ��һ��������USART2_RX_BUF��
extern uint8_t USART2_RX_BUF[];	//������200�ֽ�
extern uint16_t USART_RX_STA;
extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void UART_PutBuff (UART_HandleTypeDef *huart,uint8_t *buff, uint32_t len);
void VCAN_SendWare(UART_HandleTypeDef *huart,void *wareaddr, uint32_t waresize);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
extern void uart_receive_handler(UART_HandleTypeDef *huart);

#endif
