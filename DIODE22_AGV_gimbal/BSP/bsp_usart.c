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

extern uint8_t   dbus_buf[];

#define JUDGE_MAX_LEN     (50)
#define JUDGE_BUFLEN      (18)

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

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

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    /* Return the number of remaining data units for DMAy Streamx */
    return ((uint16_t)(dma_stream->NDTR));
}



/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* handle received data in idle interrupt */
    if (huart == &DBUS_HUART)
    {
        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* handle dbus data dbus_buf from DMA */
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
				{
            rc_callback_handler(&rc, dbus_buf);
	
        }

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief      callback this function when uart interrupt
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
      if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
              __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
      {
          uart_rx_idle_callback(huart);
			}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart1)
    {
		   VisionUartRxCpltCallback();
	
    }
//    if (huart == &JUDGE_UART)
//    {
//        judgeUartRxCpltCallback();
//    }
	  HAL_UART_ErrorCallback(huart);
}
void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

