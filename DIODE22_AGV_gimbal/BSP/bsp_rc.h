#ifndef __BSP_RC_H__
#define __BSP_RC_H__

#include "main.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART      huart3 /* for dji remote controler reciever */
/**
  * @brief  remote control information
  */
//typedef __packed struct
//{
//    /* rocker channel information */
//    int16_t ch0;
//    int16_t ch1;
//    int16_t ch2;
//    int16_t ch3;
//    int16_t ch4;	
//    /* left and right lever information */
//    uint8_t sw1;
//    uint8_t sw2;
//} rc_info_t;
typedef __packed struct
{
    /* rocker channel information */
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;	
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z; 
	
	uint8_t mouse_press_l;
	uint8_t mouse_press_r;
	
//	uint16_t key_v;
	uint8_t key[16];
} rc_info_t;

extern rc_info_t rc,rc_last;


extern void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
void dbus_uart_init(void);
#endif

