#include "stm32f4xx_hal.h"
#include "can.h"
#include "bsp_can.h"
#include "SuperCapacity.h"
uint8_t sendbuf[8];

//³¬¼¶µçÈÝ
void SuperCapacitance(float temPower) {
	
    sendbuf[0] = (uint16_t)(temPower*100) >>8;
    sendbuf[1] = (uint16_t)(temPower*100) ;

    CAN_SendMsg(&hcan1,0,0x210,8,sendbuf);
}
