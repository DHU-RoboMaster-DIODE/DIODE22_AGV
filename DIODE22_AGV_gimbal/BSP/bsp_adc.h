#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "struct_typedef.h"
#define MUC_TEMPERATE ADC_GetMcuTemperate()

uint16_t ADC_Get(uint32_t ch);

float ADC_GetMcuTemperate(void);
extern void init_vrefint_reciprocal(void);
extern fp32 get_temprate(void);
extern fp32 get_battery_voltage(void);
extern uint8_t get_hardware_version(void);

#endif

