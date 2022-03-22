/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_adc.c
 * @brief      this file contains sd card basic operating function
 * @note       ADC1(IN1) (IN6)
 * @Version    V1.0.0
 * @Date       2021.5      
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_adc.h"

extern ADC_HandleTypeDef hadc1;
float mcu_temperate;
uint16_t ADC_Get(uint32_t ch)   
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    ADC1_ChanConf.Channel=ch;                                   //ͨ��
    ADC1_ChanConf.Rank=1;                                       //��1�����У�����1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;  
	ADC1_ChanConf.Offset=0;
    HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf);        //ͨ������
	
    HAL_ADC_Start(&hadc1);                               //����ADC
	
    HAL_ADC_PollForConversion(&hadc1,10);                //��ѯת��
 
	return (uint16_t)HAL_ADC_GetValue(&hadc1);	        	//�������һ��ADC1�������ת�����
}
float ADC_GetMcuTemperate(void)
{
    uint16_t adcx = 0;
    float temperate;
	
    adcx = ADC_Get(ADC_CHANNEL_18);
    temperate = (float)adcx * (3.3f / 4096.0f);
    temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
    return temperate;
}
