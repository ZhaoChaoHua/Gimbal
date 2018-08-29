/*
 * File      : position_sen.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 偏移地址：0x4C  基地址：0x40012000
 * ADC1的数据地址0x40012000+0x4C = 0x4001204C
 */
#include "position_sen.h"

// DMA Parameter
// 数据地址
#define ADC_RES_DR_Addr                ((uint32_t)0x4001204C)


uint16_t adc_value[AD_VALUE_NUM]; // ADC数据存储

static adc_dev adc1 = {
    .ADCx        = ADC1,
    .clk         = RCC_APB2Periph_ADC1,
    .clkcmd      = RCC_APB2PeriphClockCmd,
    
	.dmaclk      = RCC_AHB1Periph_DMA1,
	.dmacmd      = RCC_AHB1PeriphClockCmd,
	.dma_stream  = DMA1_Stream5,
    .dma_channel = DMA_Channel_7,
};
/** ADC1 device. */
adc_dev* const _ADC1 = &adc1;
