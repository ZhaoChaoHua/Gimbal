#ifndef __POSITION_SEN_H__
#define __POSITION_SEN_H__

#include "stm32f40x_define.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)    //ADC1寄存器基地址

#define AD_VALUE_NUM       2

/** ADC device type. */
typedef struct adc_dev_structure {
	ADC_TypeDef*          ADCx;
	uint32_t              clk;
	rcc_clockcmd          clkcmd;
	// DMA通道
	uint32_t              dmaclk;
	rcc_clockcmd          dmacmd;
	DMA_Stream_TypeDef*   dma_stream; // DMA通道
    uint32_t              dma_channel;
} adc_dev;


extern adc_dev* const _ADC1;
extern uint16_t adc_value[AD_VALUE_NUM];

#endif
