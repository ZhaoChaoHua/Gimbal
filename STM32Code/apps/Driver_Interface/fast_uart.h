#ifndef __FAST_UART_H__
#define __FAST_UART_H__

#include <rtthread.h>
#include <stm32f4xx.h>

#define UART2_GPIO              GPIOA
#define UART2_GPIO_TX           GPIO_PinSource2
#define UART2_GPIO_RX           GPIO_PinSource3
#define UART2_GPIO_PIN_TX       GPIO_Pin_2
#define UART2_GPIO_PIN_RX       GPIO_Pin_3
#define RCC_APBPeriph_UART2     RCC_APB1Periph_USART2
#define RCC_APBPeriph_UART2_DMA RCC_AHB1Periph_DMA1
#define UART2_TX_DMA_IRQHandler DMA1_Stream6_IRQn
#define UART2_TX_DMA_FLAG_TCIF  DMA_FLAG_TCIF6
#define UART2_TX_DMA_STREAM     DMA1_Stream6
#define UART2_TX_DMA_CHANNEL    DMA_Channel_4

#define UART2_RX_DMA_IRQHandler DMA1_Stream5_IRQn
#define UART2_RX_DMA_FLAG_TCIF  DMA_FLAG_TCIF5
#define UART2_RX_DMA_STREAM     DMA1_Stream5
#define UART2_RX_DMA_CHANNEL    DMA_Channel_4

struct rt_fserial_device
{
    struct rt_device          parent;
};
typedef struct rt_fserial_device rt_fserial_t;

void Fast_Uart_Config(void);
rt_err_t fast_uart_init(void);

#endif
