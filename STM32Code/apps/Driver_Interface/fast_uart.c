#include "fast_uart.h"
#include "board.h"
#include "gpio.h"
#include <rtdevice.h>

#define TX_LEN_GSM      255
#define RX_LEN_GSM      56*20

static struct rt_fserial_device rt_fuart_device;

static uint8_t tx_buffer[TX_LEN_GSM];
static uint8_t rx_buffer[RX_LEN_GSM];

#define RINGBUFFER_SIZE     56*20
static uint8_t buffer[RINGBUFFER_SIZE];
static struct rt_ringbuffer ring_data;

void Fast_Uart_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure ;  

    GPIO_InitTypeDef GPIO_InitStructure;  

    USART_InitTypeDef USART_InitStructure;  

    DMA_InitTypeDef DMA_InitStructure;  
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
       

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
 
    // DMA 发送中断
    NVIC_InitStructure.NVIC_IRQChannel = UART2_TX_DMA_IRQHandler;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  

    // DMA 发送配置
    DMA_DeInit(UART2_TX_DMA_STREAM);  
    DMA_InitStructure.DMA_Channel = UART2_TX_DMA_CHANNEL;   
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buffer;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
    DMA_InitStructure.DMA_BufferSize = TX_LEN_GSM;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
    DMA_Init(UART2_TX_DMA_STREAM, &DMA_InitStructure);    

    DMA_ITConfig(UART2_TX_DMA_STREAM,DMA_IT_TC,ENABLE);    



    // DMA 接收配置
    DMA_DeInit(UART2_RX_DMA_STREAM);  
    DMA_InitStructure.DMA_Channel = UART2_RX_DMA_CHANNEL;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx_buffer;   
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
    DMA_InitStructure.DMA_BufferSize = RX_LEN_GSM;   
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;     
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
    DMA_Init(UART2_RX_DMA_STREAM, &DMA_InitStructure);    
    DMA_Cmd(UART2_RX_DMA_STREAM,ENABLE);  
       
		// 串口配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
    USART_InitStructure.USART_StopBits = USART_StopBits_1;    
    USART_InitStructure.USART_Parity = USART_Parity_No;    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      
    USART_InitStructure.USART_BaudRate = 115200;   
  
    USART_Init(USART2,&USART_InitStructure);    
       
		// 开启串口中断
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);    
    
		// 串口中断配置
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;       
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;              
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 
    NVIC_Init(&NVIC_InitStructure);     
          
		// 使能串口及DMA
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);  
  
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
    USART_ITConfig(USART2,USART_IT_TXE,DISABLE);  
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);    
  
    USART_Cmd(USART2, ENABLE);      
    
		// 串口引脚配置
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   
    GPIO_PinAFConfig(UART2_GPIO,UART2_GPIO_TX,GPIO_AF_USART2);    
    GPIO_PinAFConfig(UART2_GPIO,UART2_GPIO_RX,GPIO_AF_USART2);  
  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_PIN_TX;  
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_PIN_RX;  
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);        

    // 初始化环形缓冲区
    rt_ringbuffer_init(&ring_data, buffer, RINGBUFFER_SIZE);
}

void fast_uart_tx(uint8_t *data,uint16_t size)
{
    DMA_Cmd(UART2_TX_DMA_STREAM, DISABLE);
      
    while (DMA_GetCmdStatus(UART2_TX_DMA_STREAM) != DISABLE){}
        
    rt_memcpy(tx_buffer, data, size);

    DMA_SetCurrDataCounter(UART2_TX_DMA_STREAM,size);

    DMA_Cmd(UART2_TX_DMA_STREAM,ENABLE);
}

// DMA发送中断
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(UART2_TX_DMA_STREAM,DMA_IT_TCIF6) != RESET)
    {
        // 清除中断标志位 
        DMA_ClearITPendingBit(UART2_TX_DMA_STREAM, DMA_IT_TCIF6);
    }
}

// DMA接收中断
void DMA1_Stream5_IRQHandler(void)
{
    uint16_t len = 0;
    
    if(DMA_GetITStatus(UART2_RX_DMA_STREAM,DMA_IT_TCIF5)!=RESET)
    {
        DMA_Cmd(UART2_RX_DMA_STREAM, DISABLE);
  
        len =RX_LEN_GSM - DMA_GetCurrDataCounter(UART2_RX_DMA_STREAM);  
        
        rt_ringbuffer_put(&ring_data, rx_buffer, len);
        
        DMA_ClearITPendingBit(UART2_RX_DMA_STREAM,DMA_IT_TCIF5);
        DMA_SetCurrDataCounter(UART2_RX_DMA_STREAM, RX_LEN_GSM);
        DMA_Cmd(UART2_RX_DMA_STREAM, ENABLE);
    }  
}

// 串口中断
void USART2_IRQHandler(void)
{
    uint16_t len = 0;

    // 接收完成中断
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(UART2_RX_DMA_STREAM,DISABLE);
        USART2->SR;
        USART2->DR;

        // 读取数据长度
        len = RX_LEN_GSM - DMA_GetCurrDataCounter(UART2_RX_DMA_STREAM);
        if(len != 0)
            rt_ringbuffer_put(&ring_data, rx_buffer, len);
        
        DMA_ClearITPendingBit(UART2_RX_DMA_STREAM,DMA_IT_TCIF5);
        DMA_SetCurrDataCounter(UART2_RX_DMA_STREAM,RX_LEN_GSM);

        DMA_Cmd(UART2_RX_DMA_STREAM,ENABLE);
    }
}

static rt_err_t rt_fast_uart_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_fast_uart_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_fast_uart_close(rt_device_t dev)
{
    return RT_EOK;
}

int scop_wid;
int scop_rid;
static rt_err_t rt_fast_uart_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    int *len = (int *)args;
    
    switch (cmd)
    {
        case RT_DEVICE_CTRL_DATA_READY:
            *len = (ring_data.write_index >= ring_data.read_index)? (ring_data.write_index - ring_data.read_index):
                    (ring_data.buffer_size - (ring_data.read_index - ring_data.write_index));
				    scop_rid = ring_data.read_index;
				    scop_wid = ring_data.write_index;
            break;
        default :
            break;
    }

    return RT_EOK;
}

static rt_size_t rt_fast_uart_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    rt_size_t ret;
    
    ret = rt_ringbuffer_get(&ring_data, (uint8_t *)buffer, size);
    
    return ret;
}

static rt_size_t rt_fast_uart_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    fast_uart_tx((uint8_t *)buffer, size);
    return size;
}

rt_err_t fast_uart_init()
{
    /* register device */
    rt_fuart_device.parent.type    = RT_Device_Class_Char;
    rt_fuart_device.parent.rx_indicate = RT_NULL;
    rt_fuart_device.parent.tx_complete = RT_NULL;
    
    rt_fuart_device.parent.init    = rt_fast_uart_init;
    rt_fuart_device.parent.open    = rt_fast_uart_open;
    rt_fuart_device.parent.close   = rt_fast_uart_close;
    rt_fuart_device.parent.read 	= rt_fast_uart_read;
    rt_fuart_device.parent.write   = rt_fast_uart_write;
    rt_fuart_device.parent.control = rt_fast_uart_control;
    /* no private */
    rt_fuart_device.parent.user_data = RT_NULL;

    rt_device_register(&rt_fuart_device.parent, "uart2",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
