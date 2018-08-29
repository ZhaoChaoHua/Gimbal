#include "stm32_spi.h"
#include <string.h>

static uint8_t dummy = 0xFF;

// SPI片选引脚链表
static struct spi_cs_list *cs_list;
static uint8_t listCount = 0;
/*
static spi_dev spi1 = 
{
    .SPIx         = SPI1,
    .clk	      = RCC_APB2Periph_SPI1,
    .clkcmd       = RCC_APB2PeriphClockCmd,
    .bits         = 8,
    .mode         = RT_SPI_MODE_3 | RT_SPI_MSB,
    .bps          = 1000000,
    .spi_sck      = PB3,
    .spi_miso     = PB4,
    .spi_mosi     = PB5,
    .dmaclk         = RCC_AHB1Periph_DMA2,
    .dmaclkcmd      = RCC_AHB1PeriphClockCmd,
    .DMA_Stream_RX  = DMA2_Stream0,
    .DMA_Channel_RX = DMA_Channel_3, 
    .DMA_Stream_TX  = DMA2_Stream3,
    .DMA_Channel_TX = DMA_Channel_3,
    .DMA_Channel_TX_FLAG_TC = DMA_FLAG_TCIF3,
    .DMA_Channel_RX_FLAG_TC = DMA_FLAG_TCIF0,
};
*/
static spi_dev spi1 = 
{
    SPI1,
    RCC_APB2Periph_SPI1,
    RCC_APB2PeriphClockCmd,
    8,
    RT_SPI_MODE_3 | RT_SPI_MSB,
    1000000,
    PB3,
    PB4,
    PB5,
    RCC_AHB1Periph_DMA2,
    RCC_AHB1PeriphClockCmd,
    DMA2_Stream0,
    DMA_Channel_3,
    DMA2_Stream3,
    DMA_Channel_3,
    DMA_FLAG_TCIF0,
    DMA_FLAG_TCIF3,
};

spi_dev *_SPI1 = &spi1;

SPI stm32_spi(_SPI1, "spi1");

SPI::SPI(spi_dev* spi, const char *name)
{
    this->SPI_x = spi;
    this->mutex_name = name;
}

SPI::~SPI()
{
}

/* SPI GPIO引脚设置 */
rt_err_t SPI::Mutex_GPIO_Config()
{   
    rt_err_t result;
    /* initialize mutex lock */
    result = rt_mutex_init(&(this->lock), this->mutex_name, RT_IPC_FLAG_FIFO);
    
    Set_pinMode(this->SPI_x->spi_sck,  AF_OUTPUT);
    Set_pinMode(this->SPI_x->spi_miso, AF_OUTPUT);
    Set_pinMode(this->SPI_x->spi_mosi, AF_OUTPUT);
    
    return result;
}

/* 存储SPI片选引脚 */
rt_err_t SPI::SPI_CS_Config(const char *name, uint8_t cs)
{
    // 调整链表内存
    cs_list = static_cast<spi_cs_list *>(rt_realloc(cs_list, (listCount +1) * sizeof(spi_cs_list)));
    if(cs_list != RT_NULL)
    {
        rt_memcpy(cs_list[listCount].spi_cs, name, MAX_CS_PIN_NAME);
        cs_list[listCount].pin = cs;
        // 配置推挽输出模式
        Set_pinMode(cs_list[listCount].pin, OUTPUT);
        // 输出高电平
        GPIO_OUT(cs_list[listCount].pin, 1);
        listCount++;
        return RT_EOK;
    }
    return RT_ERROR;
}

/* 配置硬件SPI */
rt_err_t SPI::SPI_HW_Init()
{
    SPI_InitTypeDef SPI_InitStruct;
    // 配置GPIO引脚
    this->Mutex_GPIO_Config();
    // 开启时钟
    this->SPI_x->clkcmd(this->SPI_x->clk, ENABLE);
    SPI_DeInit(this->SPI_x->SPIx);
    //SPI_StructInit(&SPI_InitStruct);
    // 数据宽度设置
    if(this->SPI_x->bits <= 8)
    {
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    }
    else if(this->SPI_x->bits <= 16)
    {
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }
    
    /* CPOL */
    if(this->SPI_x->mode & RT_SPI_CPOL)
    {
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    }
    
    /* CPHA */
    if(this->SPI_x->mode & RT_SPI_CPHA)
    {
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    }
    
    /* MSB or LSB */
    if(this->SPI_x->mode & RT_SPI_MSB)
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    /* baudrate */
    uint32_t SPI_APB_CLOCK;
    SPI_APB_CLOCK = SystemCoreClock / 4;
    if(this->SPI_x->bps >= SPI_APB_CLOCK/2 && SPI_APB_CLOCK/2 <= 30000000)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/4)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/8)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/16)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/32)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/64)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if(this->SPI_x->bps >= SPI_APB_CLOCK/128)
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   // Full Duplex
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                        // Master Mode
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;                            // Software NSS Signal
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(this->SPI_x->SPIx, &SPI_InitStruct);

    SPI_Cmd(this->SPI_x->SPIx, ENABLE);

    return RT_EOK;
}

rt_err_t SPI::DMA_Config(const void *send_addr, void *recv_addr, rt_size_t size)
{
    uint8_t i = 0;
    // DMA配置
    DMA_InitTypeDef DMA_InitStructure;
    // DMA时钟使能
    this->SPI_x->dmaclkcmd(this->SPI_x->dmaclk,ENABLE);
    
    /* 重置DMA Stream寄存器 */
    DMA_DeInit(this->SPI_x->DMA_Stream_RX);
    DMA_DeInit(this->SPI_x->DMA_Stream_TX);
    /* 确认DMA Stream是否禁止 */
    while (DMA_GetCmdStatus(this->SPI_x->DMA_Stream_RX) != DISABLE)
    {
        i++;
        if(i>255)
            return RT_ERROR;
    }
    i = 0;
    while (DMA_GetCmdStatus(this->SPI_x->DMA_Stream_TX) != DISABLE)
    {
        i++;
        if(i>255)
            return RT_ERROR;
    }
    /* DMA接收Config */
    DMA_Cmd(this->SPI_x->DMA_Stream_RX, DISABLE);
    DMA_InitStructure.DMA_Channel = this->SPI_x->DMA_Channel_RX;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(this->SPI_x->SPIx->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = (uint32_t)size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) recv_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }
    DMA_Init(this->SPI_x->DMA_Stream_RX, &DMA_InitStructure);
    DMA_Cmd(this->SPI_x->DMA_Stream_RX, ENABLE);
    
    /* DMA发送Config */
    DMA_Cmd(this->SPI_x->DMA_Stream_TX, DISABLE);

    DMA_InitStructure.DMA_Channel = this->SPI_x->DMA_Channel_TX;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(this->SPI_x->SPIx->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = (uint32_t)size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    if(send_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)send_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        dummy = 0xFF;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(this->SPI_x->DMA_Stream_TX, &DMA_InitStructure);
    DMA_Cmd(this->SPI_x->DMA_Stream_TX, ENABLE);
    // 标志位设置
    
    return RT_EOK;
}

/* SPI收发程序 8bits */
rt_err_t SPI::rt_spi_transfer(uint8_t *send, uint8_t *recv, uint32_t size, uint8_t cs)
{
    uint8_t data = 0xFF;
    rt_err_t result;
    RT_ASSERT(this->SPI_x != RT_NULL);
    
    result = rt_mutex_take(&(this->lock), RT_WAITING_FOREVER);
    
    //GPIO_OUT(cs_list[cs].pin, 0);
    while(size--)
    {
        // 读取数据
        if(send != RT_NULL)
        {
            data = *send++;
        }
        //Wait until the transmit buffer is empty
        while (SPI_I2S_GetFlagStatus(this->SPI_x->SPIx, SPI_I2S_FLAG_TXE) == RESET);
        // Send the byte
        SPI_I2S_SendData(this->SPI_x->SPIx, data);

        //Wait until a data is received
        while (SPI_I2S_GetFlagStatus(this->SPI_x->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
        // Get the received data
        data = SPI_I2S_ReceiveData(this->SPI_x->SPIx);
        if(recv != RT_NULL)
        {
            *recv++ = data;
        }
    }
    //GPIO_OUT(cs_list[cs].pin, 1);
    
    rt_mutex_release(&(this->lock));
    
    return result;
}

/* SPI收发程序 16bits */
rt_err_t SPI::rt_spi_transfer(uint16_t *send, uint16_t *recv, uint32_t size, uint8_t cs)
{
    uint16_t data = 0xFF;
    rt_err_t result;
    RT_ASSERT(this->SPI_x != RT_NULL);
    
    result = rt_mutex_take(&(this->lock), RT_WAITING_FOREVER);
    
    GPIO_OUT(cs_list[cs].pin, 0);
    while(size--)
    {
        // 读取数据
        if(send != RT_NULL)
        {
            data = *send++;
        }
        //Wait until the transmit buffer is empty
        while (SPI_I2S_GetFlagStatus(this->SPI_x->SPIx, SPI_I2S_FLAG_TXE) == RESET);
        // Send the byte
        SPI_I2S_SendData(this->SPI_x->SPIx, data);

        //Wait until a data is received
        while (SPI_I2S_GetFlagStatus(this->SPI_x->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
        // Get the received data
        data = SPI_I2S_ReceiveData(this->SPI_x->SPIx);
        if(recv != RT_NULL)
        {
            *recv++ = data;
        }
    }
    GPIO_OUT(cs_list[cs].pin, 1);
    
    rt_mutex_release(&(this->lock));
    
    return result;
}

/* DMA发送数据重载 */
rt_err_t SPI::rt_spi_transfer(uint8_t cs, uint8_t ndtr)
{
    rt_err_t result;
    result = rt_mutex_take(&(this->lock), RT_WAITING_FOREVER);
    
    DMA_SetCurrDataCounter(this->SPI_x->DMA_Stream_RX, ndtr);
    DMA_SetCurrDataCounter(this->SPI_x->DMA_Stream_TX, ndtr);
    
    DMA_ClearFlag(this->SPI_x->DMA_Stream_RX, this->SPI_x->DMA_Channel_RX_FLAG_TC);
    DMA_ClearFlag(this->SPI_x->DMA_Stream_TX, this->SPI_x->DMA_Channel_TX_FLAG_TC);
    // 启动DMA前读一次DR
    if(SPI_I2S_GetFlagStatus(this->SPI_x->SPIx, SPI_I2S_FLAG_RXNE)!=RESET)
    {
        result=this->SPI_x->SPIx->DR;
    }
    //GPIO_OUT(cs_list[cs].pin, 0);
    // 启动DMA
    SPI_I2S_DMACmd(this->SPI_x->SPIx, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    // 等待数据传输完毕
    while (DMA_GetFlagStatus(this->SPI_x->DMA_Stream_RX, this->SPI_x->DMA_Channel_RX_FLAG_TC) == RESET
        || DMA_GetFlagStatus(this->SPI_x->DMA_Stream_TX, this->SPI_x->DMA_Channel_TX_FLAG_TC) == RESET);
    // 停止DMA传输
    SPI_I2S_DMACmd(this->SPI_x->SPIx, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    
    //GPIO_OUT(cs_list[cs].pin, 1);
    
    rt_mutex_release(&(this->lock));
    
    return result;
}

rt_err_t SPI::mpu6500_check()
{
    uint8_t send;
    uint8_t recv = 0x00;
    
    send = 0x75|0x80;
    
    return RT_EOK;
}

#ifdef __cplusplus
extern "C"
{
#endif
    
#ifdef RT_USING_FINSH
#include <finsh.h>

void spi_shell()
{   
    uint8_t send[1];
    uint8_t recv[1];
    stm32_spi.Mutex_GPIO_Config();
    stm32_spi.SPI_HW_Init();
    stm32_spi.SPI_CS_Config("test", PC5);
    stm32_spi.DMA_Config(send, recv, 1);
    GPIO_OUT(cs_list[0].pin, 0);
    send[0] = 0x75|0x80;
    stm32_spi.rt_spi_transfer(0);
    send[0] = 0xFF;
    stm32_spi.rt_spi_transfer(0);
    GPIO_OUT(cs_list[0].pin, 1);
}
// 添加FINSH的list()中
FINSH_FUNCTION_EXPORT(spi_shell, Debug spi.)
#endif
#ifdef __cplusplus
}
#endif
