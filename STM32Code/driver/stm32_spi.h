#ifndef __STM32_SPI_H__
#define __STM32_SPI_H__

#include <stdlib.h>

#define MAX_CS_PIN_NAME     6
#ifdef __cplusplus
extern "C"{
#endif
#include <rtthread.h>
#include "stm32f4xx.h"
#include "gpio.h"
    
#define RT_SPI_CPHA     (1<<0)                             /* bit[0]:CPHA, clock phase */
#define RT_SPI_CPOL     (1<<1)                             /* bit[1]:CPOL, clock polarity */
/**
 * At CPOL=0 the base value of the clock is zero
 *  - For CPHA=0, data are captured on the clock's rising edge (low→high transition)
 *    and data are propagated on a falling edge (high→low clock transition).
 *  - For CPHA=1, data are captured on the clock's falling edge and data are
 *    propagated on a rising edge.
 * At CPOL=1 the base value of the clock is one (inversion of CPOL=0)
 *  - For CPHA=0, data are captured on clock's falling edge and data are propagated
 *    on a rising edge.
 *  - For CPHA=1, data are captured on clock's rising edge and data are propagated
 *    on a falling edge.
 */
#define RT_SPI_LSB      (0<<2)                             /* bit[2]: 0-LSB */
#define RT_SPI_MSB      (1<<2)                             /* bit[2]: 1-MSB */

#define RT_SPI_MASTER   (0<<3)                             /* SPI master device */
#define RT_SPI_SLAVE    (1<<3)                             /* SPI slave device */

#define RT_SPI_MODE_0       (0 | 0)                        /* CPOL = 0, CPHA = 0 */
#define RT_SPI_MODE_1       (0 | RT_SPI_CPHA)              /* CPOL = 0, CPHA = 1 */
#define RT_SPI_MODE_2       (RT_SPI_CPOL | 0)              /* CPOL = 1, CPHA = 0 */
#define RT_SPI_MODE_3       (RT_SPI_CPOL | RT_SPI_CPHA)    /* CPOL = 1, CPHA = 1 */

#define RT_SPI_MODE_MASK    (RT_SPI_CPHA | RT_SPI_CPOL | RT_SPI_MSB)

/**
 * SPI message structure
 */
struct rt_spi_message
{
    const void *send_buf;
    void *recv_buf;
    rt_size_t length;
    struct rt_spi_message *next;

    unsigned cs_take    : 1;
    unsigned cs_release : 1;
};

/** Timer device type */
typedef struct spi_dev_structure {
    SPI_TypeDef*  SPIx;
    uint32_t      clk;        // 时钟
	rcc_clockcmd  clkcmd;     // 时钟使能函数
    // 基本配置
    uint8_t       bits;
    uint8_t       mode;
    uint32_t      bps;
    uint8_t       spi_sck;
    uint8_t       spi_miso;
    uint8_t       spi_mosi;
    // DMA设置
    uint32_t      dmaclk;
    rcc_clockcmd  dmaclkcmd;
    DMA_Stream_TypeDef* DMA_Stream_TX;
    uint32_t            DMA_Channel_TX;

    DMA_Stream_TypeDef* DMA_Stream_RX;
    uint32_t            DMA_Channel_RX;

    uint32_t            DMA_Channel_TX_FLAG_TC;
    uint32_t            DMA_Channel_RX_FLAG_TC;
    
    IRQn_Type     irq;        // 定时器中断函数
    uint32_t      priority;
    voidFuncPtr   handlers; // 用户定义的中断事件
} spi_dev;

#ifdef __cplusplus
}
#endif

struct spi_cs_list 
{
    char spi_cs[MAX_CS_PIN_NAME];
    uint8_t pin;
};

class SPI
{
    private:
        /* 信号量(保证传输不被中断) */
        struct rt_mutex lock;
        spi_dev* SPI_x;
        const char *mutex_name;
        
    public:
        SPI(spi_dev* spi, const char *name);
        ~SPI();
        /* GPIO配置 */
        rt_err_t Mutex_GPIO_Config();
        /* SPI添加CS片选引脚 */
        rt_err_t SPI_CS_Config(const char *name, uint8_t cs);
        /* SPI硬件配置 */
        rt_err_t SPI_HW_Init();
        /* 设置波特率 默认配置波特率为1MHz */
        void frequency(rt_uint32_t hz);
        /* SPI DMA配置 */
        rt_err_t DMA_Config(const void * send_addr, void * recv_addr, rt_size_t size);
        /* SPI收发程序 */
        rt_err_t rt_spi_transfer(uint8_t *send, uint8_t *recv, uint32_t size, uint8_t cs);
        rt_err_t rt_spi_transfer(uint16_t *send, uint16_t *recv, uint32_t size, uint8_t cs);
        rt_err_t rt_spi_transfer(uint8_t cs, uint8_t ndtr);
        /* 确认MPU6500死活 */
        rt_err_t mpu6500_check();
};

#endif
