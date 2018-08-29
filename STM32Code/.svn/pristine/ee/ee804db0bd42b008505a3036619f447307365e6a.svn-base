/*
 * File      : bxcan.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author            Notes
 * 2015-05-14     aubrcool@qq.com   first version
 * 2016-03-22     Lavi Liu          transplant to Cotex-M4
 */

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <bxcan.h>
#include "gpio.h"
#include "can.h"
#ifdef RT_USING_COMPONENTS_INIT
#include <components.h>
#endif

#define BX_CAN_FMRNUMBER 14    // bxCAN1提供14个可调整的筛选器组

// 最大过滤器数量是以16位列表
#define BX_CAN_MAX_FILTERS            (BX_CAN_FMRNUMBER * 4)                // 56
#define BX_CAN_MAX_FILTER_MASKS       BX_CAN_MAX_FILTERS                    // 56
#define BX_CAN_FILTER_MAX_ARRAY_SIZE  ((BX_CAN_MAX_FILTERS + 32 - 1) / 32)  // 2   每个过滤器占一位

// filter number of each FIFO
struct stm_bxcanfiltermap
{
    rt_uint32_t id32mask_cnt;
    rt_uint32_t id32bit_cnt;
    rt_uint32_t id16mask_cnt;
    rt_uint32_t id16bit_cnt;
};

// filter masks
//struct stm_bxcanfilter_masks
//{
//    // 32位拓展ID
//    rt_uint32_t id32maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE]; // 对应筛选器的32位屏蔽位
//    rt_uint32_t id32bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];  // 对应筛选器的32位列表模式
//    // 16位标准ID
//    rt_uint32_t id16maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE]; // 对应筛选器的16位屏蔽位模式
//    rt_uint32_t id16bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];  // 对应筛选器的16位列表模式
//    
//    rt_uint32_t id32maskshift[2]; // 32 bits-Mask shift
//    rt_uint32_t id32bitshift[2];  // 32 bits-Id   shift
//    rt_uint32_t id16maskshift[2]; // 16 bits-Mask shift
//    rt_uint32_t id16bitshift[2];  // 16 bits-Id   shift
//};

// CAN configure
struct stm_bxcan
{
    CAN_TypeDef *reg;
    void *mfrbase;     // Filter register bank address
    IRQn_Type sndirq;  // Transmit IRQ
    IRQn_Type rcvirq0; // FIFO0 Rx IRQ
    IRQn_Type rcvirq1; // FIFO1 Rx IRQ
    IRQn_Type errirq;  // ERROR    IRQ
    //struct stm_bxcanfilter_masks filtermask;            // 过滤器配置
    //rt_uint32_t alocmask[BX_CAN_FILTER_MAX_ARRAY_SIZE]; // memory of filter mask structure
    const rt_uint32_t filtercnt;                          // 筛选器数量
    rt_uint32_t fifo1filteroff;                     // FIFO1绑定过滤器的偏移量
    struct stm_bxcanfiltermap filtermap[2];               // FIFO0和FIFO1下的过滤器数量
};

#define BS1SHIFT 16
#define BS2SHIFT 20
#define RRESCLSHIFT 0
#define SJWSHIFT 24
#define BS1MASK ( (0x0F) << BS1SHIFT )
#define BS2MASK ( (0x07) << BS2SHIFT )
#define RRESCLMASK ( 0x3FF << RRESCLSHIFT )
#define SJWMASK ( 0x3 << SJWSHIFT )

#define MK_BKCAN_BAUD(SJW,BS1,BS2,PRES) ((SJW << SJWSHIFT) | (BS1 << BS1SHIFT) | (BS2 << BS2SHIFT) | (PRES << RRESCLSHIFT))

static const rt_uint32_t bxcan_baud_rate_tab[] =
{
    // 42 M
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 3), // 800KHz
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_6tq, CAN_BS2_3tq, 6),  // 600KHz
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 5), // 500KHz
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 11),
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 23),
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 29),
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_3tq, 59),
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_14tq, CAN_BS2_3tq, 149),
    MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_16tq, CAN_BS2_8tq, 199),
};

#define BAUD_DATA(TYPE,NO)   ((bxcan_baud_rate_tab[NO] & TYPE##MASK) >> TYPE##SHIFT)

/* CAN硬件初始化 */
static void bxcan_init(CAN_TypeDef *pcan, rt_uint32_t baud, rt_uint32_t mode)
{
    CAN_InitTypeDef CAN_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
    
    CAN_InitStructure.CAN_TTCM = DISABLE;  // 非时间触发通讯模式
    CAN_InitStructure.CAN_ABOM = DISABLE;   // 软件自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;  // 睡眠模式通过软件唤醒
    CAN_InitStructure.CAN_NART = ENABLE;  // 禁止报文自动传送 DISABLE
    CAN_InitStructure.CAN_RFLM = DISABLE;  // 报文不锁定，新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = ENABLE;   // 优先级由报文标识符决定 ENABLE
    
    switch (mode)
    {
    case RT_CAN_MODE_NORMAL:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
        break;
    case RT_CAN_MODE_LISEN:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
        break;
    case RT_CAN_MODE_LOOPBACK:
        CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
        break;
    }

    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Prescaler = 2;

    CAN_Init(pcan, &CAN_InitStructure);
}

/* CAN1硬件初始化 
 * CAN1_TX : PD1 (PB9 PA12)
 * CAN1_RX : PD0 (PB8 PA11)
 * CAN1_EN : PD3
 */
static void bxcan1_hw_init(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTD时钟	                   											 

    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOD0复用为CAN1
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOD1复用为CAN1
    
    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

// 初始化
static inline rt_err_t bxcan_enter_init(CAN_TypeDef *pcan)
{
    uint32_t wait_ack = 0x00000000;

    pcan->MCR |= CAN_MCR_INRQ ;

    while (((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
    {
        wait_ack++;
    }
    if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static inline rt_err_t bxcan_exit_init(CAN_TypeDef *pcan)
{
    uint32_t wait_ack = 0x00000000;

    pcan->MCR &= ~(uint32_t)CAN_MCR_INRQ;

    while (((pcan->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
    {
        wait_ack++;
    }
    if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_mode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    pcan->BTR &= ~(uint32_t)((uint32_t)0x03 << 30);
    switch (mode)
    {
    case RT_CAN_MODE_NORMAL:
        mode = CAN_Mode_Normal;
        break;
    case RT_CAN_MODE_LISEN:
        mode = CAN_Mode_Silent;
        break;
    case RT_CAN_MODE_LOOPBACK:
        mode = CAN_Mode_LoopBack;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        mode = CAN_Mode_Silent_LoopBack;
        break;
    }
    pcan->BTR |= ~(uint32_t)(mode << 30);
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_privmode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    if (mode == ENABLE)
    {
        pcan->MCR |= CAN_MCR_TXFP;
    }
    else
    {
        pcan->MCR &= ~(uint32_t)CAN_MCR_TXFP;
    }
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}
static rt_err_t bxcan_set_baud_rate(CAN_TypeDef *pcan, rt_uint32_t baud)
{
    rt_uint32_t mode;
    if (bxcan_enter_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    pcan->BTR = 0;
    mode = pcan->BTR & ((rt_uint32_t)0x03 << 30);
    pcan->BTR = (mode                         | \
                 ((BAUD_DATA(SJW, baud)) << 24) | \
                 ((BAUD_DATA(BS1, baud)) << 16) | \
                 ((BAUD_DATA(BS2, baud)) << 20) | \
                 (BAUD_DATA(RRESCL, baud)));
    if (bxcan_exit_init(pcan) != RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}


/*
 * 每组过滤器有两种工作模式
 * 一组过滤器中整组过滤器都使用同一种工作模式
 * 1个32位屏蔽位过滤器 EXT
 * 2个32位标识符过滤器
 * 2个16位屏蔽位过滤器 STD
 * 4个16位标识符过滤器
 */
 
static rt_err_t bxmodifyfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_item *pitem, rt_uint32_t actived)
{
    if (!actived)
    {
        return RT_EOK;
    }
    rt_uint32_t ID[2] = {0};
    rt_uint32_t shift;
    // 标识符寄存器
    rt_uint32_t thisid;
    // 掩码寄存器
    rt_uint32_t thismask;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    // 28组可编程过滤器
    if(pitem->fno > 27)
    {
        return RT_ERROR;
    }
    // 取出FR1与FR2寄存器值, 覆盖需要的修改的过滤器部分
    CAN_FilterRegister_TypeDef *pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[pitem->fno];
    ID[0] = pfilterreg->FR1;
    ID[1] = pfilterreg->FR2;
    
    // 取出对应的过滤器编号
    CAN_FilterInitStructure.CAN_FilterNumber = pitem->fno;
    
    // IDmask & IDlist
    if (pitem->mode == RT_CAN_MASK)
    {
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    }
    // 32bits & 16bits(EXT & STD)
    if (pitem->ide == RT_CAN_EXT)
    {
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
    }
    // 结构体中直接指定过滤器绑定缓冲区
    if (pitem->fifo == RT_CAN_FIFO0)
    {
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    }
    else
    {
        // 索引号加偏移
        pitem->hdr += pbxcan->fifo1filteroff;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO1;
    }
    
    // 具体参数
    // 0-STD  1-EXT
    // 0-MASK 1-LIST
    // 确定过滤器的类型
    // 过滤器参数设置
    uint8_t fcase = pitem->mode | (pitem->ide<<1);
    
    switch (fcase)
    {
    // 掩码模式 MASK
    case 0x02:  // 32位拓展屏蔽位模式
        if (actived)
        {
            shift = 3;
            thisid = (rt_uint32_t)pitem->id << shift;
            thismask = (rt_uint32_t)pitem->mask << shift;
            // EXTID
            if (pitem->ide)
            {
                thisid |= CAN_ID_EXT;
                thismask |= CAN_ID_EXT;
            }
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE;
                thismask |= CAN_RTR_REMOTE;
            }
            ID[0] = thisid;
            ID[1] = thismask;
        }
        else
        {
            ID[0] = 0xFFFFFFFF;
            ID[1] = 0xFFFFFFFF;
        }
        CAN_FilterInitStructure.CAN_FilterIdHigh = ((ID[0]>>16) & 0x0000FFFF);
        CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;
        
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ((ID[1]>>16) & 0x0000FFFF);
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[1] & 0x0000FFFF);
        break;
        
    case 0x03:  // 32位拓展列表模式
        if (actived)
        {
            shift = 3;
            thisid = (rt_uint32_t)pitem->id << shift;
            if (pitem->ide)
            {
                thisid |= CAN_ID_EXT;
            }
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE;
            }
            if(pitem->foff == 0)
            {
                ID[0] = thisid;
            }else{
                ID[1] = thisid;
            }
        }
        else
        {
            ID[0] = 0xFFFFFFFF;
        }
        CAN_FilterInitStructure.CAN_FilterIdHigh = ((ID[0]>>16) & 0x0000FFFF);
        CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;
        
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ((ID[1]>>16) & 0x0000FFFF);
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[1] & 0x0000FFFF);
        break;
        
    case 0x00:   // 16位标准屏蔽位标识符
        if (actived)
        {
            shift = 5;
            thisid = pitem->id << shift;
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE << (shift - 2);
            }
            thismask = pitem->mask << shift;
            if (pitem->rtr)
            {
                thismask |= CAN_RTR_REMOTE << (shift - 2);
            }
            if(pitem->foff == 0)
            {
                ID[0] = (thisid & 0x0000FFFF) | ((thismask & 0x0000FFFF) << 16);
            }else{
                ID[1] = (thisid & 0x0000FFFF) | ((thismask & 0x0000FFFF) << 16);
            }
        }
        else
        {
            ID[0] = 0xFFFFFFFF;
        }
        
        // ID0[31-16]       [15-0]           ID1[31-16]      [15-0]
        //   Filter 0 Mask  Filter 0 ID      Filter 1 Mask   Filter 1 ID
        CAN_FilterInitStructure.CAN_FilterIdHigh = ID[1] & 0x0000FFFF;       // Filter 1 ID
        CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;        // Filter 0 ID
        
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[1] & 0xFFFF0000) >> 16; // Filter 1 Mask
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[0] & 0xFFFF0000) >> 16;  // Filter 0 Mask
        break;
    case 0x01:   // 16位标准列表标识符
        if (actived)
        {
            shift = 5;
            thisid = pitem->id << shift;
            if (pitem->rtr)
            {
                thisid |= CAN_RTR_REMOTE << (shift - 2);
            }
            switch(pitem->foff)
            {
                case 0:
                    // 清除对应16位
                    ID[0] &= ~(0x0000FFFF);
                    ID[0] |= (thisid & 0xFFFF);
                    break;
                case 1:
                    ID[0] &= ~(0xFFFF0000);
                    ID[0] |= ((thisid & 0xFFFF)<<16);
                    break;
                case 2:
                    ID[1] &= ~(0x0000FFFF);
                    ID[1] |= (thisid & 0xFFFF);
                    break;
                case 3:
                    ID[1] &= ~(0xFFFF0000);
                    ID[1] |= ((thisid & 0xFFFF)<<16);
                    break;
                default :
                    break;
            }
        }
        else
        {
            ((rt_uint16_t *) ID)[0] = 0xFFFF;
        }
        // ID0[31-16]  [15-0]         ID1[31-16]   [15-0]
        //   Filter 1  Filter 0         Filter 3   Filter 2
        CAN_FilterInitStructure.CAN_FilterIdHigh = ID[1] & 0x0000FFFF;             // Filter 2
        CAN_FilterInitStructure.CAN_FilterIdLow =  ID[0] & 0x0000FFFF;             // Filter 0
        
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[1] & 0xFFFF0000) >> 16; // Filter 3
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[0] & 0xFFFF0000) >> 16;  // Filter 1
        break;
    }

    if (ID[0] != 0xFFFFFFFF || ID[1] != 0xFFFFFFFF)
    {
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    }
    else
    {
        CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    }
    
    CAN_FilterInit(&CAN_FilterInitStructure);
    return RT_EOK;
}


static rt_err_t setfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_config *pconfig)
{
    struct rt_can_filter_item *pitem = pconfig->items;

    rt_uint32_t count = pconfig->count;
    rt_err_t res;
    rt_uint32_t index = 0;
    // 修改过滤器匹配索引
    pbxcan->fifo1filteroff = pconfig->fifobaseoff;
    while (count) // 过滤器数量
    {
        res = bxmodifyfilter(pbxcan, &pitem[index], pconfig->actived);
        if (res != RT_EOK)
        {
            return res;
        }
        index++;
        count--;
    }

    return RT_EOK;
}

/* CAN总线配置函数 */
static rt_err_t configure(struct rt_can_device *can, struct can_configure *cfg)
{
    CAN_TypeDef *pbxcan;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));
    // CAN1初始化
    if (pbxcan == CAN1)
    {
        // GPIO,中断配置
        bxcan1_hw_init();
        // CAN总线配置(模式、波特率、基本参数)
        bxcan_init(pbxcan, cfg->baud_rate, can->config.mode);
        // Filter 设置在Control中设置
        //bxcan1_filter_init(can);
    }
    return RT_EOK;
}

static rt_err_t control(struct rt_can_device *can, int cmd, void *arg)
{
    struct stm_bxcan *pbxcan;
    rt_uint32_t argval;
    NVIC_InitTypeDef  NVIC_InitStructure;

    pbxcan = (struct stm_bxcan *) can->parent.user_data;
    assert_param(pbxcan != RT_NULL);

    switch (cmd)
    {
    // 设置初始化
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            NVIC_DisableIRQ(pbxcan->rcvirq0);
            NVIC_DisableIRQ(pbxcan->rcvirq1);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , DISABLE);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            NVIC_DisableIRQ(pbxcan->sndirq);
            CAN_ITConfig(pbxcan->reg, CAN_IT_TME, DISABLE);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , DISABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , DISABLE);
            NVIC_DisableIRQ(pbxcan->errirq);
        }
        break;
    // 控制设置
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq0;
            NVIC_Init(&NVIC_InitStructure);
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq1;
            NVIC_Init(&NVIC_InitStructure);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_TME, ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->sndirq;
            NVIC_Init(&NVIC_InitStructure);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , ENABLE);
            CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , ENABLE);
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_InitStructure.NVIC_IRQChannel = pbxcan->errirq;
            NVIC_Init(&NVIC_InitStructure);
        }
        break;
    // 设置过滤器
    case RT_CAN_CMD_SET_FILTER:
        return setfilter(pbxcan, (struct rt_can_filter_config *) arg);
        //break;
    // 设置CAN总线模式
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_NORMAL ||
                argval != RT_CAN_MODE_LISEN ||
                argval != RT_CAN_MODE_LOOPBACK ||
                argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return RT_ERROR;
        }
        if (argval != can->config.mode)
        {
            can->config.mode = argval;
            return bxcan_set_mode(pbxcan->reg, argval);
        }
        break;
    // 设置CAN总线波特率
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t) arg;
        if (argval != CAN1MBaud &&
                argval != CAN800kBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
        {
            return RT_ERROR;
        }
        if (argval != can->config.baud_rate)
        {
            can->config.baud_rate = argval;
            return bxcan_set_baud_rate(pbxcan->reg, argval);
        }
        break;
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_PRIV ||
                argval != RT_CAN_MODE_NOPRIV)
        {
            return RT_ERROR;
        }
        if (argval != can->config.privmode)
        {
            can->config.privmode = argval;
            return bxcan_set_privmode(pbxcan->reg, argval);
        }
        break;
    // 返回状态
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype;
        errtype = pbxcan->reg->ESR;
        can->status.rcverrcnt = errtype >> 24;
        can->status.snderrcnt = (errtype >> 16 & 0xFF);
        can->status.errcode = errtype & 0x07;
        if (arg != &can->status)
        {
            rt_memcpy(arg, &can->status, sizeof(can->status));
        }
    }
    break;
    }

    return RT_EOK;
}


static int sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    CAN_TypeDef *pbxcan;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));

    pbxcan->sTxMailBox[boxno].TIR &= TMIDxR_TXRQ;
    if (pmsg->ide == RT_CAN_STDID)
    {
        assert_param(IS_CAN_STDID(pmsg->id));
        pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 21) | \
                                          pmsg->rtr);
    }
    else
    {
        assert_param(IS_CAN_EXTID(pmsg->id));
        pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 3) | \
                                          pmsg->ide << 2 | \
                                          pmsg->rtr);
    }

    pmsg->len &= (uint8_t)0x0000000F;
    pbxcan->sTxMailBox[boxno].TDTR &= (uint32_t)0xFFFFFFF0;
    pbxcan->sTxMailBox[boxno].TDTR |= pmsg->len;

    pbxcan->sTxMailBox[boxno].TDLR = (((uint32_t)pmsg->data[3] << 24) |
                                      ((uint32_t)pmsg->data[2] << 16) |
                                      ((uint32_t)pmsg->data[1] << 8) |
                                      ((uint32_t)pmsg->data[0]));
    if (pmsg->len > 4)
    {
        pbxcan->sTxMailBox[boxno].TDHR = (((uint32_t)pmsg->data[7] << 24) |
                                          ((uint32_t)pmsg->data[6] << 16) |
                                          ((uint32_t)pmsg->data[5] << 8) |
                                          ((uint32_t)pmsg->data[4]));
    }
    // 发送邮箱内容请求
    pbxcan->sTxMailBox[boxno].TIR |= TMIDxR_TXRQ;

    return RT_EOK;
}

// FIFO编号
static int recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
    CAN_TypeDef *pbxcan;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;

    pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
    assert_param(IS_CAN_ALL_PERIPH(pbxcan));
    assert_param(IS_CAN_FIFO(boxno));
    pmsg->ide = ((uint8_t)0x04 & pbxcan->sFIFOMailBox[boxno].RIR) >> 2;
    if (pmsg->ide == CAN_Id_Standard)
    {
        pmsg->id = (uint32_t)0x000007FF & (pbxcan->sFIFOMailBox[boxno].RIR >> 21);
    }
    else
    {
        pmsg->id = (uint32_t)0x1FFFFFFF & (pbxcan->sFIFOMailBox[boxno].RIR >> 3);
    }

    pmsg->rtr = (uint8_t)((0x02 & pbxcan->sFIFOMailBox[boxno].RIR) >> 1);
    pmsg->len = (uint8_t)0x0F & pbxcan->sFIFOMailBox[boxno].RDTR;
    pmsg->data[0] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDLR;
    pmsg->data[1] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 8);
    pmsg->data[2] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 16);
    pmsg->data[3] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 24);
    if (pmsg->len > 4)
    {
        pmsg->data[4] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDHR;
        pmsg->data[5] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 8);
        pmsg->data[6] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 16);
        pmsg->data[7] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 24);
    }
    pmsg->hdr = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDTR >> 8);
    //如果消息来自FIFO1
    // HDR直接由寄存器中读取
    if (boxno) 
        pmsg->hdr += ((struct stm_bxcan *) can->parent.user_data)->fifo1filteroff;
    return RT_EOK;
}

// 操作函数注册
static const struct rt_can_ops canops =
{
    configure,
    control,
    sendmsg,
    recvmsg,
};

// CAN配置结构体
static struct stm_bxcan bxcan1data =
{
    .reg = CAN1,                                    // CAN1配置结构体
    .mfrbase = (void *) &CAN1->sFilterRegister[0],  // CAN1过滤寄存器首地址
    .sndirq  = CAN1_TX_IRQn,                        // 发送中断
    .rcvirq0 = CAN1_RX0_IRQn,                       // FIFO0接收中断
    .rcvirq1 = CAN1_RX1_IRQn,                       // FIFO1接收中断
    .errirq  =  CAN1_SCE_IRQn,                      // CAN总线错误中断
    //.alocmask = {0, 0},
    .filtercnt = BX_CAN_FMRNUMBER,                  // 筛选器的数量28
    .fifo1filteroff = 0,                            // FIFO0 7 FIFO1 7
};

struct rt_can_device bxcan1;
void CAN1_RX0_IRQHandler(void)
{
    if (CAN1->RF0R & 0x03)
    {
        if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0)
        {
            CAN1->RF0R = CAN_RF0R_FOVR0;
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 0 << 8);
        }
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}

void CAN1_RX1_IRQHandler(void)
{
    if (CAN1->RF1R & 0x03)
    {
        if ((CAN1->RF1R & CAN_RF1R_FOVR1) != 0)
        {
            CAN1->RF1R = CAN_RF1R_FOVR1;
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 1 << 8); // FIFO1
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 1 << 8);
        }
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }
}

void CAN1_TX_IRQHandler(void)
{
    rt_uint32_t state;
    if (CAN1->TSR & (CAN_TSR_RQCP0))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
        // 请求完整的邮箱
        CAN1->TSR |= CAN_TSR_RQCP0;
        if (state == (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    if (CAN1->TSR & (CAN_TSR_RQCP1))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
        CAN1->TSR |= CAN_TSR_RQCP1;
        if (state == (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
    }
    if (CAN1->TSR & (CAN_TSR_RQCP2))
    {
        state =  CAN1->TSR & (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
        CAN1->TSR |= CAN_TSR_RQCP2;
        if (state == (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2))
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
    }
}

void CAN1_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    errtype = CAN1->ESR;
    if (errtype & 0x70 && bxcan1.status.lasterrtype == (errtype & 0x70))
    {
        switch ((errtype & 0x70) >> 4)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            bxcan1.status.bitpaderrcnt++;
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            bxcan1.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR:
            bxcan1.status.ackerrcnt++;
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            bxcan1.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            bxcan1.status.crcerrcnt++;
            break;
        }
        bxcan1.status.lasterrtype = errtype & 0x70;
        CAN1->ESR &= ~0x70;
    }
    bxcan1.status.rcverrcnt = errtype >> 24;
    bxcan1.status.snderrcnt = (errtype >> 16 & 0xFF);
    bxcan1.status.errcode = errtype & 0x07;
    CAN1->MSR |= CAN_MSR_ERRI;
}


/*
 * 初始化CAN总线的硬件参数
 * 注册CAN总线驱动
 */
int stm32_bxcan_init(void)
{
    // 使能CAN1时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    CAN_DeInit(CAN1);
    bxcan1.config.baud_rate = CAN1MBaud;
    bxcan1.config.msgboxsz = 16;            // 16个接收链表节点
    bxcan1.config.sndboxnumber = 3;         // 三个发送链表节点
    bxcan1.config.mode = RT_CAN_MODE_NORMAL; //RT_CAN_MODE_NORMAL;
    bxcan1.config.privmode = 0;             // 优先级模式
    bxcan1.config.ticks = 50;               // 超时时间 500ms

    bxcan1.config.maxhdr = BX_CAN_FMRNUMBER * 4; // 56
    
    rt_hw_can_register(&bxcan1, "bxcan1", &canops, &bxcan1data);
    return RT_EOK;
}

