/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "board.h"

/**
 * @addtogroup STM32
 */

/*@{*/
extern int  rt_application_init(void);
#ifdef RT_USING_FINSH
extern void finsh_system_init(void);
extern void finsh_set_device(const char* device);
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define STM32_SRAM_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define STM32_SRAM_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define STM32_SRAM_BEGIN    (&__bss_end)
#endif

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
    rt_kprintf("\n\r Wrong parameter value detected on\r\n");
    rt_kprintf("       file  %s\r\n", file);
    rt_kprintf("       line  %d\r\n", line);

    while (1) ;
}

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
    /* 控制板硬件初始化 */
    rt_hw_board_init();

    /* 显示版本信息 */
    //rt_show_version();

    /* 滴答计时器初始化 */
    rt_system_tick_init();

    /* init kernel object */
    rt_system_object_init();

    /* 定时器管理系统初始化 */
    rt_system_timer_init();

    rt_system_heap_init((void*)STM32_SRAM_BEGIN, (void*)STM32_SRAM_END);

    /* 初始化线程调度器 */
    rt_system_scheduler_init();

    /* 用户自定义初始化 */
    rt_application_init();

#ifdef RT_USING_FINSH
    /* init finsh */
    finsh_system_init();
    // 使用UART2作为FINSH SHELL端口
    finsh_set_device( FINSH_DEVICE_NAME );
#endif

    /* 初始化软件定时器 */
    rt_system_timer_thread_init();

    /* 初始化空闲线程 */
    rt_thread_idle_init();

    /* 启动线程调度器 */
    rt_system_scheduler_start();

    return ;
}

int main(void)
{
    /* disable interrupt first */
    rt_hw_interrupt_disable();

    /* startup RT-Thread RTOS */
    rtthread_startup();

    return 0;
}

/*@}*/
