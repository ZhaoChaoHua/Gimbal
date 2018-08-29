#include "idle_thread.h"

#define CPU_USAGE_CALC_TICK	10
#define CPU_USAGE_LOOP		100

static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;


/* CPU使用率检测钩子函数 */
static void cpu_usage_idle_hook(void)
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;

    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count ++;
            loop = 0;

            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }

    count = 0;
    /* 获取CPU使用率 */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }

    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
        cpu_usage_major = (count * 100) / total_count;
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
    }
    else
    {
        total_count = count;

        /* CPU没有使用 */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
    }
}

// 设置空线程钩子函数
void idle_hook_init(void)
{
    rt_thread_idle_sethook(cpu_usage_idle_hook);
}

// 使用finsh调试
#ifdef RT_USING_FINSH
#include <finsh.h>

// CPU占用率读取
void cpu_usage(void)
{
    char buf[20];
    rt_sprintf(buf,"%d.%d%%",cpu_usage_major,cpu_usage_minor);
    rt_kprintf(" CPU usage: %s \r\n", buf);
}
FINSH_FUNCTION_EXPORT(cpu_usage, get the usage of CPU.)

#endif

