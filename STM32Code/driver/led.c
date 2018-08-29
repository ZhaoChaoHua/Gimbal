/*
 * File      : led.c
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
 */
#include <rtthread.h>
#include <stm32f4xx.h>
#include "led.h"

// 设备驱动
static struct rt_device_led hw_led;

const static struct rt_led_ops _sprain_led_ops =
{
    sprain_led_write,
};

// 设备初始化
static rt_err_t _led_init(rt_device_t dev)
{
    return RT_EOK;
}

// 打开LED设备
static rt_err_t _led_open(rt_device_t dev, rt_uint16_t oflag)
{
    RT_ASSERT(dev != RT_NULL);
    return RT_EOK;
}

// 关闭LED设备
static rt_err_t _led_close(rt_device_t dev)
{
    RT_ASSERT(dev != RT_NULL);
    return RT_EOK;
}

// led状态读取
static rt_size_t _led_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct rt_device_led_status *status;
    struct rt_device_led *led = (struct rt_device_led *)dev;

    /* check parameters */
    RT_ASSERT(led != RT_NULL);

    status = (struct rt_device_led_status *) buffer;
    if (status == RT_NULL || size != sizeof(*status)) return 0;

    //status->status = led->ops->led_read_ops(dev, status->led);
    return size;
}

// led数据写入
static rt_size_t _led_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_bool_t *status;
    struct rt_device_led *led = (struct rt_device_led *)dev;

    /* check parameters */
    RT_ASSERT(led != RT_NULL);

    status = (rt_bool_t *) buffer;

    led->ops->led_write_ops(dev, pos, *status);

    return size;
}

// 注册设备
int rt_device_led_register(const char *name, const struct rt_led_ops *ops, void *user_data)
{
    
    hw_led.parent.type         = RT_Device_Class_Char;
    hw_led.parent.rx_indicate  = RT_NULL;
    hw_led.parent.tx_complete  = RT_NULL;

    hw_led.parent.init         = _led_init;
    hw_led.parent.open         = _led_open;
    hw_led.parent.close        = _led_close;
    hw_led.parent.read         = _led_read;
    hw_led.parent.write        = _led_write;
    hw_led.parent.control      = RT_NULL;

    hw_led.ops                 = ops;
    hw_led.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&hw_led.parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
}

// led驱动初始化
int led_hw_init()
{
    // 硬件初始化
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(led1_rcc|led2_rcc|led3_rcc|led4_rcc,ENABLE);

    GPIO_InitStructure.GPIO_Pin   = led1_pin|led2_pin|led3_pin|led4_pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    
    GPIO_Init(led1_gpio, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_3);
    
    // 注册设备驱动
    rt_device_led_register("led_dev",&_sprain_led_ops,RT_NULL);
    return 0;
}

// OPS写函数
void sprain_led_write(rt_device_t dev, rt_base_t led, rt_bool_t value)
{
    RT_ASSERT(dev != RT_NULL)
    switch (led)
    {
    case 0:
        if(value == RT_TRUE)
            GPIO_SetBits(led1_gpio, led1_pin);
        else
            GPIO_ResetBits(led1_gpio, led1_pin);
        break;
    case 1:
        if(value == RT_TRUE)
            GPIO_SetBits(led2_gpio, led2_pin);
        else
            GPIO_ResetBits(led2_gpio, led2_pin);
        break;
    case 2:
        if(value == RT_TRUE)
            GPIO_SetBits(led3_gpio, led3_pin);
        else
            GPIO_ResetBits(led3_gpio, led3_pin);
        break;
    case 3:
        if(value == RT_TRUE)
            GPIO_SetBits(led4_gpio, led4_pin);
        else
            GPIO_ResetBits(led4_gpio, led4_pin);
        break;
    default:
        break;
    }
}
