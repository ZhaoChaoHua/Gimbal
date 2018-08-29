#ifndef __LED_H__
#define __LED_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif


#define led1_rcc                    RCC_AHB1Periph_GPIOB
#define led1_gpio                   GPIOB
#define led1_pin                    (GPIO_Pin_4)

#define led2_rcc                    RCC_AHB1Periph_GPIOB
#define led2_gpio                   GPIOB
#define led2_pin                    (GPIO_Pin_3)

#define led3_rcc                    RCC_AHB1Periph_GPIOD
#define led3_gpio                   GPIOD
#define led3_pin                    (GPIO_Pin_7)

#define led4_rcc                    RCC_AHB1Periph_GPIOD
#define led4_gpio                   GPIOD
#define led4_pin                    (GPIO_Pin_15)
    
struct rt_device_led_status
{
    rt_uint16_t led;
    rt_uint16_t status;
};

/* led驱动操作RT-Thread */
struct rt_device_led
{
    struct rt_device parent;
    const struct rt_led_ops *ops;
};

// 字符设备结构体
struct rt_led_ops
{
    void (*led_write_ops)(struct rt_device *device, rt_base_t led, rt_bool_t value);// 控制LED
};

int rt_device_led_register(const char *name, const struct rt_led_ops *ops, void *user_data);

void sprain_led_write(rt_device_t dev, rt_base_t pin, rt_bool_t value);
int led_hw_init(void);

#ifdef __cplusplus
}
#endif

#endif
