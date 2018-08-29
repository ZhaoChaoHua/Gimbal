/*
 * File      : gpio.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-05     Bernard      the first version
 */
#ifndef _GPIO_H__
#define _GPIO_H__

#include "stm32f40x_define.h"
#include "position_sen.h"
#include "stm32f40x_timer.h"

typedef void (*voidFuncPtr)(void);
typedef void (*rcc_clockcmd)(uint32_t, FunctionalState);

#define BOARD_NR_GPIO_PINS      80
#define GPIO_DEFAULT_SPEED      GPIO_Speed_100MHz

#define PA0                     0
#define PA1                     1
#define PA2                     2
#define PA3                     3
#define PA4                     4
#define PA5                     5
#define PA6                     6
#define PA7                     7
#define PA8                     8
#define PA9                     9
#define PA10                    10
#define PA11                    11
#define PA12                    12
#define PA13                    13
#define PA14                    14
#define PA15                    15

#define PB0                     16
#define PB1                     17
#define PB2                     18
#define PB3                     19
#define PB4                     20
#define PB5                     21
#define PB6                     22
#define PB7                     23
#define PB8                     24
#define PB9                     25
#define PB10                    26
#define PB11                    27
#define PB12                    28
#define PB13                    29
#define PB14                    30
#define PB15                    31

#define PC0                     32
#define PC1                     33
#define PC2                     34
#define PC3                     35
#define PC4                     36
#define PC5                     37
#define PC6                     38
#define PC7                     39
#define PC8                     40
#define PC9                     41
#define PC10                    42
#define PC11                    43
#define PC12                    44
#define PC13                    45
#define PC14                    46
#define PC15                    47

#define PD0                     48
#define PD1                     49
#define PD2                     50
#define PD3                     51
#define PD4                     52
#define PD5                     53
#define PD6                     54
#define PD7                     55
#define PD8                     56
#define PD9                     57
#define PD10                    58
#define PD11                    59
#define PD12                    60
#define PD13                    61
#define PD14                    62
#define PD15                    63

#define PE0                     64
#define PE1                     65
#define PE2                     66
#define PE3                     67
#define PE4                     68
#define PE5                     69
#define PE6                     70
#define PE7                     71
#define PE8                     72
#define PE9                     73
#define PE10                    74
#define PE11                    75
#define PE12                    76
#define PE13                    77
#define PE14                    78
#define PE15                    79

/**
 * Invalid stm32_pin_info adc_channel value.
 * @see stm32_pin_info
 */
#define ADCx                    0xFF

/**
 * @brief GPIO Pin modes.
 *
 * These only allow for 50MHZ max output speeds; if you want slower,
 * use direct register access.
 */
typedef enum gpio_pin_mode {
    GPIO_OUTPUT_PP, 		/**< Output push-pull. */
    GPIO_OUTPUT_OD, 		/**< Output open-drain. */
    GPIO_AF_OUTPUT_PP, 		/**< Alternate function output push-pull. */
    GPIO_AF_OUTPUT_OD, 		/**< Alternate function output open drain. */
    GPIO_INPUT_ANALOG, 		/**< Analog input. */
    GPIO_INPUT_FLOATING, 	/**< Input floating. */
    GPIO_INPUT_PD, 			/**< Input pull-down. */
    GPIO_INPUT_PU, 			/**< Input pull-up. */
    COM_MODE
    /* GPIO_INPUT_PU treated as a special case, for ODR twiddling */
} gpio_pin_mode;

/** GPIO device type */
typedef struct gpio_dev_structure {
    GPIO_TypeDef*  GPIOx; 		    /**< Register map */
    uint32_t       clk;		        /**< RCC clock information */
    rcc_clockcmd   clkcmd;
} gpio_dev;

/**
 * Specifies a GPIO pin behavior.
 * @see pinMode()
 */
typedef enum {
    OUTPUT, /**< Basic digital output: when the pin is HIGH, the
               voltage is held at +3.3v (Vcc) and when it is LOW, it
               is pulled down to ground. */

    OUTPUT_OPEN_DRAIN, /**< In open drain mode, the pin indicates
                          "low" by accepting current flow to ground
                          and "high" by providing increased
                          impedance. An example use would be to
                          connect a pin to a bus line (which is pulled
                          up to a positive voltage by a separate
                          supply through a large resistor). When the
                          pin is high, not much current flows through
                          to ground and the line stays at positive
                          voltage; when the pin is low, the bus
                          "drains" to ground with a small amount of
                          current constantly flowing through the large
                          resistor from the external supply. In this
                          mode, no current is ever actually sourced
                          from the pin. */

    INPUT, /**< Basic digital input. The pin voltage is sampled; when
              it is closer to 3.3v (Vcc) the pin status is high, and
              when it is closer to 0v (ground) it is low. If no
              external circuit is pulling the pin voltage to high or
              low, it will tend to randomly oscillate and be very
              sensitive to noise (e.g., a breath of air across the pin
              might cause the state to flip). */

    INPUT_ANALOG, /**< This is a special mode for when the pin will be
                     used for analog (not digital) reads.  Enables ADC
                     conversion to be performed on the voltage at the
                     pin. */

    INPUT_PULLUP, /**< The state of the pin in this mode is reported
                     the same way as with INPUT, but the pin voltage
                     is gently "pulled up" towards +3.3v. This means
                     the state will be high unless an external device
                     is specifically pulling the pin down to ground,
                     in which case the "gentle" pull up will not
                     affect the state of the input. */

    INPUT_PULLDOWN, /**< The state of the pin in this mode is reported
                       the same way as with INPUT, but the pin voltage
                       is gently "pulled down" towards 0v. This means
                       the state will be low unless an external device
                       is specifically pulling the pin up to 3.3v, in
                       which case the "gentle" pull down will not
                       affect the state of the input. */

    INPUT_FLOATING, /**< Synonym for INPUT. */

    AF_OUTPUT, /**< This is a special mode for when the pin will be used for
            PWM output (a special case of digital output). */

    PWM_OPEN_DRAIN, /**< Like PWM, except that instead of alternating
                       cycles of LOW and HIGH, the voltage on the pin
                       consists of alternating cycles of LOW and
                       floating (disconnected). */
    TRANS_PIN_MODE,
} GPIO_PinMode;

/**
 * @brief Stores STM32-specific information related to a given Maple pin.
 * @see PIN_MAP
 */
typedef struct stm32_pin_info_structure {
    gpio_dev        *gpio_device;             /**< Maple pin's GPIO device */
    timer_dev       *timer_device;            /**< Pin's timer device, if any. */
    const adc_dev   *adc_device;              /**< ADC device, if any. */
    uint16_t         gpio_bit;                 /**< Pin's GPIO port bit. */
    uint8_t          timer_channel;        	  /**< Timer channel, or 0 if none. */
    uint8_t          adc_channel;              /**< Pin ADC channel, or ADCx if none. */
    uint16_t         GPIO_PinSource;
    uint8_t          GPIO_AF;
} stm32_pin_info;


extern stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS];
void init_gpio_map(void);
void Set_pinMode(uint8_t pin, GPIO_PinMode mode);

void GPIO_OUT(uint8_t pin, uint8_t value);

#endif
