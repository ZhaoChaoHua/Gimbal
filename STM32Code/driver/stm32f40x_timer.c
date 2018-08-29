#include "stm32f40x_timer.h"

static timer_dev timer1 = {
    .TIMx         = TIM1,
    .clk	      = RCC_APB2Periph_TIM1,
    .clkcmd       = RCC_APB2PeriphClockCmd,
    .type         = TIMER_ADVANCED,
    .handlers     = {0},
};
/** Timer 1 device (advanced) */
timer_dev* TIMER1 = &timer1;

static timer_dev timer2 = {
    .TIMx         = TIM2,
    .clk    	  = RCC_APB1Periph_TIM2,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_GENERAL,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 2 device (general-purpose) */
timer_dev* TIMER2 = &timer2;

static timer_dev timer3 = {
    .TIMx         = TIM3,
    .clk	      = RCC_APB1Periph_TIM3,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_GENERAL,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 3 device (general-purpose) */
timer_dev* TIMER3 = &timer3;

static timer_dev timer4 = {
    .TIMx         = TIM4,
    .clk       	  = RCC_APB1Periph_TIM4,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_GENERAL,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 4 device (general-purpose) */
timer_dev* TIMER4 = &timer4;

static timer_dev timer5 = {
    .TIMx         = TIM5,
    .clk          = RCC_APB1Periph_TIM5,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_GENERAL,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 5 device (general-purpose) */
timer_dev* TIMER5 = &timer5;

static timer_dev timer6 = {
    .TIMx         = TIM6,
    .clk          = RCC_APB1Periph_TIM6,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_BASIC,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 6 device (basic) */
timer_dev* TIMER6 = &timer6;

static timer_dev timer7 = {
    .TIMx         = TIM7,
    .clk          = RCC_APB1Periph_TIM7,
    .clkcmd       = RCC_APB1PeriphClockCmd,
    .type         = TIMER_BASIC,
    .priority     = 20,
    .handlers     = {0},
};
/** Timer 7 device (basic) */
timer_dev* TIMER7 = &timer7;

static timer_dev timer8 = {
    .TIMx         = TIM8,
    .clk	      = RCC_APB2Periph_TIM8,
    .clkcmd       = RCC_APB2PeriphClockCmd,
    .type         = TIMER_ADVANCED,              // 高级模式
    .handlers     = {0},
};
/** Timer 8 device (advanced) */
timer_dev* TIMER8 = &timer8;

/**
 * @brief Configure a channel's output compare mode.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to configure in output compare mode.
 * @param mode Timer mode to set.
 * @param flags OR of timer_oc_mode_flags.
 * @see timer_oc_mode_flags
 * @see timer_oc_mode_flags
 */
static void timer_oc_set_mode(timer_dev *dev, uint8_t channel, uint16_t mode, uint16_t flags)
{
	uint16_t TIM_OCMode;
	TIM_OCInitTypeDef config;
	TIM_OCStructInit(&config);

	if (mode & TIMER_OC_MODE_PWM_1)
	{
		TIM_OCMode = TIM_OCMode_PWM1;
		//LASER_PWM_PATCH
		config.TIM_OutputState = TIM_OutputState_Enable;
		config.TIM_OCPolarity = TIM_OCPolarity_High;
	}
	if (mode & TIMER_OC_MODE_PWM_2) 			{ TIM_OCMode = TIM_OCMode_PWM2; }
	if (mode & TIMER_OC_MODE_FORCE_INACTIVE) 	{ TIM_OCMode = TIM_ForcedAction_InActive; }
	if (mode & TIMER_OC_MODE_FORCE_ACTIVE) 		{ TIM_OCMode = TIM_ForcedAction_Active; }
	if (mode & TIMER_OC_MODE_TOGGLE) 			{ TIM_OCMode = TIM_OCMode_Toggle; }
	if (mode & TIMER_OC_MODE_ACTIVE_ON_MATCH) 	{ TIM_OCMode = TIMER_OC_MODE_ACTIVE_ON_MATCH; }
	if (mode & TIMER_OC_MODE_INACTIVE_ON_MATCH) { TIM_OCMode = TIMER_OC_MODE_INACTIVE_ON_MATCH; }
	if (mode & TIMER_OC_MODE_FROZEN) 			{ TIM_OCMode = TIM_OCMode_Timing; }

    switch (channel)
    {
	case 1:
		TIM_OC1Init(dev->TIMx, &config);
		TIM_SelectOCxM(dev->TIMx, TIM_Channel_1, TIM_OCMode);
		if (flags & TIMER_OC_CE) { TIM_ClearOC1Ref(dev->TIMx, TIM_OCClear_Enable); }
		if (flags & TIMER_OC_PE) { TIM_OC1PreloadConfig(dev->TIMx, TIM_OCPreload_Enable); }
		if (flags & TIMER_OC_FE) { TIM_OC1FastConfig(dev->TIMx, TIM_OCFast_Enable); }
		TIM_CCxCmd(dev->TIMx, TIM_Channel_1, TIM_CCx_Enable);
		break;
	case 2:
		TIM_OC2Init(dev->TIMx, &config);
		TIM_SelectOCxM(dev->TIMx, TIM_Channel_2, TIM_OCMode);
		if (flags & TIMER_OC_CE) { TIM_ClearOC2Ref(dev->TIMx, TIM_OCClear_Enable); }
		if (flags & TIMER_OC_PE) { TIM_OC2PreloadConfig(dev->TIMx, TIM_OCPreload_Enable); }
		if (flags & TIMER_OC_FE) { TIM_OC2FastConfig(dev->TIMx, TIM_OCFast_Enable); }
		TIM_CCxCmd(dev->TIMx, TIM_Channel_2, TIM_CCx_Enable);
		break;
	case 3:
		TIM_OC3Init(dev->TIMx, &config);
		TIM_SelectOCxM(dev->TIMx, TIM_Channel_3, TIM_OCMode);
		if (flags & TIMER_OC_CE) { TIM_ClearOC3Ref(dev->TIMx, TIM_OCClear_Enable); }
		if (flags & TIMER_OC_PE) { TIM_OC3PreloadConfig(dev->TIMx, TIM_OCPreload_Enable); }
		if (flags & TIMER_OC_FE) { TIM_OC3FastConfig(dev->TIMx, TIM_OCFast_Enable); }
		TIM_CCxCmd(dev->TIMx, TIM_Channel_3, TIM_CCx_Enable);
		break;
	case 4:
		TIM_OC4Init(dev->TIMx, &config);
		TIM_SelectOCxM(dev->TIMx, TIM_Channel_4, TIM_OCMode);
		if (flags & TIMER_OC_CE) { TIM_ClearOC4Ref(dev->TIMx, TIM_OCClear_Enable); }
		if (flags & TIMER_OC_PE) { TIM_OC4PreloadConfig(dev->TIMx, TIM_OCPreload_Enable); }
		if (flags & TIMER_OC_FE) { TIM_OC4FastConfig(dev->TIMx, TIM_OCFast_Enable); }
		TIM_CCxCmd(dev->TIMx, TIM_Channel_4, TIM_CCx_Enable);
		break;
    }
}

/**
 * Default a timer, and reset its register map.
 * @param dev Timer to initialize
 */
static void TimerDefaultConfig(timer_dev *dev)
{
	const uint16_t full_overflow = 0xFFFF;
    const uint16_t half_duty = 0x8FFF;
	uint8_t channel;
	 
	// Enable timer clock
	dev->clkcmd(dev->clk, ENABLE);
	TIM_DeInit(dev->TIMx);
	// Stop a timer's counter from changing
	TIM_Cmd(dev->TIMx, DISABLE);
	
	dev->TIMx->CR1 = TIMER_CR1_ARPE; // 允许自动重转载
    dev->TIMx->PSC = 1;              // 分频系数1
    dev->TIMx->SR = 0;               // 状态寄存器
    dev->TIMx->DIER = 0;             // 中断使能
    dev->TIMx->EGR = TIMER_EGR_UG;   // 产生更新事件时初始化定时器
	
	switch (dev->type)
	{
    case TIMER_ADVANCED:    // 高级模式
    	//TIMER_BDTR_MOE equivale a  TIM_CtrlPWMOutputs(dev->regs, ENABLE);
    	dev->TIMx->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF;
	
    case TIMER_GENERAL:    // 一般模式
        dev->TIMx->ARR = full_overflow;  // 定时器重装载值 

        for (channel = 1; channel <= 4; channel++) 
		{
            timer_set_compare(dev, channel, half_duty);  // 设置每一个通道的比较值
            timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
        }
    case TIMER_BASIC:
        break;
    }
	// Start Timer counter
	TIM_Cmd(dev->TIMx, ENABLE);
}

/* 初始化所有需要初始化的定时器 */
void timer_foreach(void (*timer_init)(timer_dev*))
{
	timer_init(TIMER8); // PWM输出
}

/* 定时器初始化 */
void TimerConfigure()
{
	timer_foreach(TimerDefaultConfig);
}

/* ---------------------------重写定时器相关操作----------------------------- */
/* 使能基本定时器中断 */
static void enable_nonmuxed_irq(timer_dev *dev) 
{
	if (dev->TIMx == TIM2)	
    {
        NVIC_SetPriority(TIM2_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM2_IRQn); 
    }
	else if (dev->TIMx == TIM3) 
    {
        NVIC_SetPriority(TIM3_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM3_IRQn);
    }
	else if (dev->TIMx == TIM4) 
    {
        NVIC_SetPriority(TIM4_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM4_IRQn);
    }
	else if (dev->TIMx == TIM5) 
    {
        NVIC_SetPriority(TIM5_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM5_IRQn);
    }
	else if (dev->TIMx == TIM6) 
    {
        NVIC_SetPriority(TIM6_DAC_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
	else if (dev->TIMx == TIM7) 
    {
        NVIC_SetPriority(TIM7_IRQn,dev->priority);
        NVIC_EnableIRQ(TIM7_IRQn);
    }
	else
	{
        assert_param(0);
    }
}

/* 使能定时器中断 */
static void enable_irq(timer_dev *dev, uint8_t iid) 
{
    if (dev->type != TIMER_ADVANCED) 
	{
        enable_nonmuxed_irq(dev);
    }
}

/**
 * @brief Attach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to attach to; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @param handler Handler to attach to the given interrupt.
 * @see timer_irq_ch_id
 * @see timer_channel
 */
void timer_attach_interrupt(timer_dev *dev,uint8_t interrupt,voidFuncPtr handler) 
{
    dev->handlers[interrupt] = handler;
    timer_enable_irq(dev, interrupt);
    enable_irq(dev, interrupt);
}
					
/**
 * @brief Detach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to detach; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_detach_interrupt(timer_dev *dev, uint8_t interrupt)
{
    timer_disable_irq(dev, interrupt);
    dev->handlers[interrupt] = RT_NULL;
}

static void disable_channel(timer_dev *dev, uint8_t channel) 
{
	switch(channel)
	{
		case 1:
			timer_detach_interrupt(dev, TIMER_CC1);
			break;
		case 2:
			timer_detach_interrupt(dev, TIMER_CC2);
			break;
		case 3:
			timer_detach_interrupt(dev, TIMER_CC3);
			break;
		case 4:
			timer_detach_interrupt(dev, TIMER_CC4);
			break;
		default:
			break;
	}
    timer_cc_disable(dev, channel);
}

static void pwm_mode(timer_dev *dev, uint8_t channel) 
{
    timer_disable_irq(dev, channel);
    //timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);

    //LASER_PWM_PATCH
	//TIM_OCInitTypeDef TIM_OCInitStructure;
	//TIM_OCStructInit(&TIM_OCInitStructure);
	//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (channel)
    {
		case 1:
			//TIM_OC1Init(dev->TIMx, &TIM_OCInitStructure);
			TIM_SelectOCxM(dev->TIMx, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_OC1PreloadConfig(dev->TIMx, TIM_OCPreload_Enable);
			break;
		case 2:
			//TIM_OC2Init(dev->TIMx, &TIM_OCInitStructure);
			TIM_SelectOCxM(dev->TIMx, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_OC2PreloadConfig(dev->TIMx, TIM_OCPreload_Enable);
			break;
		case 3:
			//TIM_OC3Init(dev->TIMx, &TIM_OCInitStructure);
			TIM_SelectOCxM(dev->TIMx, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_OC3PreloadConfig(dev->TIMx, TIM_OCPreload_Enable);
			break;
		case 4:
			//TIM_OC4Init(dev->TIMx, &TIM_OCInitStructure);
			TIM_SelectOCxM(dev->TIMx, TIM_Channel_4, TIM_OCMode_PWM1);
			TIM_OC4PreloadConfig(dev->TIMx, TIM_OCPreload_Enable);
			break;
    } 
    timer_cc_enable(dev, channel);
}

static void output_compare_mode(timer_dev *dev, uint8_t channel) 
{
	// TODO
	//timer_oc_set_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
    timer_cc_enable(dev, channel);
}

/**
 * Sets the mode of an individual timer channel.
 *
 * Note that not all timers can be configured in every mode.  For
 * example, basic timers cannot be configured to output compare mode.
 * Be sure to use a timer which is appropriate for the mode you want.
 *
 * @param dev Timer whose channel mode to set
 * @param channel Relevant channel
 * @param mode New timer mode for channel
 */
void timer_set_mode(timer_dev *dev, uint8_t channel, timer_mode mode) 
{
    assert_param(channel <= 4);

    /* TODO decide about the basic timers */
    assert_param(dev->type != TIMER_BASIC);
    if (dev->type == TIMER_BASIC)
        return;
	
    switch (mode) 
	{
    case TIMER_DISABLED:
        disable_channel(dev, channel);
        break;
    case TIMER_PWM:
        pwm_mode(dev, channel);
        break;
    case TIMER_OUTPUT_COMPARE:
        output_compare_mode(dev, channel);
        break;
    }
}
