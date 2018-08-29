/**
  ******************************************************************************
  * @file    IO_Toggle/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <rtthread.h>
#include "board.h"
#include "stm32f40x_timer.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
      
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}


void Handle_IRQ(timer_dev *dev, uint16_t interrupt_statu, uint8_t irq_id)
{
	if (TIM_GetITStatus(dev->TIMx, interrupt_statu) != RESET) 
	{
		TIM_ClearITPendingBit(dev->TIMx, interrupt_statu ); 
		void(* _handler)(void) = dev->handlers[irq_id];
		if(_handler)
		{
			_handler();
		}
	}
}

void TIM2_IRQHandler(void) 
{
	Handle_IRQ(TIMER2, TIM_IT_Update,TIMER_UPDATE);
	Handle_IRQ(TIMER2, TIM_IT_CC1,   TIMER_CC1);
	Handle_IRQ(TIMER2, TIM_IT_CC2,   TIMER_CC2);
	Handle_IRQ(TIMER2, TIM_IT_CC3,   TIMER_CC3);
	Handle_IRQ(TIMER2, TIM_IT_CC4,   TIMER_CC4);
}

void TIM3_IRQHandler(void) 
{
	Handle_IRQ(TIMER3, TIM_IT_Update,TIMER_UPDATE);
	Handle_IRQ(TIMER3, TIM_IT_CC1,   TIMER_CC1);
	Handle_IRQ(TIMER3, TIM_IT_CC2,   TIMER_CC2);
	Handle_IRQ(TIMER3, TIM_IT_CC3,   TIMER_CC3);
	Handle_IRQ(TIMER3, TIM_IT_CC4,   TIMER_CC4);
}

void TIM4_IRQHandler(void)
{
	Handle_IRQ(TIMER4, TIM_IT_Update,TIMER_UPDATE);
	Handle_IRQ(TIMER4, TIM_IT_CC1,   TIMER_CC1);
	Handle_IRQ(TIMER4, TIM_IT_CC2,   TIMER_CC2);
	Handle_IRQ(TIMER4, TIM_IT_CC3,   TIMER_CC3);
	Handle_IRQ(TIMER4, TIM_IT_CC4,   TIMER_CC4);
}

void TIM5_IRQHandler(void)
{
	Handle_IRQ(TIMER5, TIM_IT_Update,TIMER_UPDATE);
	Handle_IRQ(TIMER5, TIM_IT_CC1,   TIMER_CC1);
	Handle_IRQ(TIMER5, TIM_IT_CC2,   TIMER_CC2);
	Handle_IRQ(TIMER5, TIM_IT_CC3,   TIMER_CC3);
	Handle_IRQ(TIMER5, TIM_IT_CC4,   TIMER_CC4);
}

void TIM6_DAC_IRQHandler(void)
{
	Handle_IRQ(TIMER6, TIM_IT_Update,TIMER_UPDATE);
	Handle_IRQ(TIMER6, TIM_IT_CC1,   TIMER_CC1);
	Handle_IRQ(TIMER6, TIM_IT_CC2,   TIMER_CC2);
	Handle_IRQ(TIMER6, TIM_IT_CC3,   TIMER_CC3);
	Handle_IRQ(TIMER6, TIM_IT_CC4,   TIMER_CC4);
}

/*
void TIM7_IRQHandler(void)
{
	rt_interrupt_enter();

	rt_interrupt_leave();	
}
*/

void I2C1_EV_IRQHandler(void)
{
	
}
void I2C1_ER_IRQHandler(void)
{
	
}

void I2C2_EV_IRQHandler(void)
{
	
    
}
void I2C2_ER_IRQHandler(void)
{
	
    
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
