#ifndef _STM32F40X_TIMER_H__
#define _STM32F40X_TIMER_H__

#include "stm32f40x_define.h"

#define NR_ADV_HANDLERS                 8
/* Update, capture/compare 1,2,3,4; <junk>; trigger. */
#define NR_GEN_HANDLERS                 7
/* Update only. */
#define NR_BAS_HANDLERS                 1

/* -----------------------------定时器寄存器控制------------------------------- */
/*
 * Register bit definitions
 */

/* Control register 1 (CR1) */

#define TIMER_CR1_ARPE_BIT              7
#define TIMER_CR1_DIR_BIT               4
#define TIMER_CR1_OPM_BIT               3
#define TIMER_CR1_URS_BIT               2
#define TIMER_CR1_UDIS_BIT              1
#define TIMER_CR1_CEN_BIT               0

#define TIMER_CR1_CKD                   (0x3 << 8)
#define TIMER_CR1_CKD_1TCKINT           (0x0 << 8)
#define TIMER_CR1_CKD_2TCKINT           (0x1 << 8)
#define TIMER_CR1_CKD_4TICKINT          (0x2 << 8)
#define TIMER_CR1_ARPE                  BIT(TIMER_CR1_ARPE_BIT)
#define TIMER_CR1_CKD_CMS               (0x3 << 5)
#define TIMER_CR1_CKD_CMS_EDGE          (0x0 << 5)
#define TIMER_CR1_CKD_CMS_CENTER1       (0x1 << 5)
#define TIMER_CR1_CKD_CMS_CENTER2       (0x2 << 5)
#define TIMER_CR1_CKD_CMS_CENTER3       (0x3 << 5)
#define TIMER_CR1_DIR                   BIT(TIMER_CR1_DIR_BIT)
#define TIMER_CR1_OPM                   BIT(TIMER_CR1_OPM_BIT)
#define TIMER_CR1_URS                   BIT(TIMER_CR1_URS_BIT)
#define TIMER_CR1_UDIS                  BIT(TIMER_CR1_UDIS_BIT)
#define TIMER_CR1_CEN                   BIT(TIMER_CR1_CEN_BIT)

/* Control register 2 (CR2) */

#define TIMER_CR2_OIS4_BIT              14
#define TIMER_CR2_OIS3N_BIT             13
#define TIMER_CR2_OIS3_BIT              12
#define TIMER_CR2_OIS2N_BIT             11
#define TIMER_CR2_OIS2_BIT              10
#define TIMER_CR2_OIS1N_BIT             9
#define TIMER_CR2_OIS1_BIT              8
#define TIMER_CR2_TI1S_BIT              7 /* tills? yikes */
#define TIMER_CR2_CCDS_BIT              3
#define TIMER_CR2_CCUS_BIT              2
#define TIMER_CR2_CCPC_BIT              0

#define TIMER_CR2_OIS4                  BIT(TIMER_CR2_OIS4_BIT)
#define TIMER_CR2_OIS3N                 BIT(TIMER_CR2_OIS3N_BIT)
#define TIMER_CR2_OIS3                  BIT(TIMER_CR2_OIS3_BIT)
#define TIMER_CR2_OIS2N                 BIT(TIMER_CR2_OIS2N_BIT)
#define TIMER_CR2_OIS2                  BIT(TIMER_CR2_OIS2_BIT)
#define TIMER_CR2_OIS1N                 BIT(TIMER_CR2_OIS1N_BIT)
#define TIMER_CR2_OIS1                  BIT(TIMER_CR2_OIS1_BIT)
#define TIMER_CR2_TI1S                  BIT(TIMER_CR2_TI1S_BIT)
#define TIMER_CR2_MMS                   (0x7 << 4)
#define TIMER_CR2_MMS_RESET             (0x0 << 4)
#define TIMER_CR2_MMS_ENABLE            (0x1 << 4)
#define TIMER_CR2_MMS_UPDATE            (0x2 << 4)
#define TIMER_CR2_MMS_COMPARE_PULSE     (0x3 << 4)
#define TIMER_CR2_MMS_COMPARE_OC1REF    (0x4 << 4)
#define TIMER_CR2_MMS_COMPARE_OC2REF    (0x5 << 4)
#define TIMER_CR2_MMS_COMPARE_OC3REF    (0x6 << 4)
#define TIMER_CR2_MMS_COMPARE_OC4REF    (0x7 << 4)
#define TIMER_CR2_CCDS                  BIT(TIMER_CR2_CCDS_BIT)
#define TIMER_CR2_CCUS                  BIT(TIMER_CR2_CCUS_BIT)
#define TIMER_CR2_CCPC                  BIT(TIMER_CR2_CCPC_BIT)

/* Slave mode control register (SMCR) */

#define TIMER_SMCR_ETP_BIT              15
#define TIMER_SMCR_ECE_BIT              14
#define TIMER_SMCR_MSM_BIT              7

#define TIMER_SMCR_ETP                  BIT(TIMER_SMCR_ETP_BIT)
#define TIMER_SMCR_ECE                  BIT(TIMER_SMCR_ECE_BIT)
#define TIMER_SMCR_ETPS                 (0x3 << 12)
#define TIMER_SMCR_ETPS_OFF             (0x0 << 12)
#define TIMER_SMCR_ETPS_DIV2            (0x1 << 12)
#define TIMER_SMCR_ETPS_DIV4            (0x2 << 12)
#define TIMER_SMCR_ETPS_DIV8            (0x3 << 12)
#define TIMER_SMCR_ETF                  (0xF << 12)
#define TIMER_SMCR_MSM                  BIT(TIMER_SMCR_MSM_BIT)
#define TIMER_SMCR_TS                   (0x3 << 4)
#define TIMER_SMCR_TS_ITR0              (0x0 << 4)
#define TIMER_SMCR_TS_ITR1              (0x1 << 4)
#define TIMER_SMCR_TS_ITR2              (0x2 << 4)
#define TIMER_SMCR_TS_ITR3              (0x3 << 4)
#define TIMER_SMCR_TS_TI1F_ED           (0x4 << 4)
#define TIMER_SMCR_TS_TI1FP1            (0x5 << 4)
#define TIMER_SMCR_TS_TI2FP2            (0x6 << 4)
#define TIMER_SMCR_TS_ETRF              (0x7 << 4)
#define TIMER_SMCR_SMS                  0x3
#define TIMER_SMCR_SMS_DISABLED         0x0
#define TIMER_SMCR_SMS_ENCODER1         0x1
#define TIMER_SMCR_SMS_ENCODER2         0x2
#define TIMER_SMCR_SMS_ENCODER3         0x3
#define TIMER_SMCR_SMS_RESET            0x4
#define TIMER_SMCR_SMS_GATED            0x5
#define TIMER_SMCR_SMS_TRIGGER          0x6
#define TIMER_SMCR_SMS_EXTERNAL         0x7

/* DMA/Interrupt enable register (DIER) */

#define TIMER_DIER_TDE_BIT              14
#define TIMER_DIER_CC4DE_BIT            12
#define TIMER_DIER_CC3DE_BIT            11
#define TIMER_DIER_CC2DE_BIT            10
#define TIMER_DIER_CC1DE_BIT            9
#define TIMER_DIER_UDE_BIT              8
#define TIMER_DIER_TIE_BIT              6
#define TIMER_DIER_CC4IE_BIT            4
#define TIMER_DIER_CC3IE_BIT            3
#define TIMER_DIER_CC2IE_BIT            2
#define TIMER_DIER_CC1IE_BIT            1
#define TIMER_DIER_UIE_BIT              0

#define TIMER_DIER_TDE                  BIT(TIMER_DIER_TDE_BIT)
#define TIMER_DIER_CC4DE                BIT(TIMER_DIER_CC4DE_BIT)
#define TIMER_DIER_CC3DE                BIT(TIMER_DIER_CC3DE_BIT)
#define TIMER_DIER_CC2DE                BIT(TIMER_DIER_CC2DE_BIT)
#define TIMER_DIER_CC1DE                BIT(TIMER_DIER_CC1DE_BIT)
#define TIMER_DIER_UDE                  BIT(TIMER_DIER_UDE_BIT)
#define TIMER_DIER_TIE                  BIT(TIMER_DIER_TIE_BIT)
#define TIMER_DIER_CC4IE                BIT(TIMER_DIER_CC4IE_BIT)
#define TIMER_DIER_CC3IE                BIT(TIMER_DIER_CC3IE_BIT)
#define TIMER_DIER_CC2IE                BIT(TIMER_DIER_CC2IE_BIT)
#define TIMER_DIER_CC1IE                BIT(TIMER_DIER_CC1IE_BIT)
#define TIMER_DIER_UIE                  BIT(TIMER_DIER_UIE_BIT)

/* Status register (SR) */

#define TIMER_SR_CC4OF_BIT              12
#define TIMER_SR_CC3OF_BIT              11
#define TIMER_SR_CC2OF_BIT              10
#define TIMER_SR_CC1OF_BIT              9
#define TIMER_SR_BIF_BIT                7
#define TIMER_SR_TIF_BIT                6
#define TIMER_SR_COMIF_BIT              5
#define TIMER_SR_CC4IF_BIT              4
#define TIMER_SR_CC3IF_BIT              3
#define TIMER_SR_CC2IF_BIT              2
#define TIMER_SR_CC1IF_BIT              1
#define TIMER_SR_UIF_BIT                0

#define TIMER_SR_CC4OF                  BIT(TIMER_SR_CC4OF_BIT)
#define TIMER_SR_CC3OF                  BIT(TIMER_SR_CC3OF_BIT)
#define TIMER_SR_CC2OF                  BIT(TIMER_SR_CC2OF_BIT)
#define TIMER_SR_CC1OF                  BIT(TIMER_SR_CC1OF_BIT)
#define TIMER_SR_BIF                    BIT(TIMER_SR_BIF_BIT)
#define TIMER_SR_TIF                    BIT(TIMER_SR_TIF_BIT)
#define TIMER_SR_COMIF                  BIT(TIMER_SR_COMIF_BIT)
#define TIMER_SR_CC4IF                  BIT(TIMER_SR_CC4IF_BIT)
#define TIMER_SR_CC3IF                  BIT(TIMER_SR_CC3IF_BIT)
#define TIMER_SR_CC2IF                  BIT(TIMER_SR_CC2IF_BIT)
#define TIMER_SR_CC1IF                  BIT(TIMER_SR_CC1IF_BIT)
#define TIMER_SR_UIF                    BIT(TIMER_SR_UIF_BIT)

/* Event generation register (EGR) */

#define TIMER_EGR_TG_BIT                6
#define TIMER_EGR_CC4G_BIT              4
#define TIMER_EGR_CC3G_BIT              3
#define TIMER_EGR_CC2G_BIT              2
#define TIMER_EGR_CC1G_BIT              1
#define TIMER_EGR_UG_BIT                0

#define TIMER_EGR_TG                    BIT(TIMER_EGR_TG_BIT)
#define TIMER_EGR_CC4G                  BIT(TIMER_EGR_CC4G_BIT)
#define TIMER_EGR_CC3G                  BIT(TIMER_EGR_CC3G_BIT)
#define TIMER_EGR_CC2G                  BIT(TIMER_EGR_CC2G_BIT)
#define TIMER_EGR_CC1G                  BIT(TIMER_EGR_CC1G_BIT)
#define TIMER_EGR_UG                    BIT(TIMER_EGR_UG_BIT)

/* Capture/compare mode registers, common values */

#define TIMER_CCMR_CCS_OUTPUT           0x0
#define TIMER_CCMR_CCS_INPUT_TI1        0x1
#define TIMER_CCMR_CCS_INPUT_TI2        0x2
#define TIMER_CCMR_CCS_INPUT_TRC        0x3

/* Capture/compare mode register 1 (CCMR1) */

#define TIMER_CCMR1_OC2CE_BIT           15
#define TIMER_CCMR1_OC2PE_BIT           11
#define TIMER_CCMR1_OC2FE_BIT           10
#define TIMER_CCMR1_OC1CE_BIT           7
#define TIMER_CCMR1_OC1PE_BIT           3
#define TIMER_CCMR1_OC1FE_BIT           2

#define TIMER_CCMR1_OC2CE               BIT(TIMER_CCMR1_OC2CE_BIT)
#define TIMER_CCMR1_OC2M                (0x3 << 12)
#define TIMER_CCMR1_IC2F                (0xF << 12)
#define TIMER_CCMR1_OC2PE               BIT(TIMER_CCMR1_OC2PE_BIT)
#define TIMER_CCMR1_OC2FE               BIT(TIMER_CCMR1_OC2FE_BIT)
#define TIMER_CCMR1_IC2PSC              (0x3 << 10)
#define TIMER_CCMR1_CC2S                (0x3 << 8)
#define TIMER_CCMR1_CC2S_OUTPUT         (TIMER_CCMR_CCS_OUTPUT << 8)
#define TIMER_CCMR1_CC2S_INPUT_TI1      (TIMER_CCMR_CCS_INPUT_TI1 << 8)
#define TIMER_CCMR1_CC2S_INPUT_TI2      (TIMER_CCMR_CCS_INPUT_TI2 << 8)
#define TIMER_CCMR1_CC2S_INPUT_TRC      (TIMER_CCMR_CCS_INPUT_TRC << 8)
#define TIMER_CCMR1_OC1CE               BIT(TIMER_CCMR1_OC1CE_BIT)
#define TIMER_CCMR1_OC1M                (0x3 << 4)
#define TIMER_CCMR1_IC1F                (0xF << 4)
#define TIMER_CCMR1_OC1PE               BIT(TIMER_CCMR1_OC1PE_BIT)
#define TIMER_CCMR1_OC1FE               BIT(TIMER_CCMR1_OC1FE_BIT)
#define TIMER_CCMR1_IC1PSC              (0x3 << 2)
#define TIMER_CCMR1_CC1S                0x3
#define TIMER_CCMR1_CC1S_OUTPUT         TIMER_CCMR_CCS_OUTPUT
#define TIMER_CCMR1_CC1S_INPUT_TI1      TIMER_CCMR_CCS_INPUT_TI1
#define TIMER_CCMR1_CC1S_INPUT_TI2      TIMER_CCMR_CCS_INPUT_TI2
#define TIMER_CCMR1_CC1S_INPUT_TRC      TIMER_CCMR_CCS_INPUT_TRC

/* Capture/compare mode register 2 (CCMR2) */

#define TIMER_CCMR2_OC4CE_BIT           15
#define TIMER_CCMR2_OC4PE_BIT           11
#define TIMER_CCMR2_OC4FE_BIT           10
#define TIMER_CCMR2_OC3CE_BIT           7
#define TIMER_CCMR2_OC3PE_BIT           3
#define TIMER_CCMR2_OC3FE_BIT           2

#define TIMER_CCMR2_OC4CE               BIT(TIMER_CCMR2_OC4CE_BIT)
#define TIMER_CCMR2_OC4M                (0x3 << 12)
#define TIMER_CCMR2_IC2F                (0xF << 12)
#define TIMER_CCMR2_OC4PE               BIT(TIMER_CCMR2_OC4PE_BIT)
#define TIMER_CCMR2_OC4FE               BIT(TIMER_CCMR2_OC4FE_BIT)
#define TIMER_CCMR2_IC2PSC              (0x3 << 10)
#define TIMER_CCMR2_CC4S                (0x3 << 8)
#define TIMER_CCMR1_CC4S_OUTPUT         (TIMER_CCMR_CCS_OUTPUT << 8)
#define TIMER_CCMR1_CC4S_INPUT_TI1      (TIMER_CCMR_CCS_INPUT_TI1 << 8)
#define TIMER_CCMR1_CC4S_INPUT_TI2      (TIMER_CCMR_CCS_INPUT_TI2 << 8)
#define TIMER_CCMR1_CC4S_INPUT_TRC      (TIMER_CCMR_CCS_INPUT_TRC << 8)
#define TIMER_CCMR2_OC3CE               BIT(TIMER_CCMR2_OC3CE_BIT)
#define TIMER_CCMR2_OC3M                (0x3 << 4)
#define TIMER_CCMR2_IC1F                (0xF << 4)
#define TIMER_CCMR2_OC3PE               BIT(TIMER_CCMR2_OC3PE_BIT)
#define TIMER_CCMR2_OC3FE               BIT(TIMER_CCMR2_OC3FE_BIT)
#define TIMER_CCMR2_IC1PSC              (0x3 << 2)
#define TIMER_CCMR2_CC3S                0x3
#define TIMER_CCMR1_CC3S_OUTPUT         TIMER_CCMR_CCS_OUTPUT
#define TIMER_CCMR1_CC3S_INPUT_TI1      TIMER_CCMR_CCS_INPUT_TI1
#define TIMER_CCMR1_CC3S_INPUT_TI2      TIMER_CCMR_CCS_INPUT_TI2
#define TIMER_CCMR1_CC3S_INPUT_TRC      TIMER_CCMR_CCS_INPUT_TRC

/* Capture/compare enable register (CCER) */

#define TIMER_CCER_CC4P_BIT             13
#define TIMER_CCER_CC4E_BIT             12
#define TIMER_CCER_CC3P_BIT             9
#define TIMER_CCER_CC3E_BIT             8
#define TIMER_CCER_CC2P_BIT             5
#define TIMER_CCER_CC2E_BIT             4
#define TIMER_CCER_CC1P_BIT             1
#define TIMER_CCER_CC1E_BIT             0

#define TIMER_CCER_CC4P                 BIT(TIMER_CCER_CC4P_BIT)
#define TIMER_CCER_CC4E                 BIT(TIMER_CCER_CC4E_BIT)
#define TIMER_CCER_CC3P                 BIT(TIMER_CCER_CC3P_BIT)
#define TIMER_CCER_CC3E                 BIT(TIMER_CCER_CC3E_BIT)
#define TIMER_CCER_CC2P                 BIT(TIMER_CCER_CC2P_BIT)
#define TIMER_CCER_CC2E                 BIT(TIMER_CCER_CC2E_BIT)
#define TIMER_CCER_CC1P                 BIT(TIMER_CCER_CC1P_BIT)
#define TIMER_CCER_CC1E                 BIT(TIMER_CCER_CC1E_BIT)

/* Break and dead-time register (BDTR) */

#define TIMER_BDTR_MOE_BIT              15
#define TIMER_BDTR_AOE_BIT              14
#define TIMER_BDTR_BKP_BIT              13
#define TIMER_BDTR_BKE_BIT              12
#define TIMER_BDTR_OSSR_BIT             11
#define TIMER_BDTR_OSSI_BIT             10

#define TIMER_BDTR_MOE                  BIT(TIMER_BDTR_MOE_BIT)
#define TIMER_BDTR_AOE                  BIT(TIMER_BDTR_AOE_BIT)
#define TIMER_BDTR_BKP                  BIT(TIMER_BDTR_BKP_BIT)
#define TIMER_BDTR_BKE                  BIT(TIMER_BDTR_BKE_BIT)
#define TIMER_BDTR_OSSR                 BIT(TIMER_BDTR_OSSR_BIT)
#define TIMER_BDTR_OSSI                 BIT(TIMER_BDTR_OSSI_BIT)
#define TIMER_BDTR_LOCK                 (0x3 << 8)
#define TIMER_BDTR_LOCK_OFF             (0x0 << 8)
#define TIMER_BDTR_LOCK_LEVEL1          (0x1 << 8)
#define TIMER_BDTR_LOCK_LEVEL2          (0x2 << 8)
#define TIMER_BDTR_LOCK_LEVEL3          (0x3 << 8)
#define TIMER_BDTR_DTG                  0xFF

/* DMA control register (DCR) */

#define TIMER_DCR_DBL                   (0x1F << 8)
#define TIMER_DCR_DBL_1BYTE             (0x0 << 8)
#define TIMER_DCR_DBL_2BYTE             (0x1 << 8)
#define TIMER_DCR_DBL_3BYTE             (0x2 << 8)
#define TIMER_DCR_DBL_4BYTE             (0x3 << 8)
#define TIMER_DCR_DBL_5BYTE             (0x4 << 8)
#define TIMER_DCR_DBL_6BYTE             (0x5 << 8)
#define TIMER_DCR_DBL_7BYTE             (0x6 << 8)
#define TIMER_DCR_DBL_8BYTE             (0x7 << 8)
#define TIMER_DCR_DBL_9BYTE             (0x8 << 8)
#define TIMER_DCR_DBL_10BYTE            (0x9 << 8)
#define TIMER_DCR_DBL_11BYTE            (0xA << 8)
#define TIMER_DCR_DBL_12BYTE            (0xB << 8)
#define TIMER_DCR_DBL_13BYTE            (0xC << 8)
#define TIMER_DCR_DBL_14BYTE            (0xD << 8)
#define TIMER_DCR_DBL_15BYTE            (0xE << 8)
#define TIMER_DCR_DBL_16BYTE            (0xF << 8)
#define TIMER_DCR_DBL_17BYTE            (0x10 << 8)
#define TIMER_DCR_DBL_18BYTE            (0x11 << 8)
#define TIMER_DCR_DBA                   0x1F
#define TIMER_DCR_DBA_CR1               0x0
#define TIMER_DCR_DBA_CR2               0x1
#define TIMER_DCR_DBA_SMCR              0x2
#define TIMER_DCR_DBA_DIER              0x3
#define TIMER_DCR_DBA_SR                0x4
#define TIMER_DCR_DBA_EGR               0x5
#define TIMER_DCR_DBA_CCMR1             0x6
#define TIMER_DCR_DBA_CCMR2             0x7
#define TIMER_DCR_DBA_CCER              0x8
#define TIMER_DCR_DBA_CNT               0x9
#define TIMER_DCR_DBA_PSC               0xA
#define TIMER_DCR_DBA_ARR               0xB
#define TIMER_DCR_DBA_RCR               0xC
#define TIMER_DCR_DBA_CCR1              0xD
#define TIMER_DCR_DBA_CCR2              0xE
#define TIMER_DCR_DBA_CCR3              0xF
#define TIMER_DCR_DBA_CCR4              0x10
#define TIMER_DCR_DBA_BDTR              0x11
#define TIMER_DCR_DBA_DCR               0x12
#define TIMER_DCR_DBA_DMAR              0x13

/**
 * Timer output compare modes.
 */
typedef enum timer_oc_mode {
    TIMER_OC_MODE_FROZEN = 0 << 4, /**< Frozen: comparison between output
                                      compare register and counter has no
                                      effect on the outputs. */
    TIMER_OC_MODE_ACTIVE_ON_MATCH = 1 << 4, /**< OCxREF signal is forced
                                               high when the count matches
                                               the channel capture/compare
                                               register. */
    TIMER_OC_MODE_INACTIVE_ON_MATCH = 2 << 4, /**< OCxREF signal is forced
                                                 low when the counter matches
                                                 the channel capture/compare
                                                 register. */
    TIMER_OC_MODE_TOGGLE = 3 << 4, /**< OCxREF toggles when counter
                                      matches the cannel capture/compare
                                      register. */
    TIMER_OC_MODE_FORCE_INACTIVE = 4 << 4, /**< OCxREF is forced low. */
    TIMER_OC_MODE_FORCE_ACTIVE = 5 << 4, /**< OCxREF is forced high. */
    TIMER_OC_MODE_PWM_1 = 6 << 4, /**< PWM mode 1.  In upcounting, channel is
                                     active as long as count is less than
                                     channel capture/compare register, else
                                     inactive.  In downcounting, channel is
                                     inactive as long as count exceeds
                                     capture/compare register, else
                                     active. */
    TIMER_OC_MODE_PWM_2 = 7 << 4  /**< PWM mode 2. In upcounting, channel is
                                     inactive as long as count is less than
                                     capture/compare register, else active.
                                     In downcounting, channel is active as
                                     long as count exceeds capture/compare
                                     register, else inactive. */
} timer_oc_mode;

/**
 * Timer output compare mode flags.
 * @see timer_oc_set_mode()
 */
typedef enum timer_oc_mode_flags {
    TIMER_OC_CE = BIT(7),       /**< Output compare clear enable. */
    TIMER_OC_PE = BIT(3),       /**< Output compare preload enable. */
    TIMER_OC_FE = BIT(2)        /**< Output compare fast enable. */
} timer_oc_mode_flags;

/**
 * @brief Timer type
 *
 * Type marker for timer_dev.
 *
 * @see timer_dev
 */
typedef enum timer_type {
    TIMER_ADVANCED,             /**< Advanced type */
    TIMER_GENERAL,              /**< General purpose type */
    TIMER_BASIC                 /**< Basic type */
} timer_type;

/** Timer device type */
typedef struct timer_dev_structure {
    TIM_TypeDef*  TIMx;
    uint32_t      clk;        // 时钟
	rcc_clockcmd  clkcmd;     // 时钟使能函数
    IRQn_Type     irq;        // 定时器中断函数
    timer_type    type;       // 定时器类型
    uint32_t      priority;
    voidFuncPtr   handlers[NR_ADV_HANDLERS]; // 用户定义的中断事件
} timer_dev;

/**
 * Used to configure the behavior of a timer channel.  Note that not
 * all timers can be configured in every mode.
 */
/* TODO TIMER_PWM_CENTER_ALIGNED, TIMER_INPUT_CAPTURE, TIMER_ONE_PULSE */
typedef enum timer_mode_structure {
    TIMER_DISABLED, /**< In this mode, the timer stops counting,
                         channel interrupts are detached, and no state
                         changes are output. */
    TIMER_PWM, /**< PWM output mode. This is the default mode for pins
                    after initialization. */
    /* TIMER_PWM_CENTER_ALIGNED, /\**< Center-aligned PWM output mode. *\/ */
    TIMER_OUTPUT_COMPARE, /**< In this mode, the timer counts from 0
                               to its reload value repeatedly; every
                               time the counter value reaches one of
                               the channel compare values, the
                               corresponding interrupt is fired. */
    /* TIMER_INPUT_CAPTURE, /\**< In this mode, the timer can measure the */
    /*                           pulse lengths of input signals. *\/ */
    /* TIMER_ONE_PULSE /\**< In this mode, the timer can generate a single */
    /*                      pulse on a GPIO pin for a specified amount of */
    /*                      time. *\/ */
} timer_mode;

/**
 * @brief Timer interrupt number.
 *
 * Not all timers support all of these values; see the descriptions
 * for each value.
 */
typedef enum timer_irq_ch_structure{
    TIMER_UPDATE = 0, /**< Update interrupt, available on all timers. */
    TIMER_CC1, /**< Capture/compare 1 interrupt, available
                              on general and advanced timers only. */
    TIMER_CC2, /**< Capture/compare 2 interrupt, general and
                              advanced timers only. */
    TIMER_CC3, /**< Capture/compare 3 interrupt, general and
                              advanced timers only. */
    TIMER_CC4, /**< Capture/compare 4 interrupt, general and
                              advanced timers only. */
    TIMER_COM, /**< COM interrupt, advanced timers only */
    TIMER_TRG, /**< Trigger interrupt, general and advanced
                              timers only */
    TIMER_BREAK /**< Break interrupt, advanced timers only. */
} timer_irq_ch_id;

/* --------------------内联函数定义-------------------- */

/**
 * @brief Returns the given timer's prescaler.
 * @param dev Timer whose prescaler to return
 * @see timer_generate_update()
 */
static inline uint16_t timer_get_prescaler(timer_dev *dev) 
{
    return (uint16_t)(dev->TIMx)->PSC;
}

/**
 * @brief Returns a timer's reload value.
 * @param dev Timer whose reload value to return
 */
static inline uint16_t timer_get_reload(timer_dev *dev) 
{
    return (uint16_t)(dev->TIMx)->ARR;
}

/**
 * @brief Get the compare value for the given timer channel.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose compare value to get.
 */
static inline uint32_t timer_get_compare(timer_dev *dev, uint8_t channel) 
{
    __IO uint32_t *ccr = &(dev->TIMx)->CCR1 + (channel - 1);
    return *ccr;
}

/**
 * @brief Set the compare value for the given timer channel.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose compare value to set.
 * @param value   New compare value.
 */
static inline void timer_set_compare(timer_dev *dev, uint8_t channel, uint16_t value) 
{
    switch(channel)
	{
		case 1:
			/* PWM1 Mode configuration: Channel1 */
			TIM_SetCompare1(dev->TIMx, value);
			//TIM_OC1Init(dev->TIMx, &TIM_OCInitStructure);
			break;
		case 2:
			/* PWM2 Mode configuration: Channel1 */
			TIM_SetCompare2(dev->TIMx, value);
			//TIM_OC2Init(dev->TIMx, &TIM_OCInitStructure);
			break;
		case 3:
			/* PWM3 Mode configuration: Channel1 */
			TIM_SetCompare3(dev->TIMx, value);
			//TIM_OC3Init(dev->TIMx, &TIM_OCInitStructure);
			break;
		case 4:
			/* PWM4 Mode configuration: Channel1 */
			TIM_SetCompare4(dev->TIMx, value);
			//TIM_OC4Init(dev->TIMx, &TIM_OCInitStructure);
			break;
		default:
			break;
	}
}
/**
 * @brief Generate an update event for the given timer.
 *
 * Normally, this will cause the prescaler and auto-reload values in
 * the PSC and ARR registers to take immediate effect.  However, this
 * function will do nothing if the UDIS bit is set in the timer's CR1
 * register (UDIS is cleared by default).
 *
 * @param dev Timer device to generate an update for.
 */
static inline void timer_generate_update(timer_dev *dev) 
{
    TIM_GenerateEvent(dev->TIMx,TIM_EventSource_Update);
}
/**
 * @brief Enable a timer interrupt.
 * @param dev Timer device.
 * @param interrupt Interrupt number to enable; this may be any
 *                  timer_irq_ch_id value appropriate for the timer.
 * @see timer_irq_ch_id
 * @see timer_channel
 */
static inline void timer_enable_irq(timer_dev *dev, uint8_t interrupt)
{
	switch(interrupt)
	{
		case TIMER_UPDATE:
			TIM_ITConfig(dev->TIMx, TIM_IT_Update, ENABLE);
			break;
		case TIMER_CC1: 
			TIM_ITConfig(dev->TIMx, TIM_IT_CC1, ENABLE);
			break;
		case TIMER_CC2:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC2, ENABLE);
			break;
		case TIMER_CC3:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC3, ENABLE);
			break;
		case TIMER_CC4:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC4, ENABLE);
			break;
		case TIMER_TRG:
			TIM_ITConfig(dev->TIMx, TIM_IT_Trigger, ENABLE);
			break;
		case TIMER_COM:
			TIM_ITConfig(dev->TIMx, TIM_IT_COM, ENABLE);
			break;
		case TIMER_BREAK:
			TIM_ITConfig(dev->TIMx, TIM_IT_Break, ENABLE);
			break;
	}
}

static inline void set_count_mode(timer_dev *dev, uint16_t TIM_CounterMode)
{
    TIM_CounterModeConfig(dev->TIMx, TIM_CounterMode);
}

static inline void set_clock_division(timer_dev *dev, uint16_t TIM_CKD)
{
    TIM_SetClockDivision(dev->TIMx, TIM_CKD);
}

/**
 * @brief Disable a timer interrupt.
 * @param dev Timer device.
 * @param interrupt Interrupt number to disable; this may be any
 *                  timer_irq_ch_id value appropriate for the timer.
 * @see timer_irq_ch_id
 * @see timer_channel
 */
static inline void timer_disable_irq(timer_dev *dev, uint8_t interrupt) 
{
	switch(interrupt)
	{
		case TIMER_UPDATE:
			TIM_ITConfig(dev->TIMx, TIM_IT_Update, DISABLE);
			break;
		case TIMER_CC1: 
			TIM_ITConfig(dev->TIMx, TIM_IT_CC1, DISABLE);
			break;
		case TIMER_CC2:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC2, DISABLE);
			break;
		case TIMER_CC3:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC3, DISABLE);
			break;
		case TIMER_CC4:
			TIM_ITConfig(dev->TIMx, TIM_IT_CC4, DISABLE);
			break;
		case TIMER_TRG:
			TIM_ITConfig(dev->TIMx, TIM_IT_Trigger, DISABLE);
			break;
		case TIMER_COM:
			TIM_ITConfig(dev->TIMx, TIM_IT_COM, DISABLE);
			break;
		case TIMER_BREAK:
			TIM_ITConfig(dev->TIMx, TIM_IT_Break, DISABLE);
			break;
	} 
}


/**
 * @brief Disable a timer channel's output compare or input capture signal.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to disable, from 1 to 4.
 * @see timer_cc_enable()
 */
static inline void timer_cc_disable(timer_dev *dev, uint8_t channel) 
{
    switch(channel)
    {
		case 1:
			dev->TIMx->CCER &= (uint16_t)~TIM_CCER_CC1E;
			break;
		case 2:
			dev->TIMx->CCER &= (uint16_t)~TIM_CCER_CC2E;
			break;
		case 3:
			dev->TIMx->CCER &= (uint16_t)~TIM_CCER_CC3E;
			break;
		case 4:
			dev->TIMx->CCER &= (uint16_t)~TIM_CCER_CC4E;
			break;
		default:
			break;
	}
}

/**
 * @brief Enable a timer channel's capture/compare signal.
 *
 * If the channel is configured as output, the corresponding output
 * compare signal will be output on the corresponding output pin.  If
 * the channel is configured as input, enables capture of the counter
 * value into the input capture/compare register.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to enable, from 1 to 4.
 */
static inline void timer_cc_enable(timer_dev *dev, uint8_t channel) 
{
    switch(channel)
    {
		case 1:
			dev->TIMx->CCER |= (uint16_t)TIM_CCER_CC1E;
			break;
		case 2:
			dev->TIMx->CCER |= (uint16_t)TIM_CCER_CC2E;
			break;
		case 3:
			dev->TIMx->CCER |= (uint16_t)TIM_CCER_CC3E;
			break;
		case 4:
			dev->TIMx->CCER |= (uint16_t)TIM_CCER_CC4E;
			break;
		default:
			break;
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
void timer_attach_interrupt(timer_dev *dev,uint8_t interrupt,voidFuncPtr handler);
					
/**
 * @brief Detach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to detach; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_detach_interrupt(timer_dev *dev, uint8_t interrupt);
void timer_set_mode(timer_dev *dev, uint8_t channel, timer_mode mode);

extern timer_dev* TIMER1;
extern timer_dev* TIMER2;
extern timer_dev* TIMER3;
extern timer_dev* TIMER4;
extern timer_dev* TIMER5;
extern timer_dev* TIMER6;
extern timer_dev* TIMER7;
extern timer_dev* TIMER8;



#endif
