#ifndef _STM32F40X_DEFINE_H__
#define _STM32F40X_DEFINE_H__

#include <stm32f4xx.h>
#include <rtthread.h>

typedef void (* thread_entry)(void*);
typedef void (* voidFuncPtr)(void);
typedef void (* rcc_clockcmd)(uint32_t, FunctionalState);
typedef void (* bgc_main_t)(void* param);

struct eigen_matrix_instance {
	int numRows;
	int numCols;
	float *pData;
};

/*
 * Bit manipulation
 */

/** 1 << the bit number */
#define BIT(shift)                     (1UL << (shift))
/** Mask shifted left by 'shift' */
#define BIT_MASK_SHIFT(mask, shift)    ((mask) << (shift))
/** Bits m to n of x */
#define GET_BITS(x, m, n)              ((((uint32)x) << (31 - (n))) >> ((31 - (n)) + (m)))
/** True if v is a power of two (1, 2, 4, 8, ...) */
#define IS_POWER_OF_TWO(v)             ((v) && !((v) & ((v) - 1)))


#define min_def(a,b)                       ((a)<(b)?(a):(b))
#define max_def(a,b)                       ((a)>(b)?(a):(b))
#define constrain_def(amt,low,high)        ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round_def(x)                       ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define floor_def(x)                       ((x)>=0?(long)((x)-0.5):(long)((x)+0.5))
#define radians_def(deg)                   ((deg)*DEG_TO_RAD)
#define degrees_def(rad)                   ((rad)*RAD_TO_DEG)
#define sq_def(x)                          ((x)*(x))


#define TIMER_PER_MICROSECOND         (SystemCoreClock/1000000)
#define TIMER_RELOAD_VAL             (CYCLES_PER_MICROSECOND*1000-1)

#define Bit_Band(Addr, BitNum)  ((Addr & 0xF0000000)+0x2000000+((Addr &0xFFFFF)<<5)+(BitNum<<2))
#define Mem_Addr(Addr)          *((vu16 *)(Addr))
#define Bit_Addr(Addr, BitNum)  Mem_Addr(Bit_Band(Addr, BitNum))

#define GPIOA_ODR_Addr  (GPIOA_BASE+0x14)
#define GPIOB_ODR_Addr  (GPIOB_BASE+0x14)
#define GPIOC_ODR_Addr  (GPIOC_BASE+0x14)
#define GPIOD_ODR_Addr  (GPIOD_BASE+0x14)
#define GPIOE_ODR_Addr  (GPIOE_BASE+0x14)
#define GPIOF_ODR_Addr  (GPIOF_BASE+0x14)
#define GPIOG_ODR_Addr  (GPIOG_BASE+0x14)
#define GPIOA_IDR_Addr  (GPIOA_BASE+0x10)
#define GPIOB_IDR_Addr  (GPIOB_BASE+0x10)
#define GPIOC_IDR_Addr  (GPIOC_BASE+0x10)
#define GPIOD_IDR_Addr  (GPIOD_BASE+0x10)
#define GPIOE_IDR_Addr  (GPIOE_BASE+0x10)
#define GPIOF_IDR_Addr  (GPIOF_BASE+0x10)
#define GPIOG_IDR_Addr  (GPIOG_BASE+0x10)

#define PAO(Pin)  Bit_Addr(GPIOA_ODR_Addr, Pin)
#define PAI(Pin)  Bit_Addr(GPIOA_IDR_Addr, Pin)
#define PBO(Pin)  Bit_Addr(GPIOB_ODR_Addr, Pin)
#define PBI(Pin)  Bit_Addr(GPIOB_IDR_Addr, Pin)
#define PCO(Pin)  Bit_Addr(GPIOC_ODR_Addr, Pin)
#define PCI(Pin)  Bit_Addr(GPIOC_IDR_Addr, Pin)
#define PDO(Pin)  Bit_Addr(GPIOD_ODR_Addr, Pin)
#define PDI(Pin)  Bit_Addr(GPIOD_IDR_Addr, Pin)
#define PEO(Pin)  Bit_Addr(GPIOE_ODR_Addr, Pin)
#define PEI(Pin)  Bit_Addr(GPIOE_IDR_Addr, Pin)
#define PFO(Pin)  Bit_Addr(GPIOF_ODR_Addr, Pin)
#define PFI(Pin)  Bit_Addr(GPIOF_IDR_Addr, Pin)
#define PGO(Pin)  Bit_Addr(GPIOG_ODR_Addr, Pin)
#define PGI(Pin)  Bit_Addr(GPIOG_IDR_Addr, Pin)
/*====================================================================================================*/
/*====================================================================================================*/
#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)

#define Byte32(Byte4, Byte3, Byte2, Byte1)  ((u32)((((u8)(Byte4))<<24) | (((u8)(Byte3))<<16) | (((u8)(Byte2))<<8) | ((u8)(Byte1))))
#define Byte16(ByteH, ByteL)  ((u16)((((u16)(ByteH))<<8) | ((u16)(ByteL))))
#define Byte8H(ByteH)         ((u8)(((u16)(ByteH))>>8))
#define Byte8L(ByteL)         ((u8)(ByteL))

#define US_T (SysTick->LOAD/(1000000.0f/RT_TICK_PER_SECOND))
#define LOAD_T SysTick->LOAD

#endif
