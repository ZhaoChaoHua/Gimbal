#ifndef __FIR_H__
#define __FIR_H__

#include "arm_math.h"
#include "stdlib.h"


/* Library allocations */
#ifndef LIB_ALLOC_FUNC
#define LIB_ALLOC_FUNC       malloc
#endif
#ifndef LIB_FREE_FUNC
#define LIB_FREE_FUNC        free
#endif

/**
 * @brief  FIR Filter structure
 */
typedef struct _filter_fir_f32_t
{
	arm_fir_instance_f32* f;
	size_t BlockSize;
	size_t StatesCount;
	union{
		struct{
			uint8_t MallocStates:1;
		} F;
		uint8_t FlagsValues;
	} Flags;
} filter_fir_f32_t;


/**
 * @brief  Creates and initializes ARM FIR filter using F32 coefficients
 * @note   Malloc is used to initialize proper size of heap memory
 * @param  coeff_size: Number of cofficients for FIR filtering.
 *           When possible, use coeff_size multiply of 8 elements, because filtering process is designed for this coeff size to be fastest
 * @param  *coeffs: Pointer to coefficients FIR filter array coeff_size length
 * @param  *StateBuffer: Pointer to state buffer. Set to null to let malloc allocate memory in heap
 * @param  block_size: size of block for FIR filtering. Number of elements for filtering at a time.
 *           When possible, use block_size multiply of 8 samples, because filtering process is designed for this block size to be fastest
 * @retval Pointer to @ref TM_FILTER_FIR_F32_t instance or NULL if allocation failed
 */
filter_fir_f32_t* filter_fir_f32_init(size_t coeff_size, const float32_t* coeffs, float32_t* StateBuffer, size_t block_size);

/**
 * @brief  Process data through ARM FIR F32 filter
 * @param  *instance: Pointer to @ref TM_FILTER_FIR_F32_t instance
 * @param  *In: Input data array to process in filter. Length must be the same as block_size parameter in initialization
 * @param  *Out: Output data array to store filtered values. Length must be the same as block_size parameter in initialization
 * @retval Pointer to @ref TM_FILTER_FIR_F32_t instance
 */
filter_fir_f32_t* filter_fir_f32_process(filter_fir_f32_t* instance, float32_t* In, float32_t* Out);

/**
 * @brief  Process all input data through ARM FIR F32 filter
 * @param  *instance: Pointer to @ref TM_FILTER_FIR_F32_t instance
 * @param  *In: Input data array to process in filter
 * @param  *Out: Output data array to store filtered values
 * @param  count: Number of elements in input array and output array. Length must be multiply of block size set on initialization
 * @retval Pointer to @ref TM_FILTER_FIR_F32_t instance
 */
filter_fir_f32_t* filter_fir_f32_process_all(filter_fir_f32_t* instance, float32_t* In, float32_t* Out, size_t count);


/**
 * @brief  Clear filter's state buffer, set values to 0
 * @param  instance: Pointer to TM_FILTER_FIR_F32_t instance
 * @retval Pointer to @ref TM_FILTER_FIR_F32_t structure
 */
filter_fir_f32_t* filter_fir_f32_clear(filter_fir_f32_t* instance);

/**
 * @brief  Deallocates ARM FIR filter with all allocated buffers
 * @param  Pointer to @ref TM_FILTER_FIR_F32_t which will be used for deinit
 * @retval None 
 */
void filter_fir_f32_deinit(filter_fir_f32_t* instance);

#endif



