#include "fir.h"

/* Private functions */
static
void filter_fir_f32_free(filter_fir_f32_t* instance) {
	/* Free state buffer */
	if (instance->f->pState != NULL && instance->Flags.F.MallocStates) {
		LIB_FREE_FUNC(instance->f->pState);
	}
	
	/* Free arm instance */
	if (instance->f != NULL) {
		LIB_FREE_FUNC(instance->f);
	}
	
	/* Free main instance */
	if (instance != NULL) {
		LIB_FREE_FUNC(instance);
	}
}

filter_fir_f32_t* filter_fir_f32_init(size_t coeff_size, const float32_t* coeffs, float32_t* StateBuffer, size_t block_size) {
	filter_fir_f32_t* instance;
	
	/* Allocate memory for TM_FILTER_FIR_F32_t */
	instance = (filter_fir_f32_t *) LIB_ALLOC_FUNC(sizeof(filter_fir_f32_t));
	/* Check value */
	if (instance == NULL) {
		return NULL;
	}
	
	/* Reset structure */
	memset(instance, 0, sizeof(filter_fir_f32_t));
	
	/* Save */
	instance->BlockSize = block_size;
	instance->StatesCount = coeff_size + block_size - 1;
	
	/* Allocate memory for arm_fir_instance_f32 */
	instance->f = (arm_fir_instance_f32 *) LIB_ALLOC_FUNC(sizeof(arm_fir_instance_f32));
	
	/* Check value */
	if (instance->f == NULL) {
		filter_fir_f32_free(instance);
		return NULL;
	}
	
	/* Reset structure */
	memset(instance->f, 0, sizeof(arm_fir_instance_f32));
	
	/* Allocate state buffer */
	if (StateBuffer == NULL) {
		/* Allocate memory for states */
		instance->f->pState = (float32_t *) LIB_ALLOC_FUNC(instance->StatesCount * sizeof(float32_t));
		
		/* Check value */
		if (instance->f->pState == NULL) {
			filter_fir_f32_free(instance);
			return NULL;
		}
		instance->Flags.F.MallocStates = 1;
	} else {
		instance->f->pState = StateBuffer;
	}
	
	/* Call ARM FIR initialization */
	arm_fir_init_f32(instance->f, coeff_size, (float32_t *)coeffs, instance->f->pState, block_size);
	
	/* Return pointer */
	return instance;
}



filter_fir_f32_t* filter_fir_f32_process(filter_fir_f32_t* instance, float32_t* In, float32_t* Out) {
	/* Check input values */
	if (instance == NULL || In == NULL || Out == NULL || instance->f == NULL) {
		return NULL;
	}
		
	/* Call ARM FIR function */
	arm_fir_f32(instance->f, In, Out, instance->BlockSize);
	
	/* Return instance value */
	return instance;
}



filter_fir_f32_t* filter_fir_f32_process_all(filter_fir_f32_t* instance, float32_t* In, float32_t* Out, size_t count) {
	uint32_t blocks;
	
	/* Check input values */
	if (instance == NULL || In == NULL || Out == NULL || instance->f == NULL) {
		return NULL;
	}
	
	/* Get number of blocks to process */
	blocks = count / instance->BlockSize;
	
	while (blocks-- > 0U) {
		/* Call ARM FIR function */
		arm_fir_f32(instance->f, In, Out, instance->BlockSize);
		
		/* Increase pointers */
		In += instance->BlockSize;
		Out += instance->BlockSize;
	}
	
	/* Get remaining bytes */
	blocks = count % instance->BlockSize;
	
	while (blocks-- > 0U) {
		/* Call ARM FIR function */
		arm_fir_f32(instance->f, In, Out, 1);
		
		/* Increase pointers */
		In++;
		Out++;
	}
	
	/* Return instance value */
	return instance;
}


filter_fir_f32_t* filter_fir_f32_clear(filter_fir_f32_t* instance) {
	/* No filter selected */
	if (instance == NULL) {
		return instance;
	}
	
	/* Set all to 0 */
	memset(instance->f->pState, 0, instance->StatesCount * sizeof(float32_t));
	
	/* Return instance */
	return instance;
}

void filter_fir_f32_deinit(filter_fir_f32_t* instance) {
	/* Free allocated memory for filter */
	filter_fir_f32_free(instance);
}