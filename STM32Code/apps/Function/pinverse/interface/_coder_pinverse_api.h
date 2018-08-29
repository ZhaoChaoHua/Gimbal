/*
 * File: _coder_pinverse_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

#ifndef _CODER_PINVERSE_API_H
#define _CODER_PINVERSE_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_pinverse_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void pinverse(real32_T x[36], real32_T y[36]);
extern void pinverse_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
extern void pinverse_atexit(void);
extern void pinverse_initialize(void);
extern void pinverse_terminate(void);
extern void pinverse_xil_terminate(void);

#endif

/*
 * File trailer for _coder_pinverse_api.h
 *
 * [EOF]
 */
