/*
 * File: _coder_pinverse_api.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_pinverse_api.h"
#include "_coder_pinverse_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131450U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "pinverse",                          /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real32_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId))[36];
static real32_T (*c_emlrt_marshallIn(const mxArray *src, const
  emlrtMsgIdentifier *msgId))[36];
static real32_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))
  [36];
static const mxArray *emlrt_marshallOut(const real32_T u[36]);

/* Function Definitions */

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[36]
 */
static real32_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId))[36]
{
  real32_T (*y)[36];
  y = c_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[36]
 */
  static real32_T (*c_emlrt_marshallIn(const mxArray *src, const
  emlrtMsgIdentifier *msgId))[36]
{
  real32_T (*ret)[36];
  static const int32_T dims[2] = { 6, 6 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "single", false, 2U,
    dims);
  ret = (real32_T (*)[36])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray *x
 *                const char_T *identifier
 * Return Type  : real32_T (*)[36]
 */
static real32_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))
  [36]
{
  real32_T (*y)[36];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}
/*
 * Arguments    : const real32_T u[36]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real32_T u[36])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 6, 6 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, iv1, 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void pinverse_api(const mxArray * const prhs[1], const mxArray *plhs[1])
{
  real32_T (*y)[36];
  real32_T (*x)[36];
  y = (real32_T (*)[36])mxMalloc(sizeof(real32_T [36]));

  /* Marshall function inputs */
  x = emlrt_marshallIn(emlrtAlias(prhs[0]), "x");

  /* Invoke the target function */
  pinverse(*x, *y);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*y);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pinverse_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  pinverse_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pinverse_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pinverse_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_pinverse_api.c
 *
 * [EOF]
 */
