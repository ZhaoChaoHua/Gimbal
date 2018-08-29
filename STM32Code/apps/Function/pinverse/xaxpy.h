/*
 * File: xaxpy.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

#ifndef XAXPY_H
#define XAXPY_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "pinverse_types.h"

/* Function Declarations */
extern void b_xaxpy(int n, float a, const float x[36], int ix0, float y[6], int
                    iy0);
extern void c_xaxpy(int n, float a, const float x[6], int ix0, float y[36], int
                    iy0);
extern void xaxpy(int n, float a, int ix0, float y[36], int iy0);

#endif

/*
 * File trailer for xaxpy.h
 *
 * [EOF]
 */
