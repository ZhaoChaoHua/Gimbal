/*
 * File: xaxpy.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xaxpy.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                float a
 *                const float x[36]
 *                int ix0
 *                float y[6]
 *                int iy0
 * Return Type  : void
 */
void b_xaxpy(int n, float a, const float x[36], int ix0, float y[6], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                float a
 *                const float x[6]
 *                int ix0
 *                float y[36]
 *                int iy0
 * Return Type  : void
 */
void c_xaxpy(int n, float a, const float x[6], int ix0, float y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                float a
 *                int ix0
 *                float y[36]
 *                int iy0
 * Return Type  : void
 */
void xaxpy(int n, float a, int ix0, float y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * File trailer for xaxpy.c
 *
 * [EOF]
 */
