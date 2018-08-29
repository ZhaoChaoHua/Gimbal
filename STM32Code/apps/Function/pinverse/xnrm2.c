/*
 * File: xnrm2.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xnrm2.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const float x[6]
 *                int ix0
 * Return Type  : float
 */
float b_xnrm2(int n, const float x[6], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = (float)fabs(x[ix0 - 1]);
    } else {
      scale = 1.17549435E-38F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = (float)fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0F + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * (float)sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : int n
 *                const float x[36]
 *                int ix0
 * Return Type  : float
 */
float xnrm2(int n, const float x[36], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = (float)fabs(x[ix0 - 1]);
    } else {
      scale = 1.17549435E-38F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = (float)fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0F + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * (float)sqrt(y);
    }
  }

  return y;
}

/*
 * File trailer for xnrm2.c
 *
 * [EOF]
 */
