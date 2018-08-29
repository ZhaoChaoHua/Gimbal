/*
 * File: xdotc.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xdotc.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const float x[36]
 *                int ix0
 *                const float y[36]
 *                int iy0
 * Return Type  : float
 */
float xdotc(int n, const float x[36], int ix0, const float y[36], int iy0)
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/*
 * File trailer for xdotc.c
 *
 * [EOF]
 */
