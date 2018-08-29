/*
 * File: xswap.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xswap.h"

/* Function Definitions */

/*
 * Arguments    : float x[36]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
void xswap(float x[36], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/*
 * File trailer for xswap.c
 *
 * [EOF]
 */
