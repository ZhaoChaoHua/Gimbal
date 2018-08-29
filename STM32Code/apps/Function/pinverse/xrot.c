/*
 * File: xrot.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xrot.h"

/* Function Definitions */

/*
 * Arguments    : float x[36]
 *                int ix0
 *                int iy0
 *                float c
 *                float s
 * Return Type  : void
 */
void xrot(float x[36], int ix0, int iy0, float c, float s)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

/*
 * File trailer for xrot.c
 *
 * [EOF]
 */
