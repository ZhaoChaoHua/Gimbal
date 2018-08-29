/*
 * File: xscal.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xscal.h"

/* Function Definitions */

/*
 * Arguments    : float a
 *                float x[36]
 *                int ix0
 * Return Type  : void
 */
void xscal(float a, float x[36], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 5; k++) {
    x[k - 1] *= a;
  }
}

/*
 * File trailer for xscal.c
 *
 * [EOF]
 */
