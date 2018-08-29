/*
 * File: pinverse.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "svd.h"

/* Function Definitions */

/*
 * Arguments    : const float x[36]
 *                float y[36]
 * Return Type  : void
 */
void pinverse(const float x[36], float y[36])
{
  boolean_T p;
  int ar;
  float U[36];
  float s[6];
  float V[36];
  int i0;
  float absxk;
  int vcol;
  int r;
  int br;
  int ic;
  int ib;
  int ia;
  p = false;
  for (ar = 0; ar < 36; ar++) {
    y[ar] = 0.0F;
    if (p || rtIsInfF(x[ar]) || rtIsNaNF(x[ar])) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    for (i0 = 0; i0 < 36; i0++) {
      y[i0] = ((real32_T)rtNaN);
    }
  } else {
    svd(x, U, s, V);
    absxk = (float)fabs(s[0]);
    if ((!rtIsInfF(absxk)) && (!rtIsNaNF(absxk))) {
      if (absxk <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        frexp(absxk, &vcol);
        absxk = (float)ldexp(1.0, vcol - 24);
      }
    } else {
      absxk = ((real32_T)rtNaN);
    }

    absxk *= 6.0F;
    r = 0;
    ar = 1;
    while ((ar < 7) && (s[ar - 1] > absxk)) {
      r++;
      ar++;
    }

    if (r > 0) {
      vcol = 0;
      for (br = 1; br <= r; br++) {
        absxk = 1.0F / s[br - 1];
        for (ar = vcol; ar + 1 <= vcol + 6; ar++) {
          V[ar] *= absxk;
        }

        vcol += 6;
      }

      for (vcol = 0; vcol <= 31; vcol += 6) {
        for (ic = vcol; ic + 1 <= vcol + 6; ic++) {
          y[ic] = 0.0F;
        }
      }

      br = -1;
      for (vcol = 0; vcol <= 31; vcol += 6) {
        ar = -1;
        br++;
        i0 = (br + 6 * (r - 1)) + 1;
        for (ib = br; ib + 1 <= i0; ib += 6) {
          if (U[ib] != 0.0F) {
            ia = ar;
            for (ic = vcol; ic + 1 <= vcol + 6; ic++) {
              ia++;
              y[ic] += U[ib] * V[ia];
            }
          }

          ar += 6;
        }
      }
    }
  }
}

/*
 * File trailer for pinverse.c
 *
 * [EOF]
 */
