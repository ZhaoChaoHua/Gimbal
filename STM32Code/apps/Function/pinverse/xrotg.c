/*
 * File: xrotg.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "xrotg.h"

/* Function Definitions */

/*
 * Arguments    : float *a
 *                float *b
 *                float *c
 *                float *s
 * Return Type  : void
 */
void xrotg(float *a, float *b, float *c, float *s)
{
  float roe;
  float absa;
  float absb;
  float scale;
  float ads;
  float bds;
  roe = *b;
  absa = (float)fabs(*a);
  absb = (float)fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    scale = 0.0F;
    *b = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= (float)sqrt(ads * ads + bds * bds);
    if (roe < 0.0F) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0F) {
      *b = 1.0F / *c;
    } else {
      *b = 1.0F;
    }
  }

  *a = scale;
}

/*
 * File trailer for xrotg.c
 *
 * [EOF]
 */
