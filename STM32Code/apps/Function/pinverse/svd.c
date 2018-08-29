/*
 * File: svd.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 20-Nov-2017 17:59:47
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "pinverse.h"
#include "svd.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xscal.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"

/* Function Definitions */

/*
 * Arguments    : const float A[36]
 *                float U[36]
 *                float s[6]
 *                float V[36]
 * Return Type  : void
 */
void svd(const float A[36], float U[36], float s[6], float V[36])
{
  float b_A[36];
  int i;
  float b_s[6];
  float Vf[36];
  float e[6];
  int q;
  float work[6];
  int m;
  int iter;
  boolean_T apply_transform;
  float ztest0;
  int qp1jj;
  int qs;
  float snorm;
  float rt;
  float ztest;
  int exitg1;
  boolean_T exitg2;
  float f;
  float varargin_1[5];
  float mtmp;
  float sqds;
  memcpy(&b_A[0], &A[0], 36U * sizeof(float));
  for (i = 0; i < 6; i++) {
    b_s[i] = 0.0F;
    e[i] = 0.0F;
    work[i] = 0.0F;
  }

  memset(&U[0], 0, 36U * sizeof(float));
  memset(&Vf[0], 0, 36U * sizeof(float));
  for (q = 0; q < 5; q++) {
    iter = q + 6 * q;
    apply_transform = false;
    ztest0 = xnrm2(6 - q, b_A, iter + 1);
    if (ztest0 > 0.0F) {
      apply_transform = true;
      if (b_A[iter] < 0.0F) {
        b_s[q] = -ztest0;
      } else {
        b_s[q] = ztest0;
      }

      if ((float)fabs(b_s[q]) >= 9.86076132E-32F) {
        ztest0 = 1.0F / b_s[q];
        i = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= i; qp1jj++) {
          b_A[qp1jj] *= ztest0;
        }
      } else {
        i = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= i; qp1jj++) {
          b_A[qp1jj] /= b_s[q];
        }
      }

      b_A[iter]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0F;
    }

    for (qs = q + 1; qs + 1 < 7; qs++) {
      i = q + 6 * qs;
      if (apply_transform) {
        xaxpy(6 - q, -(xdotc(6 - q, b_A, iter + 1, b_A, i + 1) / b_A[q + 6 * q]),
              iter + 1, b_A, i + 1);
      }

      e[qs] = b_A[i];
    }

    for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
      U[qp1jj + 6 * q] = b_A[qp1jj + 6 * q];
    }

    if (q + 1 <= 4) {
      ztest0 = b_xnrm2(5 - q, e, q + 2);
      if (ztest0 == 0.0F) {
        e[q] = 0.0F;
      } else {
        if (e[q + 1] < 0.0F) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = e[q];
        if ((float)fabs(e[q]) >= 9.86076132E-32F) {
          ztest0 = 1.0F / e[q];
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] *= ztest0;
          }
        } else {
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
          work[qp1jj] = 0.0F;
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          b_xaxpy(5 - q, e[qs], b_A, (q + 6 * qs) + 2, work, q + 2);
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          c_xaxpy(5 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 6 * qs) + 2);
        }
      }

      for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
        Vf[qp1jj + 6 * q] = e[qp1jj];
      }
    }
  }

  m = 4;
  b_s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0F;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    U[30 + qp1jj] = 0.0F;
  }

  U[35] = 1.0F;
  for (q = 4; q >= 0; q += -1) {
    iter = q + 6 * q;
    if (b_s[q] != 0.0F) {
      for (qs = q + 1; qs + 1 < 7; qs++) {
        i = (q + 6 * qs) + 1;
        xaxpy(6 - q, -(xdotc(6 - q, U, iter + 1, U, i) / U[iter]), iter + 1, U,
              i);
      }

      for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
        U[qp1jj + 6 * q] = -U[qp1jj + 6 * q];
      }

      U[iter]++;
      for (qp1jj = 1; qp1jj <= q; qp1jj++) {
        U[(qp1jj + 6 * q) - 1] = 0.0F;
      }
    } else {
      for (qp1jj = 0; qp1jj < 6; qp1jj++) {
        U[qp1jj + 6 * q] = 0.0F;
      }

      U[iter] = 1.0F;
    }
  }

  for (q = 5; q >= 0; q += -1) {
    if ((q + 1 <= 4) && (e[q] != 0.0F)) {
      i = (q + 6 * q) + 2;
      for (qs = q + 1; qs + 1 < 7; qs++) {
        qp1jj = (q + 6 * qs) + 2;
        xaxpy(5 - q, -(xdotc(5 - q, Vf, i, Vf, qp1jj) / Vf[i - 1]), i, Vf, qp1jj);
      }
    }

    for (qp1jj = 0; qp1jj < 6; qp1jj++) {
      Vf[qp1jj + 6 * q] = 0.0F;
    }

    Vf[q + 6 * q] = 1.0F;
  }

  for (q = 0; q < 6; q++) {
    ztest0 = e[q];
    if (b_s[q] != 0.0F) {
      rt = (float)fabs(b_s[q]);
      ztest = b_s[q] / rt;
      b_s[q] = rt;
      if (q + 1 < 6) {
        ztest0 = e[q] / ztest;
      }

      xscal(ztest, U, 1 + 6 * q);
    }

    if ((q + 1 < 6) && (ztest0 != 0.0F)) {
      rt = (float)fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      b_s[q + 1] *= ztest;
      xscal(ztest, Vf, 1 + 6 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0F;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    ztest0 = (float)fabs(b_s[qp1jj]);
    ztest = (float)fabs(e[qp1jj]);
    if ((ztest0 > ztest) || rtIsNaNF(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if (!((snorm > ztest0) || rtIsNaNF(ztest0))) {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    qp1jj = m;
    do {
      exitg1 = 0;
      q = qp1jj + 1;
      if (qp1jj + 1 == 0) {
        exitg1 = 1;
      } else {
        ztest0 = (float)fabs(e[qp1jj]);
        if ((ztest0 <= 1.1920929E-7F * ((float)fabs(b_s[qp1jj]) + (float)fabs
              (b_s[qp1jj + 1]))) || (ztest0 <= 9.86076132E-32F) || ((iter > 20) &&
             (ztest0 <= 1.1920929E-7F * snorm))) {
          e[qp1jj] = 0.0F;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (qp1jj + 1 == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg2 = false;
      while ((!exitg2) && (i >= qp1jj + 1)) {
        qs = i;
        if (i == qp1jj + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (i < m + 2) {
            ztest0 = (float)fabs(e[i - 1]);
          }

          if (i > qp1jj + 2) {
            ztest0 += (float)fabs(e[i - 2]);
          }

          ztest = (float)fabs(b_s[i - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            b_s[i - 1] = 0.0F;
            exitg2 = true;
          } else {
            i--;
          }
        }
      }

      if (qs == qp1jj + 1) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }

    switch (i) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (qp1jj = m; qp1jj + 1 >= q + 1; qp1jj--) {
        xrotg(&b_s[qp1jj], &f, &ztest0, &ztest);
        if (qp1jj + 1 > q + 1) {
          f = -ztest * e[qp1jj - 1];
          e[qp1jj - 1] *= ztest0;
        }

        xrot(Vf, 1 + 6 * qp1jj, 1 + 6 * (m + 1), ztest0, ztest);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (qp1jj = q; qp1jj + 1 <= m + 2; qp1jj++) {
        xrotg(&b_s[qp1jj], &f, &ztest0, &ztest);
        f = -ztest * e[qp1jj];
        e[qp1jj] *= ztest0;
        xrot(U, 1 + 6 * qp1jj, 1 + 6 * (q - 1), ztest0, ztest);
      }
      break;

     case 3:
      varargin_1[0] = (float)fabs(b_s[m + 1]);
      varargin_1[1] = (float)fabs(b_s[m]);
      varargin_1[2] = (float)fabs(e[m]);
      varargin_1[3] = (float)fabs(b_s[q]);
      varargin_1[4] = (float)fabs(e[q]);
      i = 1;
      mtmp = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        qp1jj = 2;
        exitg2 = false;
        while ((!exitg2) && (qp1jj < 6)) {
          i = qp1jj;
          if (!rtIsNaNF(varargin_1[qp1jj - 1])) {
            mtmp = varargin_1[qp1jj - 1];
            exitg2 = true;
          } else {
            qp1jj++;
          }
        }
      }

      if (i < 5) {
        while (i + 1 < 6) {
          if (varargin_1[i] > mtmp) {
            mtmp = varargin_1[i];
          }

          i++;
        }
      }

      f = b_s[m + 1] / mtmp;
      ztest0 = b_s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = b_s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0F;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0F) || (ztest0 != 0.0F)) {
        ztest = (float)sqrt(rt * rt + ztest0);
        if (rt < 0.0F) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0F;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      rt = sqds * (e[q] / mtmp);
      for (qp1jj = q + 1; qp1jj <= m + 1; qp1jj++) {
        xrotg(&f, &rt, &ztest0, &ztest);
        if (qp1jj > q + 1) {
          e[qp1jj - 2] = f;
        }

        f = ztest0 * b_s[qp1jj - 1] + ztest * e[qp1jj - 1];
        e[qp1jj - 1] = ztest0 * e[qp1jj - 1] - ztest * b_s[qp1jj - 1];
        rt = ztest * b_s[qp1jj];
        b_s[qp1jj] *= ztest0;
        xrot(Vf, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest0, ztest);
        b_s[qp1jj - 1] = f;
        xrotg(&b_s[qp1jj - 1], &rt, &ztest0, &ztest);
        f = ztest0 * e[qp1jj - 1] + ztest * b_s[qp1jj];
        b_s[qp1jj] = -ztest * e[qp1jj - 1] + ztest0 * b_s[qp1jj];
        rt = ztest * e[qp1jj];
        e[qp1jj] *= ztest0;
        xrot(U, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest0, ztest);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (b_s[q] < 0.0F) {
        b_s[q] = -b_s[q];
        xscal(-1.0F, Vf, 1 + 6 * q);
      }

      i = q + 1;
      while ((q + 1 < 6) && (b_s[q] < b_s[i])) {
        rt = b_s[q];
        b_s[q] = b_s[i];
        b_s[i] = rt;
        xswap(Vf, 1 + 6 * q, 1 + 6 * (q + 1));
        xswap(U, 1 + 6 * q, 1 + 6 * (q + 1));
        q = i;
        i++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    s[qp1jj] = b_s[qp1jj];
    for (i = 0; i < 6; i++) {
      V[i + 6 * qp1jj] = Vf[i + 6 * qp1jj];
    }
  }
}

/*
 * File trailer for svd.c
 *
 * [EOF]
 */
