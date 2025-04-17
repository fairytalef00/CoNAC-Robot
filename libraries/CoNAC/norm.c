/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 14-Apr-2025 21:10:39
 */

/* Include Files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "xrotg.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[10]
 * Return Type  : double
 */
double b_norm(const double x[10])
{
  double A[10];
  double e[2];
  double s[2];
  double absx;
  double absxk;
  double f;
  double scale;
  double sm;
  double snorm;
  double sqds;
  double t;
  double y;
  int k;
  int kase;
  int nmqp1_tmp;
  int q;
  int qp1;
  int qq;
  int qq_tmp;
  boolean_T apply_transform;
  boolean_T exitg1;
  y = 0.0;
  for (kase = 0; kase < 2; kase++) {
    for (qq = 0; qq < 5; qq++) {
      absx = fabs(x[qq + 5 * kase]);
      if (rtIsNaN(absx) || (absx > y)) {
        y = absx;
      }
    }
  }
  if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
    memcpy(&A[0], &x[0], 10U * sizeof(double));
    for (q = 0; q < 2; q++) {
      s[q] = 0.0;
      qp1 = q + 2;
      qq_tmp = q + 5 * q;
      qq = qq_tmp + 1;
      nmqp1_tmp = 4 - q;
      apply_transform = false;
      absx = 0.0;
      scale = 3.3121686421112381E-170;
      kase = (qq_tmp - q) + 5;
      for (k = qq; k <= kase; k++) {
        absxk = fabs(A[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          absx = absx * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          absx += t * t;
        }
      }
      absx = scale * sqrt(absx);
      if (absx > 0.0) {
        apply_transform = true;
        if (A[qq_tmp] < 0.0) {
          absx = -absx;
        }
        s[q] = absx;
        if (fabs(absx) >= 1.0020841800044864E-292) {
          absx = 1.0 / absx;
          for (k = qq; k <= kase; k++) {
            A[k - 1] *= absx;
          }
        } else {
          for (k = qq; k <= kase; k++) {
            A[k - 1] /= s[q];
          }
        }
        A[qq_tmp]++;
        s[q] = -s[q];
      } else {
        s[q] = 0.0;
      }
      for (kase = qp1; kase < 3; kase++) {
        if (apply_transform) {
          t = 0.0;
          for (k = 0; k <= nmqp1_tmp; k++) {
            t += A[qq_tmp + k] * A[(q + k) + 5];
          }
          absx = -(t / A[qq_tmp]);
          if (!(absx == 0.0)) {
            for (k = 0; k <= nmqp1_tmp; k++) {
              qq = (q + k) + 5;
              A[qq] += absx * A[qq_tmp + k];
            }
          }
        }
      }
    }
    q = 2;
    e[0] = A[5];
    e[1] = 0.0;
    if (s[0] != 0.0) {
      absxk = fabs(s[0]);
      absx = s[0] / absxk;
      s[0] = absxk;
      e[0] = A[5] / absx;
    }
    if (e[0] != 0.0) {
      absxk = fabs(e[0]);
      absx = absxk / e[0];
      e[0] = absxk;
      s[1] *= absx;
    }
    if (s[1] != 0.0) {
      s[1] = fabs(s[1]);
    }
    qp1 = 0;
    snorm = 0.0;
    if ((s[0] >= e[0]) || rtIsNaN(e[0])) {
      absx = s[0];
    } else {
      absx = e[0];
    }
    if ((!(absx <= 0.0)) && (!rtIsNaN(absx))) {
      snorm = absx;
    }
    if (s[1] >= 0.0) {
      absx = s[1];
    } else {
      absx = 0.0;
    }
    if ((!(snorm >= absx)) && (!rtIsNaN(absx))) {
      snorm = absx;
    }
    while ((q > 0) && (qp1 < 75)) {
      qq_tmp = q - 1;
      nmqp1_tmp = q - 1;
      exitg1 = false;
      while (!(exitg1 || (nmqp1_tmp == 0))) {
        absx = fabs(e[0]);
        if ((absx <= 2.2204460492503131E-16 * (fabs(s[0]) + fabs(s[1]))) ||
            (absx <= 1.0020841800044864E-292) ||
            ((qp1 > 20) && (absx <= 2.2204460492503131E-16 * snorm))) {
          e[0] = 0.0;
          exitg1 = true;
        } else {
          nmqp1_tmp = 0;
        }
      }
      if (nmqp1_tmp == q - 1) {
        kase = 4;
      } else {
        qq = q;
        kase = q;
        exitg1 = false;
        while ((!exitg1) && (kase >= nmqp1_tmp)) {
          qq = kase;
          if (kase == nmqp1_tmp) {
            exitg1 = true;
          } else {
            absx = 0.0;
            if (kase < q) {
              absx = fabs(e[0]);
            }
            if (kase > nmqp1_tmp + 1) {
              absx += fabs(e[0]);
            }
            absxk = fabs(s[kase - 1]);
            if ((absxk <= 2.2204460492503131E-16 * absx) ||
                (absxk <= 1.0020841800044864E-292)) {
              s[kase - 1] = 0.0;
              exitg1 = true;
            } else {
              kase--;
            }
          }
        }
        if (qq == nmqp1_tmp) {
          kase = 3;
        } else if (qq == q) {
          kase = 1;
        } else {
          kase = 2;
          nmqp1_tmp = qq;
        }
      }
      switch (kase) {
      case 1:
        f = e[0];
        e[0] = 0.0;
        for (k = qq_tmp; k >= nmqp1_tmp + 1; k--) {
          xrotg(&s[0], &f, &sm);
        }
        break;
      case 2:
        f = e[nmqp1_tmp - 1];
        e[nmqp1_tmp - 1] = 0.0;
        for (k = nmqp1_tmp + 1; k <= q; k++) {
          t = xrotg(&s[k - 1], &f, &sm);
          absx = e[k - 1];
          f = -sm * absx;
          e[k - 1] = absx * t;
        }
        break;
      case 3:
        absxk = s[q - 1];
        scale = fabs(absxk);
        absx = fabs(s[0]);
        if ((!(scale >= absx)) && (!rtIsNaN(absx))) {
          scale = absx;
        }
        absx = fabs(e[0]);
        if ((!(scale >= absx)) && (!rtIsNaN(absx))) {
          scale = absx;
        }
        absx = fabs(s[nmqp1_tmp]);
        if ((!(scale >= absx)) && (!rtIsNaN(absx))) {
          scale = absx;
        }
        absx = fabs(e[nmqp1_tmp]);
        if ((!(scale >= absx)) && (!rtIsNaN(absx))) {
          scale = absx;
        }
        sm = absxk / scale;
        absx = s[0] / scale;
        absxk = e[0] / scale;
        sqds = s[nmqp1_tmp] / scale;
        t = ((absx + sm) * (absx - sm) + absxk * absxk) / 2.0;
        absx = sm * absxk;
        absx *= absx;
        if ((t != 0.0) || (absx != 0.0)) {
          absxk = sqrt(t * t + absx);
          if (t < 0.0) {
            absxk = -absxk;
          }
          absxk = absx / (t + absxk);
        } else {
          absxk = 0.0;
        }
        f = (sqds + sm) * (sqds - sm) + absxk;
        absx = sqds * (e[nmqp1_tmp] / scale);
        for (k = nmqp1_tmp + 1; k < 2; k++) {
          t = xrotg(&f, &absx, &sm);
          f = t * s[0] + sm * e[0];
          absx = t * e[0] - sm * s[0];
          e[0] = absx;
          absxk = sm * s[1];
          s[1] *= t;
          s[0] = f;
          t = xrotg(&s[0], &absxk, &sm);
          f = t * absx + sm * s[1];
          s[1] = -sm * absx + t * s[1];
          absx = sm * e[1];
          e[1] *= t;
        }
        e[0] = f;
        qp1++;
        break;
      default:
        if (s[nmqp1_tmp] < 0.0) {
          s[nmqp1_tmp] = -s[nmqp1_tmp];
        }
        while ((nmqp1_tmp + 1 < 2) && (s[0] < s[1])) {
          absxk = s[0];
          s[0] = s[1];
          s[1] = absxk;
          nmqp1_tmp = 1;
        }
        qp1 = 0;
        q--;
        break;
      }
    }
    y = s[0];
  }
  return y;
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
