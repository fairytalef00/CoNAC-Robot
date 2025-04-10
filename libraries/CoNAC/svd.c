/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: svd.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 10-Apr-2025 18:08:03
 */

/* Include Files */
#include "svd.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A[20]
 *                double U[4]
 * Return Type  : void
 */
void b_svd(const double A[20], double U[4])
{
  double b_A[20];
  double work[5];
  double e[4];
  double s[4];
  double nrm;
  double r;
  double rt;
  double scale;
  double sm;
  double snorm;
  double sqds;
  int i;
  int iter;
  int jj;
  int k;
  int m;
  int nmqp1_tmp;
  int q;
  int qp1;
  int qs;
  boolean_T apply_transform;
  boolean_T exitg1;
  memcpy(&b_A[0], &A[0], 20U * sizeof(double));
  s[0] = 0.0;
  e[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  s[3] = 0.0;
  e[3] = 0.0;
  for (i = 0; i < 5; i++) {
    work[i] = 0.0;
  }
  for (q = 0; q < 4; q++) {
    qp1 = q + 2;
    iter = q + 5 * q;
    i = iter + 1;
    nmqp1_tmp = 4 - q;
    apply_transform = false;
    nrm = c_xnrm2(5 - q, b_A, iter + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        m = (iter - q) + 5;
        for (k = i; k <= m; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        m = (iter - q) + 5;
        for (k = i; k <= m; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 5; jj++) {
      i = q + 5 * (jj - 1);
      if (apply_transform) {
        nrm = 0.0;
        for (k = 0; k <= nmqp1_tmp; k++) {
          nrm += b_A[iter + k] * b_A[i + k];
        }
        nrm = -(nrm / b_A[iter]);
        if (!(nrm == 0.0)) {
          for (k = 0; k <= nmqp1_tmp; k++) {
            qs = i + k;
            b_A[qs] += nrm * b_A[iter + k];
          }
        }
      }
      e[jj - 1] = b_A[i];
    }
    if (q + 1 <= 2) {
      nrm = b_xnrm2(3 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        nrm = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (k = qp1; k < 5; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (k = qp1; k < 5; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (jj = qp1; jj < 6; jj++) {
          work[jj - 1] = 0.0;
        }
        for (jj = qp1; jj < 5; jj++) {
          nrm = e[jj - 1];
          if (!(nrm == 0.0)) {
            i = q + 5 * (jj - 1);
            m = 3 - q;
            for (k = 0; k <= m; k++) {
              qs = (q + k) + 1;
              work[qs] += nrm * b_A[(i + k) + 1];
            }
          }
        }
        for (jj = qp1; jj < 5; jj++) {
          b_xaxpy(4 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                  (q + 5 * (jj - 1)) + 2);
        }
      }
    }
  }
  m = 2;
  e[2] = b_A[17];
  e[3] = 0.0;
  iter = 0;
  snorm = 0.0;
  nrm = s[0];
  if (s[0] != 0.0) {
    rt = fabs(s[0]);
    r = s[0] / rt;
    nrm = rt;
    s[0] = rt;
    e[0] /= r;
  }
  if (e[0] != 0.0) {
    rt = fabs(e[0]);
    r = rt / e[0];
    e[0] = rt;
    s[1] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[0])) && (!rtIsNaN(e[0]))) {
    nrm = e[0];
  }
  if ((!(nrm <= 0.0)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[1];
  if (s[1] != 0.0) {
    rt = fabs(s[1]);
    r = s[1] / rt;
    nrm = rt;
    s[1] = rt;
    e[1] /= r;
  }
  if (e[1] != 0.0) {
    rt = fabs(e[1]);
    r = rt / e[1];
    e[1] = rt;
    s[2] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[1])) && (!rtIsNaN(e[1]))) {
    nrm = e[1];
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[2];
  if (s[2] != 0.0) {
    rt = fabs(s[2]);
    r = s[2] / rt;
    nrm = rt;
    s[2] = rt;
    e[2] = b_A[17] / r;
  }
  if (e[2] != 0.0) {
    rt = fabs(e[2]);
    r = rt / e[2];
    e[2] = rt;
    s[3] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[2])) && (!rtIsNaN(e[2]))) {
    nrm = e[2];
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[3];
  if (s[3] != 0.0) {
    rt = fabs(s[3]);
    nrm = rt;
    s[3] = rt;
  }
  nrm = fabs(nrm);
  if (!(nrm >= 0.0)) {
    nrm = 0.0;
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  while ((m + 2 > 0) && (iter < 75)) {
    nmqp1_tmp = m + 1;
    jj = m + 1;
    exitg1 = false;
    while (!(exitg1 || (jj == 0))) {
      nrm = fabs(e[jj - 1]);
      if ((nrm <= 2.2204460492503131E-16 * (fabs(s[jj - 1]) + fabs(s[jj]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[jj - 1] = 0.0;
        exitg1 = true;
      } else {
        jj--;
      }
    }
    if (jj == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg1 = false;
      while ((!exitg1) && (i >= jj)) {
        qs = i;
        if (i == jj) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (i < m + 2) {
            nrm = fabs(e[i - 1]);
          }
          if (i > jj + 1) {
            nrm += fabs(e[i - 2]);
          }
          rt = fabs(s[i - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[i - 1] = 0.0;
            exitg1 = true;
          } else {
            i--;
          }
        }
      }
      if (qs == jj) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        jj = qs;
      }
    }
    switch (i) {
    case 1:
      rt = e[m];
      e[m] = 0.0;
      for (k = nmqp1_tmp; k >= jj + 1; k--) {
        sm = xrotg(&s[k - 1], &rt, &sqds);
        if (k > jj + 1) {
          r = e[k - 2];
          rt = -sqds * r;
          e[k - 2] = r * sm;
        }
      }
      break;
    case 2:
      rt = e[jj - 1];
      e[jj - 1] = 0.0;
      for (k = jj + 1; k <= m + 2; k++) {
        sm = xrotg(&s[k - 1], &rt, &sqds);
        r = e[k - 1];
        rt = -sqds * r;
        e[k - 1] = r * sm;
      }
      break;
    case 3:
      rt = s[m + 1];
      scale = fabs(rt);
      nrm = fabs(s[m]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(e[m]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(s[jj]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(e[jj]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      sm = rt / scale;
      nrm = s[m] / scale;
      rt = e[m] / scale;
      sqds = s[jj] / scale;
      r = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((r != 0.0) || (nrm != 0.0)) {
        rt = sqrt(r * r + nrm);
        if (r < 0.0) {
          rt = -rt;
        }
        rt = nrm / (r + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[jj] / scale);
      for (k = jj + 1; k <= nmqp1_tmp; k++) {
        sm = xrotg(&rt, &nrm, &sqds);
        if (k > jj + 1) {
          e[k - 2] = rt;
        }
        nrm = e[k - 1];
        r = s[k - 1];
        e[k - 1] = sm * nrm - sqds * r;
        rt = sqds * s[k];
        s[k] *= sm;
        s[k - 1] = sm * r + sqds * nrm;
        sm = xrotg(&s[k - 1], &rt, &sqds);
        r = e[k - 1];
        rt = sm * r + sqds * s[k];
        s[k] = -sqds * r + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
      }
      e[m] = rt;
      iter++;
      break;
    default:
      if (s[jj] < 0.0) {
        s[jj] = -s[jj];
      }
      qp1 = jj + 1;
      while ((jj + 1 < 4) && (s[jj] < s[qp1])) {
        rt = s[jj];
        s[jj] = s[qp1];
        s[qp1] = rt;
        jj = qp1;
        qp1++;
      }
      iter = 0;
      m--;
      break;
    }
  }
  U[0] = s[0];
  U[1] = s[1];
  U[2] = s[2];
  U[3] = s[3];
}

/*
 * Arguments    : const double A[28]
 *                double U[4]
 * Return Type  : void
 */
void svd(const double A[28], double U[4])
{
  double b_A[28];
  double work[7];
  double e[4];
  double s[4];
  double nrm;
  double r;
  double rt;
  double scale;
  double sm;
  double snorm;
  double sqds;
  int i;
  int iter;
  int jj;
  int k;
  int m;
  int nmqp1_tmp;
  int q;
  int qp1;
  int qs;
  boolean_T apply_transform;
  boolean_T exitg1;
  memcpy(&b_A[0], &A[0], 28U * sizeof(double));
  s[0] = 0.0;
  e[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  s[3] = 0.0;
  e[3] = 0.0;
  for (i = 0; i < 7; i++) {
    work[i] = 0.0;
  }
  for (q = 0; q < 4; q++) {
    qp1 = q + 2;
    iter = q + 7 * q;
    i = iter + 1;
    nmqp1_tmp = 6 - q;
    apply_transform = false;
    nrm = xnrm2(7 - q, b_A, iter + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        m = (iter - q) + 7;
        for (k = i; k <= m; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        m = (iter - q) + 7;
        for (k = i; k <= m; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 5; jj++) {
      i = q + 7 * (jj - 1);
      if (apply_transform) {
        nrm = 0.0;
        for (k = 0; k <= nmqp1_tmp; k++) {
          nrm += b_A[iter + k] * b_A[i + k];
        }
        nrm = -(nrm / b_A[iter]);
        if (!(nrm == 0.0)) {
          for (k = 0; k <= nmqp1_tmp; k++) {
            qs = i + k;
            b_A[qs] += nrm * b_A[iter + k];
          }
        }
      }
      e[jj - 1] = b_A[i];
    }
    if (q + 1 <= 2) {
      nrm = b_xnrm2(3 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        nrm = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (k = qp1; k < 5; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (k = qp1; k < 5; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (jj = qp1; jj < 8; jj++) {
          work[jj - 1] = 0.0;
        }
        for (jj = qp1; jj < 5; jj++) {
          nrm = e[jj - 1];
          if (!(nrm == 0.0)) {
            i = q + 7 * (jj - 1);
            m = 5 - q;
            for (k = 0; k <= m; k++) {
              qs = (q + k) + 1;
              work[qs] += nrm * b_A[(i + k) + 1];
            }
          }
        }
        for (jj = qp1; jj < 5; jj++) {
          xaxpy(6 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                (q + 7 * (jj - 1)) + 2);
        }
      }
    }
  }
  m = 2;
  e[2] = b_A[23];
  e[3] = 0.0;
  iter = 0;
  snorm = 0.0;
  nrm = s[0];
  if (s[0] != 0.0) {
    rt = fabs(s[0]);
    r = s[0] / rt;
    nrm = rt;
    s[0] = rt;
    e[0] /= r;
  }
  if (e[0] != 0.0) {
    rt = fabs(e[0]);
    r = rt / e[0];
    e[0] = rt;
    s[1] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[0])) && (!rtIsNaN(e[0]))) {
    nrm = e[0];
  }
  if ((!(nrm <= 0.0)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[1];
  if (s[1] != 0.0) {
    rt = fabs(s[1]);
    r = s[1] / rt;
    nrm = rt;
    s[1] = rt;
    e[1] /= r;
  }
  if (e[1] != 0.0) {
    rt = fabs(e[1]);
    r = rt / e[1];
    e[1] = rt;
    s[2] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[1])) && (!rtIsNaN(e[1]))) {
    nrm = e[1];
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[2];
  if (s[2] != 0.0) {
    rt = fabs(s[2]);
    r = s[2] / rt;
    nrm = rt;
    s[2] = rt;
    e[2] = b_A[23] / r;
  }
  if (e[2] != 0.0) {
    rt = fabs(e[2]);
    r = rt / e[2];
    e[2] = rt;
    s[3] *= r;
  }
  nrm = fabs(nrm);
  if ((!(nrm >= e[2])) && (!rtIsNaN(e[2]))) {
    nrm = e[2];
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  nrm = s[3];
  if (s[3] != 0.0) {
    rt = fabs(s[3]);
    nrm = rt;
    s[3] = rt;
  }
  nrm = fabs(nrm);
  if (!(nrm >= 0.0)) {
    nrm = 0.0;
  }
  if ((!(snorm >= nrm)) && (!rtIsNaN(nrm))) {
    snorm = nrm;
  }
  while ((m + 2 > 0) && (iter < 75)) {
    nmqp1_tmp = m + 1;
    jj = m + 1;
    exitg1 = false;
    while (!(exitg1 || (jj == 0))) {
      nrm = fabs(e[jj - 1]);
      if ((nrm <= 2.2204460492503131E-16 * (fabs(s[jj - 1]) + fabs(s[jj]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[jj - 1] = 0.0;
        exitg1 = true;
      } else {
        jj--;
      }
    }
    if (jj == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg1 = false;
      while ((!exitg1) && (i >= jj)) {
        qs = i;
        if (i == jj) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (i < m + 2) {
            nrm = fabs(e[i - 1]);
          }
          if (i > jj + 1) {
            nrm += fabs(e[i - 2]);
          }
          rt = fabs(s[i - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[i - 1] = 0.0;
            exitg1 = true;
          } else {
            i--;
          }
        }
      }
      if (qs == jj) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        jj = qs;
      }
    }
    switch (i) {
    case 1:
      rt = e[m];
      e[m] = 0.0;
      for (k = nmqp1_tmp; k >= jj + 1; k--) {
        sm = xrotg(&s[k - 1], &rt, &sqds);
        if (k > jj + 1) {
          r = e[k - 2];
          rt = -sqds * r;
          e[k - 2] = r * sm;
        }
      }
      break;
    case 2:
      rt = e[jj - 1];
      e[jj - 1] = 0.0;
      for (k = jj + 1; k <= m + 2; k++) {
        sm = xrotg(&s[k - 1], &rt, &sqds);
        r = e[k - 1];
        rt = -sqds * r;
        e[k - 1] = r * sm;
      }
      break;
    case 3:
      rt = s[m + 1];
      scale = fabs(rt);
      nrm = fabs(s[m]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(e[m]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(s[jj]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      nrm = fabs(e[jj]);
      if ((!(scale >= nrm)) && (!rtIsNaN(nrm))) {
        scale = nrm;
      }
      sm = rt / scale;
      nrm = s[m] / scale;
      rt = e[m] / scale;
      sqds = s[jj] / scale;
      r = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((r != 0.0) || (nrm != 0.0)) {
        rt = sqrt(r * r + nrm);
        if (r < 0.0) {
          rt = -rt;
        }
        rt = nrm / (r + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[jj] / scale);
      for (k = jj + 1; k <= nmqp1_tmp; k++) {
        sm = xrotg(&rt, &nrm, &sqds);
        if (k > jj + 1) {
          e[k - 2] = rt;
        }
        nrm = e[k - 1];
        r = s[k - 1];
        e[k - 1] = sm * nrm - sqds * r;
        rt = sqds * s[k];
        s[k] *= sm;
        s[k - 1] = sm * r + sqds * nrm;
        sm = xrotg(&s[k - 1], &rt, &sqds);
        r = e[k - 1];
        rt = sm * r + sqds * s[k];
        s[k] = -sqds * r + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
      }
      e[m] = rt;
      iter++;
      break;
    default:
      if (s[jj] < 0.0) {
        s[jj] = -s[jj];
      }
      qp1 = jj + 1;
      while ((jj + 1 < 4) && (s[jj] < s[qp1])) {
        rt = s[jj];
        s[jj] = s[qp1];
        s[qp1] = rt;
        jj = qp1;
        qp1++;
      }
      iter = 0;
      m--;
      break;
    }
  }
  U[0] = s[0];
  U[1] = s[1];
  U[2] = s[2];
  U[3] = s[3];
}

/*
 * File trailer for svd.c
 *
 * [EOF]
 */
