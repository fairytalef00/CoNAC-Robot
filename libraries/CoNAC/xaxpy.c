/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 10-Apr-2025 18:08:03
 */

/* Include Files */
#include "xaxpy.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : int n
 *                double a
 *                const double x[5]
 *                int ix0
 *                double y[20]
 *                int iy0
 * Return Type  : void
 */
void b_xaxpy(int n, double a, const double x[5], int ix0, double y[20], int iy0)
{
  int i;
  int i1;
  int k;
  if (!(a == 0.0)) {
    i = n - 1;
    for (k = 0; k <= i; k++) {
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                const double x[7]
 *                int ix0
 *                double y[28]
 *                int iy0
 * Return Type  : void
 */
void xaxpy(int n, double a, const double x[7], int ix0, double y[28], int iy0)
{
  int i;
  int i1;
  int k;
  if (!(a == 0.0)) {
    i = n - 1;
    for (k = 0; k <= i; k++) {
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

/*
 * File trailer for xaxpy.c
 *
 * [EOF]
 */
