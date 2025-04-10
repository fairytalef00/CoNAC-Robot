/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 10-Apr-2025 18:08:03
 */

#ifndef XAXPY_H
#define XAXPY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xaxpy(int n, double a, const double x[5], int ix0, double y[20],
             int iy0);

void xaxpy(int n, double a, const double x[7], int ix0, double y[28], int iy0);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xaxpy.h
 *
 * [EOF]
 */
