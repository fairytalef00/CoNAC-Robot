/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CoNAC.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 14-Apr-2025 21:10:39
 */

#ifndef CONAC_H
#define CONAC_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void CoNAC(const double x1[2], const double xd1[2], const double x2[2],
                  const double xd2[2], double th[58], double lbd[8],
                  double zeta[2], const double A_zeta[4],
                  const double B_zeta[4], double alp1, double alp2,
                  double ctrl_dt, const double Lambda[4],
                  const double th_max[3], double u_ball, const double beta[8],
                  double uMax1, double uMax2, double CONTROL_NUM, double rho,
                  double out[2], double Vn[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CoNAC.h
 *
 * [EOF]
 */
