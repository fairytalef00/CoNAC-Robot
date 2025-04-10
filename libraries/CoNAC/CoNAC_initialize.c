/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CoNAC_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 10-Apr-2025 18:08:03
 */

/* Include Files */
#include "CoNAC_initialize.h"
#include "CoNAC_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void CoNAC_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_CoNAC = true;
}

/*
 * File trailer for CoNAC_initialize.c
 *
 * [EOF]
 */
