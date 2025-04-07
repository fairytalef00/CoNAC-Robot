/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 19-Mar-2025 20:36:32
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "CoNAC.h"
#include "CoNAC_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_114x1_real_T(double result[114]);

static void argInit_2x1_real_T(double result[2]);

static void argInit_3x1_real_T(double result[3]);

static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : double result[114]
 * Return Type  : void
 */
static void argInit_114x1_real_T(double result[114])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 114; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
static void argInit_2x1_real_T(double result[2])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_3x1_real_T(double result[3])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_CoNAC();
  /* Terminate the application.
You do not need to do this more than one time. */
  CoNAC_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_CoNAC(void)
{
  double th[114];
  double lbd[3];
  double Vn[2];
  double out[2];
  double x_tmp[2];
  /* Initialize function 'CoNAC' input arguments. */
  /* Initialize function input argument 'x'. */
  argInit_2x1_real_T(x_tmp);
  /* Initialize function input argument 'xd'. */
  /* Initialize function input argument 'th'. */
  /* Initialize function input argument 'lbd'. */
  /* Call the entry-point 'CoNAC'. */
  argInit_114x1_real_T(th);
  argInit_3x1_real_T(lbd);
  CoNAC(x_tmp, x_tmp, th, lbd, out, Vn);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
