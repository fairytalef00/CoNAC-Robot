/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rtwtypes.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 10-Apr-2025 17:48:40
 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int64_T                      - signed 64 bit integers               *
 *   uint64_T                     - unsigned 64 bit integers             *
 *=======================================================================*/

#if defined(__APPLE__)
#ifndef INT64_T
#define INT64_T long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

#if defined(__APPLE__)
#ifndef UINT64_T
#define UINT64_T unsigned long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

// /* Include Files */
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t uint32_T;
typedef int32_t int32_T;
typedef uint16_t uint16_T;
typedef int16_t int16_T;
typedef uint8_t uint8_T;
typedef int8_t int8_T;

typedef float real32_T;
typedef double real_T;
typedef bool boolean_T;

#endif

// /* Include Files */
// #include "tmwtypes.h"

// #endif
/*
 * File trailer for rtwtypes.h
 *
 * [EOF]
 */
