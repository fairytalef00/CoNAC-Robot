/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CoNAC.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 14-Apr-2025 21:10:39
 */

/* Include Files */
#include "CoNAC.h"
#include "CoNAC_data.h"
#include "CoNAC_initialize.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Parameters
 *
 * Arguments    : const double x1[2]
 *                const double xd1[2]
 *                const double x2[2]
 *                const double xd2[2]
 *                double th[58]
 *                double lbd[8]
 *                double zeta[2]
 *                const double A_zeta[4]
 *                const double B_zeta[4]
 *                double alp1
 *                double alp2
 *                double ctrl_dt
 *                const double Lambda[4]
 *                const double th_max[3]
 *                double u_ball
 *                const double beta[8]
 *                double uMax1
 *                double uMax2
 *                double CONTROL_NUM
 *                double rho
 *                double out[2]
 *                double Vn[3]
 * Return Type  : void
 */
void CoNAC(const double x1[2], const double xd1[2], const double x2[2],
           const double xd2[2], double th[58], double lbd[8], double zeta[2],
           const double A_zeta[4], const double B_zeta[4], double alp1,
           double alp2, double ctrl_dt, const double Lambda[4],
           const double th_max[3], double u_ball, const double beta[8],
           double uMax1, double uMax2, double CONTROL_NUM, double rho,
           double out[2], double Vn[3])
{
  static const signed char b_A[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1};
  static const signed char A[4] = {1, 0, 0, 1};
  double cd_data[464];
  double b_out[290];
  double nnGrad[116];
  double grad2[112];
  double grad1[80];
  double b_a[64];
  double b_grad_to_back1[56];
  double b_grad1[40];
  double V0[28];
  double V1[20];
  double grad0[20];
  double out1_tmp[20];
  double V2[10];
  double out_tmp[10];
  double c_a[9];
  double grad_to_back1[8];
  double in0[7];
  double in1[5];
  double in2[5];
  double dv[4];
  double out0[4];
  double out1[4];
  double a;
  double absx;
  double absxk_tmp;
  double b_absxk_tmp;
  double b_y;
  double d;
  double d1;
  double del_u_idx_1;
  double e_idx_0;
  double e_idx_1;
  double t;
  double y;
  int b_i;
  int i;
  int i1;
  int j2;
  int kidx;
  int th_index;
  signed char cumsum_th[4];
  if (!isInitialized_CoNAC) {
    CoNAC_initialize();
  }
  /*  layer info (node numbers) */
  /*  input layer */
  /*  output layer */
  /*  multiplier number */
  /*     %% FEEDFORWAD  */
  /*  ================================================ */
  /*      FEEDFORWARD (CONTROL INPUT CALCULATION) */
  /*  ================================================ */
  /*  neural network input */
  absx = x1[0] - xd1[0];
  t = x1[1] - xd1[1];
  e_idx_0 = (x2[0] - xd2[0]) + (Lambda[0] * absx + Lambda[2] * t);
  e_idx_1 = (x2[1] - xd2[1]) + (Lambda[1] * absx + Lambda[3] * t);
  /*     %% FORWARD (layer 1) */
  th_index = 0;
  in0[0] = x1[0];
  in0[2] = xd1[0];
  in0[4] = e_idx_0;
  in0[1] = x1[1];
  in0[3] = xd1[1];
  in0[5] = e_idx_1;
  in0[6] = 1.0;
  /*  pointer to next */
  /*  current norm */
  y = 0.0;
  for (kidx = 0; kidx < 4; kidx++) {
    d = 0.0;
    for (i = 0; i < 7; i++) {
      d1 = th[th_index + i];
      V0[i + 7 * kidx] = d1;
      d += d1 * in0[i];
      absx = fabs(d1);
      if (rtIsNaN(absx) || (absx > y)) {
        y = absx;
      }
    }
    th_index += 7;
    out0[kidx] = d;
  }
  if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
    svd(V0, dv);
    y = dv[0];
  }
  Vn[0] = y;
  /*     %% FORWARD (layer 2) */
  th_index = 28;
  for (kidx = 0; kidx < 4; kidx++) {
    for (i = 0; i < 5; i++) {
      V1[i + 5 * kidx] = th[th_index + i];
    }
    th_index += 5;
    in1[kidx] = tanh(out0[kidx]);
  }
  in1[4] = 1.0;
  for (b_i = 0; b_i < 5; b_i++) {
    th_index = b_i << 2;
    out1_tmp[th_index] = V1[b_i];
    out1_tmp[th_index + 1] = V1[b_i + 5];
    out1_tmp[th_index + 2] = V1[b_i + 10];
    out1_tmp[th_index + 3] = V1[b_i + 15];
  }
  /*  pointer to next */
  /*  current norm */
  b_y = 0.0;
  for (kidx = 0; kidx < 4; kidx++) {
    d = 0.0;
    for (i = 0; i < 5; i++) {
      d += out1_tmp[kidx + (i << 2)] * in1[i];
      absx = fabs(V1[i + 5 * kidx]);
      if (rtIsNaN(absx) || (absx > b_y)) {
        b_y = absx;
      }
    }
    out1[kidx] = d;
  }
  if ((!rtIsInf(b_y)) && (!rtIsNaN(b_y))) {
    b_svd(V1, dv);
    b_y = dv[0];
  }
  Vn[1] = b_y;
  /*     %% FORWARD (layer 3) */
  th_index = 48;
  for (kidx = 0; kidx < 2; kidx++) {
    for (i = 0; i < 5; i++) {
      V2[i + 5 * kidx] = th[th_index + i];
    }
    th_index += 5;
  }
  a = tanh(out1[0]);
  in2[0] = a;
  absx = tanh(out1[1]);
  in2[1] = absx;
  del_u_idx_1 = tanh(out1[2]);
  in2[2] = del_u_idx_1;
  t = tanh(out1[3]);
  in2[3] = t;
  in2[4] = 1.0;
  for (b_i = 0; b_i < 5; b_i++) {
    th_index = b_i << 1;
    out_tmp[th_index] = V2[b_i];
    out_tmp[th_index + 1] = V2[b_i + 5];
  }
  /*  pointer to next */
  /*  current norm */
  Vn[2] = b_norm(V2);
  /*     %% RESULT */
  /*     %% DEAD-ZONE */
  /*  ================================================ */
  /*                    STOP UPDATE */
  /*  ================================================ */
  /*  if norm(e) < e_tol */
  /*      th_grad = zeros(th_size, 1); */
  /*      lbd_grad = zeros(lbd_size, 1); */
  /*      return; */
  /*  end */
  /*     %% NN GRADIENT CALCULATION */
  /*  ================================================ */
  /*        NEURAL NETWORK JACOBIAN CALCULATION */
  /*  ================================================ */
  /*     %% BACKPROPAGATION (layer end-0) */
  /*  (I \otimes phi_k^T) */
  kidx = -1;
  for (i = 0; i < 2; i++) {
    d = 0.0;
    th_index = i << 1;
    for (j2 = 0; j2 < 5; j2++) {
      d1 = in2[j2];
      d += out_tmp[i + (j2 << 1)] * d1;
      grad0[kidx + 1] = (double)A[th_index] * d1;
      grad0[kidx + 2] = (double)A[th_index + 1] * d1;
      kidx += 2;
    }
    out[i] = d;
  }
  /*     %% BACKPROPAGATION (layer end-1) */
  kidx = -1;
  for (i = 0; i < 4; i++) {
    th_index = i << 2;
    for (j2 = 0; j2 < 5; j2++) {
      d = in1[j2];
      grad1[kidx + 1] = (double)b_A[th_index] * d;
      grad1[kidx + 2] = (double)b_A[th_index + 1] * d;
      grad1[kidx + 3] = (double)b_A[th_index + 2] * d;
      grad1[kidx + 4] = (double)b_A[th_index + 3] * d;
      kidx += 4;
    }
  }
  /*  LOCAL FUNCTIONS */
  memset(&V1[0], 0, 20U * sizeof(double));
  V1[0] = 1.0 - a * a;
  V1[6] = 1.0 - absx * absx;
  V1[12] = 1.0 - del_u_idx_1 * del_u_idx_1;
  V1[18] = 1.0 - t * t;
  for (b_i = 0; b_i < 2; b_i++) {
    for (i1 = 0; i1 < 4; i1++) {
      d = 0.0;
      for (i = 0; i < 5; i++) {
        d += out_tmp[b_i + (i << 1)] * V1[i + 5 * i1];
      }
      grad_to_back1[b_i + (i1 << 1)] = d;
    }
    d = grad_to_back1[b_i];
    d1 = grad_to_back1[b_i + 2];
    a = grad_to_back1[b_i + 4];
    del_u_idx_1 = grad_to_back1[b_i + 6];
    for (i1 = 0; i1 < 20; i1++) {
      i = i1 << 2;
      b_grad1[b_i + (i1 << 1)] =
          ((d * grad1[i] + d1 * grad1[i + 1]) + a * grad1[i + 2]) +
          del_u_idx_1 * grad1[i + 3];
    }
  }
  /*     %% BACKPROPAGATION (layer end-2) */
  kidx = -1;
  for (i = 0; i < 4; i++) {
    th_index = i << 2;
    for (j2 = 0; j2 < 7; j2++) {
      d = in0[j2];
      grad2[kidx + 1] = (double)b_A[th_index] * d;
      grad2[kidx + 2] = (double)b_A[th_index + 1] * d;
      grad2[kidx + 3] = (double)b_A[th_index + 2] * d;
      grad2[kidx + 4] = (double)b_A[th_index + 3] * d;
      kidx += 4;
    }
  }
  /*  LOCAL FUNCTIONS */
  memset(&V1[0], 0, 20U * sizeof(double));
  a = tanh(out0[0]);
  V1[0] = 1.0 - a * a;
  a = tanh(out0[1]);
  V1[6] = 1.0 - a * a;
  a = tanh(out0[2]);
  V1[12] = 1.0 - a * a;
  a = tanh(out0[3]);
  V1[18] = 1.0 - a * a;
  /*     %% RESULT */
  for (b_i = 0; b_i < 2; b_i++) {
    d = grad_to_back1[b_i];
    d1 = grad_to_back1[b_i + 2];
    a = grad_to_back1[b_i + 4];
    del_u_idx_1 = grad_to_back1[b_i + 6];
    for (i1 = 0; i1 < 5; i1++) {
      i = i1 << 2;
      out_tmp[b_i + (i1 << 1)] =
          ((d * out1_tmp[i] + d1 * out1_tmp[i + 1]) + a * out1_tmp[i + 2]) +
          del_u_idx_1 * out1_tmp[i + 3];
    }
    for (i1 = 0; i1 < 4; i1++) {
      d = 0.0;
      for (i = 0; i < 5; i++) {
        d += out_tmp[b_i + (i << 1)] * V1[i + 5 * i1];
      }
      grad_to_back1[b_i + (i1 << 1)] = d;
    }
    d = grad_to_back1[b_i];
    d1 = grad_to_back1[b_i + 2];
    a = grad_to_back1[b_i + 4];
    del_u_idx_1 = grad_to_back1[b_i + 6];
    for (i1 = 0; i1 < 28; i1++) {
      i = i1 << 2;
      b_grad_to_back1[b_i + (i1 << 1)] =
          ((d * grad2[i] + d1 * grad2[i + 1]) + a * grad2[i + 2]) +
          del_u_idx_1 * grad2[i + 3];
    }
  }
  for (b_i = 0; b_i < 28; b_i++) {
    th_index = b_i << 1;
    nnGrad[th_index] = b_grad_to_back1[th_index];
    nnGrad[th_index + 1] = b_grad_to_back1[th_index + 1];
  }
  for (b_i = 0; b_i < 20; b_i++) {
    th_index = b_i << 1;
    kidx = (b_i + 28) << 1;
    nnGrad[kidx] = b_grad1[th_index];
    nnGrad[kidx + 1] = b_grad1[th_index + 1];
  }
  for (b_i = 0; b_i < 10; b_i++) {
    th_index = b_i << 1;
    kidx = (b_i + 48) << 1;
    nnGrad[kidx] = grad0[th_index];
    nnGrad[kidx + 1] = grad0[th_index + 1];
  }
  /*  ERROR CHECK */
  /*  assert(pt_V == (NN_size(1)+1)*NN_size(2)); */
  /*  assert(pt_tape == 0); */
  /*     %% CONSTRINT HANDLING */
  /*  ================================================ */
  /*       */
  /*  ================================================ */
  /*     %% CONSTRAINT FUNC. CALCULATION */
  cumsum_th[0] = 0;
  /*  ball condition  */
  cumsum_th[1] = 28;
  cumsum_th[2] = 48;
  cumsum_th[3] = 58;
  /*  input ball  */
  absx = 3.3121686421112381E-170;
  absxk_tmp = fabs(out[0]);
  if (absxk_tmp > 3.3121686421112381E-170) {
    a = 1.0;
    absx = absxk_tmp;
  } else {
    t = absxk_tmp / 3.3121686421112381E-170;
    a = t * t;
  }
  b_absxk_tmp = fabs(out[1]);
  if (b_absxk_tmp > absx) {
    t = absx / b_absxk_tmp;
    a = a * t * t + 1.0;
    absx = b_absxk_tmp;
  } else {
    t = b_absxk_tmp / absx;
    a += t * t;
  }
  a = absx * sqrt(a);
  /*  input 1 */
  /*  summary */
  grad_to_back1[0] = 0.5 * (y * y - th_max[0] * th_max[0]);
  grad_to_back1[1] = 0.5 * (b_y * b_y - th_max[1] * th_max[1]);
  grad_to_back1[2] = 0.5 * (Vn[2] * Vn[2] - th_max[2] * th_max[2]);
  grad_to_back1[3] = 0.5 * (a * a - u_ball * u_ball);
  grad_to_back1[4] = out[0] - uMax1;
  grad_to_back1[5] = out[1] - uMax2;
  grad_to_back1[6] = -uMax1 - out[0];
  grad_to_back1[7] = -uMax2 - out[1];
  /*     %% CONSTRAINT GREADIENT (dC/dth) */
  memset(&cd_data[0], 0, 464U * sizeof(double));
  /*  for l_idx = 1:1:length(th_max) */
  /*      start_pt = cumsum_V(l_idx)+1; */
  /*      end_pt = cumsum_V(l_idx+1); */
  /*      cd(l_idx, start_pt:end_pt) = 2 * th(start_pt:end_pt,1); */
  /*  end */
  /*  cd(l_idx+1:end, :) = 2*nnGrad'*out; */
  kidx = 1;
  for (j2 = 0; j2 < 3; j2++) {
    kidx = j2 + 1;
    b_i = cumsum_th[j2 + 1];
    i1 = cumsum_th[j2];
    if (i1 + 1 > b_i) {
      i = 1;
      i1 = 0;
      b_i = 0;
    } else {
      i = i1 + 1;
    }
    th_index = b_i - i1;
    for (b_i = 0; b_i < th_index; b_i++) {
      cd_data[j2 + 8 * (i1 + b_i)] = th[(i + b_i) - 1];
    }
  }
  d = out[0];
  d1 = out[1];
  for (b_i = 0; b_i < 58; b_i++) {
    i1 = b_i << 1;
    a = nnGrad[i1];
    del_u_idx_1 = nnGrad[i1 + 1];
    b_out[5 * b_i] = d * a + d1 * del_u_idx_1;
    b_out[5 * b_i + 1] = a;
    b_out[5 * b_i + 2] = del_u_idx_1;
    b_out[5 * b_i + 3] = -a;
    b_out[5 * b_i + 4] = -del_u_idx_1;
  }
  th_index = 8 - kidx;
  for (b_i = 0; b_i < 58; b_i++) {
    for (i1 = 0; i1 < th_index; i1++) {
      cd_data[(kidx + i1) + 8 * b_i] = b_out[i1 + (8 - kidx) * b_i];
    }
  }
  /*     %% BACKPROPAGATION  */
  /*  ================================================ */
  /*      BACKPROPAGATION (NEURAL NETWROK UPDATE) */
  /*  ================================================ */
  if (CONTROL_NUM == 2.0) {
    absx = 0.0;
    del_u_idx_1 = 0.0;
    if (absxk_tmp > uMax1) {
      if (rtIsNaN(out[0])) {
        d = rtNaN;
      } else if (out[0] < 0.0) {
        d = -1.0;
      } else {
        d = (out[0] > 0.0);
      }
      absx = out[0] - uMax1 * d;
    }
    if (b_absxk_tmp > uMax2) {
      if (rtIsNaN(out[1])) {
        d = rtNaN;
      } else if (out[1] < 0.0) {
        d = -1.0;
      } else {
        d = (out[1] > 0.0);
      }
      del_u_idx_1 = out[1] - uMax2 * d;
    }
    t = zeta[0] * A_zeta[1] + zeta[1] * A_zeta[3];
    zeta[0] += ((A_zeta[0] * zeta[0] + zeta[1] * A_zeta[2]) +
                (B_zeta[0] * absx + B_zeta[2] * del_u_idx_1)) *
               ctrl_dt;
    zeta[1] += (t + (B_zeta[1] * absx + B_zeta[3] * del_u_idx_1)) * ctrl_dt;
    e_idx_0 += zeta[0];
    e_idx_1 += zeta[1];
    d = lbd[0];
    d1 = lbd[1];
    a = lbd[2];
    for (i = 0; i < 58; i++) {
      th_index = i * 3;
      b_i = i << 1;
      del_u_idx_1 = nnGrad[b_i];
      absx = nnGrad[b_i + 1];
      t = th[i];
      t += -((((del_u_idx_1 * alp1 + absx * 0.0) * e_idx_0 +
               (del_u_idx_1 * 0.0 + absx * alp2) * e_idx_1) +
              ((cd_data[th_index % 3 + 8 * (th_index / 3)] * d +
                cd_data[(th_index + 1) % 3 + 8 * ((th_index + 1) / 3)] * d1) +
               cd_data[(th_index + 2) % 3 + 8 * ((th_index + 2) / 3)] * a)) +
             rho * t) *
           ctrl_dt;
      th[i] = t;
    }
    memset(&c_a[0], 0, 9U * sizeof(double));
    c_a[0] = beta[0];
    c_a[4] = beta[1];
    c_a[8] = beta[2];
    d = grad_to_back1[0];
    d1 = grad_to_back1[1];
    a = grad_to_back1[2];
    for (kidx = 0; kidx < 3; kidx++) {
      del_u_idx_1 =
          lbd[kidx] +
          ((c_a[kidx] * d + c_a[kidx + 3] * d1) + c_a[kidx + 6] * a) * ctrl_dt;
      if (del_u_idx_1 >= 0.0) {
        lbd[kidx] = del_u_idx_1;
      } else {
        lbd[kidx] = 0.0;
      }
    }
  } else {
    for (i = 0; i < 58; i++) {
      th_index = i << 3;
      t = 0.0;
      for (kidx = 0; kidx < 8; kidx++) {
        t += cd_data[th_index + kidx] * lbd[kidx];
      }
      b_i = i << 1;
      d = nnGrad[b_i];
      d1 = nnGrad[b_i + 1];
      a = th[i];
      a += -((((d * alp1 + d1 * 0.0) * e_idx_0 +
               (d * 0.0 + d1 * alp2) * e_idx_1) +
              t) +
             rho * a) *
           ctrl_dt;
      th[i] = a;
    }
    memset(&b_a[0], 0, 64U * sizeof(double));
    for (kidx = 0; kidx < 8; kidx++) {
      b_a[kidx + (kidx << 3)] = beta[kidx];
    }
    for (kidx = 0; kidx < 8; kidx++) {
      d = 0.0;
      for (b_i = 0; b_i < 8; b_i++) {
        d += b_a[kidx + (b_i << 3)] * grad_to_back1[b_i];
      }
      d = lbd[kidx] + d * ctrl_dt;
      if (d >= 0.0) {
        lbd[kidx] = d;
      } else {
        lbd[kidx] = 0.0;
      }
    }
  }
}

/*
 * File trailer for CoNAC.c
 *
 * [EOF]
 */
