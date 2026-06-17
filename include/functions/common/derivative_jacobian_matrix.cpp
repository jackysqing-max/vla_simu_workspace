//
// File: derivative_jacobian_matrix.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 17-Apr-2022 12:11:48
//

// Include Files
#include "derivative_jacobian_matrix.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const struct0_T *robot
//                const coder::array<double, 2U> &q
//                const coder::array<double, 2U> &qd
//                coder::array<double, 2U> &dJb
//                coder::array<double, 2U> &Jb
//                double dT[16]
//                double T[16]
// Return Type  : void
//
void derivative_jacobian_matrix(const Robot *robot, const coder::array<
  double, 2U> &q, const coder::array<double, 2U> &qd, coder::array<double, 2U>
  &dJb, coder::array<double, 2U> &Jb, double dT[16], double T[16])
{
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_tform[36];
  double dInvT[16];
  double dv[16];
  double tform[16];
  double W[9];
  double b_W[9];
  double so_w_hat[9];
  double sa[7];
  double y[6];
  double theta;
  double w_hat_idx_0;
  int i;
  signed char b_I[9];
  Jb.set_size(6, (static_cast<int>(robot->dof)));
  dJb.set_size(6, (static_cast<int>(robot->dof)));
  for (i = 0; i < 16; i++) {
    T[i] = robot->ME[i];
    dT[i] = 0.0;
  }

  i = static_cast<int>(((-1.0 - robot->dof) + 1.0) / -1.0);
  for (int b_i = 0; b_i < i; b_i++) {
    double absxk;
    double c_i;
    double scale;
    double t;
    int W_tmp;
    int b_tform_tmp;
    int i1;
    int tform_tmp;
    c_i = robot->dof + -static_cast<double>(b_i);
    for (i1 = 0; i1 < 3; i1++) {
      W[3 * i1] = dT[i1];
      so_w_hat[3 * i1] = T[i1];
      W_tmp = 3 * i1 + 1;
      W[W_tmp] = dT[i1 + 4];
      so_w_hat[W_tmp] = T[i1 + 4];
      W_tmp = 3 * i1 + 2;
      W[W_tmp] = dT[i1 + 8];
      so_w_hat[W_tmp] = T[i1 + 8];
    }

    for (i1 = 0; i1 < 9; i1++) {
      b_W[i1] = -W[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      W_tmp = i1 << 2;
      dInvT[W_tmp] = W[3 * i1];
      dInvT[W_tmp + 1] = W[3 * i1 + 1];
      dInvT[W_tmp + 2] = W[3 * i1 + 2];
      dInvT[i1 + 12] = ((b_W[i1] * T[12] + b_W[i1 + 3] * T[13]) + b_W[i1 + 6] *
                        T[14]) - ((so_w_hat[i1] * dT[12] + so_w_hat[i1 + 3] *
        dT[13]) + so_w_hat[i1 + 6] * dT[14]);
    }

    dInvT[3] = 0.0;
    dInvT[7] = 0.0;
    dInvT[11] = 0.0;
    dInvT[15] = 0.0;
    for (i1 = 0; i1 < 9; i1++) {
      b_W[i1] = -so_w_hat[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      W_tmp = i1 << 2;
      tform[W_tmp] = so_w_hat[3 * i1];
      tform[W_tmp + 1] = so_w_hat[3 * i1 + 1];
      tform[W_tmp + 2] = so_w_hat[3 * i1 + 2];
      tform[i1 + 12] = (b_W[i1] * T[12] + b_W[i1 + 3] * T[13]) + b_W[i1 + 6] *
        T[14];
    }

    tform[3] = 0.0;
    tform[7] = 0.0;
    tform[11] = 0.0;
    tform[15] = 1.0;

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    W[0] = 0.0;
    W[3] = -tform[14];
    W[6] = tform[13];
    W[1] = tform[14];
    W[4] = 0.0;
    W[7] = -tform[12];
    W[2] = -tform[13];
    W[5] = tform[12];
    W[8] = 0.0;

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    for (i1 = 0; i1 < 3; i1++) {
      absxk = W[i1];
      t = W[i1 + 3];
      theta = W[i1 + 6];
      for (tform_tmp = 0; tform_tmp < 3; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        b_W[i1 + 3 * tform_tmp] = (absxk * tform[b_tform_tmp] + t *
          tform[b_tform_tmp + 1]) + theta * tform[b_tform_tmp + 2];
        b_tform[tform_tmp + 6 * i1] = tform[tform_tmp + (i1 << 2)];
        b_tform[tform_tmp + 6 * (i1 + 3)] = 0.0;
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      b_tform[6 * i1 + 3] = b_W[3 * i1];
      W_tmp = i1 << 2;
      b_tform_tmp = 6 * (i1 + 3);
      b_tform[b_tform_tmp + 3] = tform[W_tmp];
      b_tform[6 * i1 + 4] = b_W[3 * i1 + 1];
      b_tform[b_tform_tmp + 4] = tform[W_tmp + 1];
      b_tform[6 * i1 + 5] = b_W[3 * i1 + 2];
      b_tform[b_tform_tmp + 5] = tform[W_tmp + 2];
    }

    for (i1 = 0; i1 < 6; i1++) {
      Jb[i1 + 6 * (static_cast<int>(c_i) - 1)] = 0.0;
      for (tform_tmp = 0; tform_tmp < 6; tform_tmp++) {
        Jb[i1 + 6 * (static_cast<int>(c_i) - 1)] = Jb[i1 + 6 * (static_cast<int>
          (c_i) - 1)] + b_tform[i1 + 6 * tform_tmp] * robot->A[(static_cast<int>
          (c_i) + robot->A.size(0) * tform_tmp) - 1];
      }
    }

    b_W[0] = 0.0;
    b_W[3] = -dInvT[14];
    b_W[6] = dInvT[13];
    b_W[1] = dInvT[14];
    b_W[4] = 0.0;
    b_W[7] = -dInvT[12];
    b_W[2] = -dInvT[13];
    b_W[5] = dInvT[12];
    b_W[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      absxk = b_W[i1];
      t = b_W[i1 + 3];
      theta = b_W[i1 + 6];
      for (tform_tmp = 0; tform_tmp < 3; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        so_w_hat[i1 + 3 * tform_tmp] = (absxk * tform[b_tform_tmp] + t *
          tform[b_tform_tmp + 1]) + theta * tform[b_tform_tmp + 2];
      }

      absxk = W[i1];
      t = W[i1 + 3];
      theta = W[i1 + 6];
      for (tform_tmp = 0; tform_tmp < 3; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        b_W[i1 + 3 * tform_tmp] = (absxk * dInvT[b_tform_tmp] + t *
          dInvT[b_tform_tmp + 1]) + theta * dInvT[b_tform_tmp + 2];
        b_tform[tform_tmp + 6 * i1] = dInvT[tform_tmp + (i1 << 2)];
        b_tform[tform_tmp + 6 * (i1 + 3)] = 0.0;
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      b_tform[6 * i1 + 3] = so_w_hat[3 * i1] + b_W[3 * i1];
      W_tmp = i1 << 2;
      b_tform_tmp = 6 * (i1 + 3);
      b_tform[b_tform_tmp + 3] = dInvT[W_tmp];
      tform_tmp = 3 * i1 + 1;
      b_tform[6 * i1 + 4] = so_w_hat[tform_tmp] + b_W[tform_tmp];
      b_tform[b_tform_tmp + 4] = dInvT[W_tmp + 1];
      tform_tmp = 3 * i1 + 2;
      b_tform[6 * i1 + 5] = so_w_hat[tform_tmp] + b_W[tform_tmp];
      b_tform[b_tform_tmp + 5] = dInvT[W_tmp + 2];
    }

    w_hat_idx_0 = q[static_cast<int>(c_i) - 1];
    for (i1 = 0; i1 < 6; i1++) {
      dJb[i1 + 6 * (static_cast<int>(c_i) - 1)] = 0.0;
      for (tform_tmp = 0; tform_tmp < 6; tform_tmp++) {
        dJb[i1 + 6 * (static_cast<int>(c_i) - 1)] = dJb[i1 + 6 * (static_cast<
          int>(c_i) - 1)] + b_tform[i1 + 6 * tform_tmp] * robot->A[(static_cast<
          int>(c_i) + robot->A.size(0) * tform_tmp) - 1];
      }

      y[i1] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * i1) - 1] *
        w_hat_idx_0;
    }

    std::memset(&dInvT[0], 0, 16U * sizeof(double));
    dInvT[15] = 1.0;

    //  v is nx6 each row represents a twist
    for (i1 = 0; i1 < 7; i1++) {
      sa[i1] = 0.0;
    }

    w_hat_idx_0 = 0.0;
    scale = 3.3121686421112381E-170;
    for (W_tmp = 0; W_tmp < 6; W_tmp++) {
      absxk = std::abs(y[W_tmp]);
      if (absxk > scale) {
        t = scale / absxk;
        w_hat_idx_0 = w_hat_idx_0 * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        w_hat_idx_0 += t * t;
      }
    }

    w_hat_idx_0 = scale * std::sqrt(w_hat_idx_0);
    if (w_hat_idx_0 <= 2.2204460492503131E-16) {
      sa[5] = 1.0;
    } else {
      scale = 3.3121686421112381E-170;
      absxk = std::abs(y[0]);
      if (absxk > 3.3121686421112381E-170) {
        theta = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        theta = t * t;
      }

      absxk = std::abs(y[1]);
      if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        theta += t * t;
      }

      absxk = std::abs(y[2]);
      if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        theta += t * t;
      }

      theta = scale * std::sqrt(theta);
      if (theta <= 2.2204460492503131E-16) {
        scale = 3.3121686421112381E-170;
        absxk = std::abs(y[3]);
        if (absxk > 3.3121686421112381E-170) {
          w_hat_idx_0 = 1.0;
          scale = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          w_hat_idx_0 = t * t;
        }

        absxk = std::abs(y[4]);
        if (absxk > scale) {
          t = scale / absxk;
          w_hat_idx_0 = w_hat_idx_0 * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          w_hat_idx_0 += t * t;
        }

        absxk = std::abs(y[5]);
        if (absxk > scale) {
          t = scale / absxk;
          w_hat_idx_0 = w_hat_idx_0 * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          w_hat_idx_0 += t * t;
        }

        w_hat_idx_0 = scale * std::sqrt(w_hat_idx_0);
        sa[6] = w_hat_idx_0;
        sa[3] = y[3] / w_hat_idx_0;
        sa[4] = y[4] / w_hat_idx_0;
        sa[5] = y[5] / w_hat_idx_0;
      } else {
        sa[6] = theta;
        for (i1 = 0; i1 < 6; i1++) {
          sa[i1] = y[i1] / theta;
        }
      }
    }

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    W[0] = 0.0;
    W[3] = -sa[2];
    W[6] = sa[1];
    W[1] = sa[2];
    W[4] = 0.0;
    W[7] = -sa[0];
    W[2] = -sa[1];
    W[5] = sa[0];
    W[8] = 0.0;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(y[0]);
    if (absxk > 3.3121686421112381E-170) {
      theta = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta = t * t;
    }

    absxk = std::abs(y[1]);
    if (absxk > scale) {
      t = scale / absxk;
      theta = theta * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta += t * t;
    }

    absxk = std::abs(y[2]);
    if (absxk > scale) {
      t = scale / absxk;
      theta = theta * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta += t * t;
    }

    theta = scale * std::sqrt(theta);
    if (theta <= 2.2204460492503131E-16) {
      std::memset(&so_w_hat[0], 0, 9U * sizeof(double));
      so_w_hat[0] = 1.0;
      so_w_hat[4] = 1.0;
      so_w_hat[8] = 1.0;
    } else {
      w_hat_idx_0 = y[0] / theta;
      scale = y[1] / theta;
      absxk = y[2] / theta;

      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      so_w_hat[0] = 0.0;
      so_w_hat[3] = -absxk;
      so_w_hat[6] = scale;
      so_w_hat[1] = absxk;
      so_w_hat[4] = 0.0;
      so_w_hat[7] = -w_hat_idx_0;
      so_w_hat[2] = -scale;
      so_w_hat[5] = w_hat_idx_0;
      so_w_hat[8] = 0.0;
      scale = std::sin(theta);
      w_hat_idx_0 = 1.0 - std::cos(theta);
      for (i1 = 0; i1 < 9; i1++) {
        b_I[i1] = 0;
      }

      for (W_tmp = 0; W_tmp < 3; W_tmp++) {
        b_I[W_tmp + 3 * W_tmp] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          b_W[W_tmp + 3 * i1] = (so_w_hat[W_tmp] * so_w_hat[3 * i1] +
            so_w_hat[W_tmp + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[W_tmp + 6] *
            so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        so_w_hat[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) +
          w_hat_idx_0 * b_W[i1];
      }
    }

    w_hat_idx_0 = std::cos(sa[6]);
    scale = sa[6] - std::sin(sa[6]);
    for (i1 = 0; i1 < 3; i1++) {
      for (tform_tmp = 0; tform_tmp < 3; tform_tmp++) {
        dInvT[tform_tmp + (i1 << 2)] = so_w_hat[tform_tmp + 3 * i1];
        W_tmp = i1 + 3 * tform_tmp;
        b_W[W_tmp] = (static_cast<double>(a[W_tmp]) * sa[6] + (1.0 - w_hat_idx_0)
                      * W[W_tmp]) + ((scale * W[i1] * W[3 * tform_tmp] + scale *
          W[i1 + 3] * W[3 * tform_tmp + 1]) + scale * W[i1 + 6] * W[3 *
          tform_tmp + 2]);
      }

      dInvT[i1 + 12] = (b_W[i1] * sa[3] + b_W[i1 + 3] * sa[4]) + b_W[i1 + 6] *
        sa[5];
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (tform_tmp = 0; tform_tmp < 4; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        tform[i1 + b_tform_tmp] = ((robot->M[i1 + 16 * (static_cast<int>(c_i) -
          1)] * dInvT[b_tform_tmp] + robot->M[(i1 + 16 * (static_cast<int>(c_i)
          - 1)) + 4] * dInvT[b_tform_tmp + 1]) + robot->M[(i1 + 16 * (
          static_cast<int>(c_i) - 1)) + 8] * dInvT[b_tform_tmp + 2]) + robot->M
          [(i1 + 16 * (static_cast<int>(c_i) - 1)) + 12] * dInvT[b_tform_tmp + 3];
      }
    }

    //  v -> [v]
    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    dInvT[0] = 0.0;
    w_hat_idx_0 = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * 2) - 1];
    dInvT[4] = -w_hat_idx_0;
    scale = robot->A[(static_cast<int>(c_i) + robot->A.size(0)) - 1];
    dInvT[8] = scale;
    dInvT[1] = w_hat_idx_0;
    dInvT[5] = 0.0;
    w_hat_idx_0 = robot->A[static_cast<int>(c_i) - 1];
    dInvT[9] = -w_hat_idx_0;
    dInvT[2] = -scale;
    dInvT[6] = w_hat_idx_0;
    dInvT[10] = 0.0;
    dInvT[12] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * 3) - 1];
    dInvT[13] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * 4) - 1];
    dInvT[14] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * 5) - 1];
    dInvT[3] = 0.0;
    dInvT[7] = 0.0;
    dInvT[11] = 0.0;
    dInvT[15] = 0.0;
    w_hat_idx_0 = qd[static_cast<int>(c_i) - 1];
    for (i1 = 0; i1 < 4; i1++) {
      absxk = dInvT[i1];
      t = dInvT[i1 + 4];
      theta = dInvT[i1 + 8];
      scale = dInvT[i1 + 12];
      for (tform_tmp = 0; tform_tmp < 4; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        dv[i1 + b_tform_tmp] = ((absxk * T[b_tform_tmp] + t * T[b_tform_tmp + 1])
          + theta * T[b_tform_tmp + 2]) + scale * T[b_tform_tmp + 3];
      }
    }

    for (i1 = 0; i1 < 16; i1++) {
      dv[i1] = dv[i1] * w_hat_idx_0 + dT[i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      absxk = tform[i1];
      t = tform[i1 + 4];
      theta = tform[i1 + 8];
      scale = tform[i1 + 12];
      for (tform_tmp = 0; tform_tmp < 4; tform_tmp++) {
        b_tform_tmp = tform_tmp << 2;
        W_tmp = i1 + b_tform_tmp;
        dInvT[W_tmp] = ((absxk * T[b_tform_tmp] + t * T[b_tform_tmp + 1]) +
                        theta * T[b_tform_tmp + 2]) + scale * T[b_tform_tmp + 3];
        dT[W_tmp] = ((absxk * dv[b_tform_tmp] + t * dv[b_tform_tmp + 1]) + theta
                     * dv[b_tform_tmp + 2]) + scale * dv[b_tform_tmp + 3];
      }
    }

    std::memcpy(&T[0], &dInvT[0], 16U * sizeof(double));
  }
}

//
// File trailer for derivative_jacobian_matrix.cpp
//
// [EOF]
//
