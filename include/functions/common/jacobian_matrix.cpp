//
// File: jacobian_matrix.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 21:33:31
//

// Include Files
#include "jacobian_matrix.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const struct0_T *robot
//                const coder::array<double, 2U> &q
//                coder::array<double, 2U> &Jb
//                double T[16]
// Return Type  : void
//
void jacobian_matrix(const Robot *robot, const coder::array<double, 2U> &q,
                     coder::array<double, 2U> &Jb, double T[16])
{
  static const signed char b_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double a[36];
  double b_robot[16];
  double invT[16];
  double W[9];
  double b_so_w_hat[9];
  double so_w_hat[9];
  double sa[7];
  double y[6];
  double t;
  double theta;
  double w_hat_idx_0;
  int i;
  signed char b_I[9];
  signed char invT_tmp_tmp[3];
  std::memcpy(&T[0], &robot->ME[0], 16U * sizeof(double));
  Jb.set_size(6, (static_cast<int>(robot->dof)));
  i = static_cast<int>(((-1.0 - robot->dof) + 1.0) / -1.0);
  if (0 <= i - 1) {
    W[0] = 0.0;
    W[4] = 0.0;
    W[8] = 0.0;
  }

  for (int b_i = 0; b_i < i; b_i++) {
    double absxk;
    double c_i;
    double scale;
    int a_tmp;
    int i1;
    int invT_tmp;
    c_i = robot->dof + -static_cast<double>(b_i);

    //  T: 4x4xn
    for (i1 = 0; i1 < 3; i1++) {
      invT_tmp_tmp[i1] = static_cast<signed char>(i1 + 1);
      so_w_hat[3 * i1] = -T[i1];
      so_w_hat[3 * i1 + 1] = -T[i1 + 4];
      so_w_hat[3 * i1 + 2] = -T[i1 + 8];
    }

    for (i1 = 0; i1 < 3; i1++) {
      invT_tmp = i1 << 2;
      invT[invT_tmp] = T[(invT_tmp_tmp[i1] + ((invT_tmp_tmp[0] - 1) << 2)) - 1];
      invT[invT_tmp + 1] = T[(invT_tmp_tmp[i1] + ((invT_tmp_tmp[1] - 1) << 2)) -
        1];
      invT[invT_tmp + 2] = T[(invT_tmp_tmp[i1] + ((invT_tmp_tmp[2] - 1) << 2)) -
        1];
      invT[i1 + 12] = (so_w_hat[i1] * T[12] + so_w_hat[i1 + 3] * T[13]) +
        so_w_hat[i1 + 6] * T[14];
    }

    invT[3] = 0.0;
    invT[7] = 0.0;
    invT[11] = 0.0;
    invT[15] = 1.0;

    //  AdT operator T -> 6x6 mapping
    std::memset(&a[0], 0, 36U * sizeof(double));
    for (i1 = 0; i1 < 3; i1++) {
      int b_a_tmp;
      invT_tmp = (invT_tmp_tmp[i1] - 1) << 2;
      a[6 * i1] = invT[(invT_tmp_tmp[0] + invT_tmp) - 1];
      a_tmp = i1 << 2;
      b_a_tmp = 6 * (i1 + 3);
      a[b_a_tmp + 3] = invT[a_tmp];
      a[6 * i1 + 1] = invT[(invT_tmp_tmp[1] + invT_tmp) - 1];
      a[b_a_tmp + 4] = invT[a_tmp + 1];
      a[6 * i1 + 2] = invT[(invT_tmp_tmp[2] + invT_tmp) - 1];
      a[b_a_tmp + 5] = invT[a_tmp + 2];
    }

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    so_w_hat[0] = 0.0;
    so_w_hat[3] = -invT[14];
    so_w_hat[6] = invT[13];
    so_w_hat[1] = invT[14];
    so_w_hat[4] = 0.0;
    so_w_hat[7] = -invT[12];
    so_w_hat[2] = -invT[13];
    so_w_hat[5] = invT[12];
    so_w_hat[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      absxk = so_w_hat[i1];
      t = so_w_hat[i1 + 3];
      scale = so_w_hat[i1 + 6];
      for (a_tmp = 0; a_tmp < 3; a_tmp++) {
        invT_tmp = a_tmp << 2;
        a[(i1 + 6 * a_tmp) + 3] = (absxk * invT[invT_tmp] + t * invT[invT_tmp +
          1]) + scale * invT[invT_tmp + 2];
      }
    }

    w_hat_idx_0 = q[static_cast<int>(c_i) - 1];
    for (i1 = 0; i1 < 6; i1++) {
      Jb[i1 + 6 * (static_cast<int>(c_i) - 1)] = 0.0;
      for (a_tmp = 0; a_tmp < 6; a_tmp++) {
        Jb[i1 + 6 * (static_cast<int>(c_i) - 1)] = Jb[i1 + 6 * (static_cast<int>
          (c_i) - 1)] + a[i1 + 6 * a_tmp] * robot->A[(static_cast<int>(c_i) +
          robot->A.size(0) * a_tmp) - 1];
      }

      y[i1] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * i1) - 1] *
        w_hat_idx_0;
    }

    std::memset(&invT[0], 0, 16U * sizeof(double));
    invT[15] = 1.0;

    //  v is nx6 each row represents a twist
    for (i1 = 0; i1 < 7; i1++) {
      sa[i1] = 0.0;
    }

    w_hat_idx_0 = 0.0;
    scale = 3.3121686421112381E-170;
    for (invT_tmp = 0; invT_tmp < 6; invT_tmp++) {
      absxk = std::abs(y[invT_tmp]);
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
    W[3] = -sa[2];
    W[6] = sa[1];
    W[1] = sa[2];
    W[7] = -sa[0];
    W[2] = -sa[1];
    W[5] = sa[0];
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

      for (invT_tmp = 0; invT_tmp < 3; invT_tmp++) {
        b_I[invT_tmp + 3 * invT_tmp] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          b_so_w_hat[invT_tmp + 3 * i1] = (so_w_hat[invT_tmp] * so_w_hat[3 * i1]
            + so_w_hat[invT_tmp + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[invT_tmp
            + 6] * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        so_w_hat[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) +
          w_hat_idx_0 * b_so_w_hat[i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      invT_tmp = i1 << 2;
      invT[invT_tmp] = so_w_hat[3 * i1];
      invT[invT_tmp + 1] = so_w_hat[3 * i1 + 1];
      invT[invT_tmp + 2] = so_w_hat[3 * i1 + 2];
    }

    scale = sa[6] - std::sin(sa[6]);
    absxk = 1.0 - std::cos(sa[6]);
    for (i1 = 0; i1 < 3; i1++) {
      t = 0.0;
      for (a_tmp = 0; a_tmp < 3; a_tmp++) {
        invT_tmp = i1 + 3 * a_tmp;
        t += ((static_cast<double>(b_a[invT_tmp]) * sa[6] + absxk * W[invT_tmp])
              + ((scale * W[i1] * W[3 * a_tmp] + scale * W[i1 + 3] * W[3 * a_tmp
                  + 1]) + scale * W[i1 + 6] * W[3 * a_tmp + 2])) * sa[a_tmp + 3];
      }

      invT[i1 + 12] = t;
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (a_tmp = 0; a_tmp < 4; a_tmp++) {
        invT_tmp = a_tmp << 2;
        b_robot[i1 + invT_tmp] = ((robot->M[i1 + 16 * (static_cast<int>(c_i) - 1)]
          * invT[invT_tmp] + robot->M[(i1 + 16 * (static_cast<int>(c_i) - 1)) +
          4] * invT[invT_tmp + 1]) + robot->M[(i1 + 16 * (static_cast<int>(c_i)
          - 1)) + 8] * invT[invT_tmp + 2]) + robot->M[(i1 + 16 * (static_cast<
          int>(c_i) - 1)) + 12] * invT[invT_tmp + 3];
      }
    }

    for (i1 = 0; i1 < 4; i1++) {
      absxk = b_robot[i1];
      t = b_robot[i1 + 4];
      scale = b_robot[i1 + 8];
      w_hat_idx_0 = b_robot[i1 + 12];
      for (a_tmp = 0; a_tmp < 4; a_tmp++) {
        invT_tmp = a_tmp << 2;
        invT[i1 + invT_tmp] = ((absxk * T[invT_tmp] + t * T[invT_tmp + 1]) +
          scale * T[invT_tmp + 2]) + w_hat_idx_0 * T[invT_tmp + 3];
      }
    }

    std::memcpy(&T[0], &invT[0], 16U * sizeof(double));
  }
}

//
// File trailer for jacobian_matrix.cpp
//
// [EOF]
//
