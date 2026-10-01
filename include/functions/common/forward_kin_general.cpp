//
// File: forward_kin_general.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 23:12:51
//

// Include Files
#include "forward_kin_general.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const struct0_T *robot
//                const coder::array<double, 2U> &q
//                double T[16]
// Return Type  : void
//
void forward_kin_general(const Robot *robot, const coder::array<double, 2U>
  &q, double T[16])
{
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b[16];
  double b_robot[16];
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
  std::memcpy(&T[0], &robot->ME[0], 16U * sizeof(double));
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
    int i1;
    int i2;
    int k;
    c_i = robot->dof + -static_cast<double>(b_i);
    w_hat_idx_0 = q[static_cast<int>(c_i) - 1];
    for (i1 = 0; i1 < 6; i1++) {
      y[i1] = robot->A[(static_cast<int>(c_i) + robot->A.size(0) * i1) - 1] *
        w_hat_idx_0;
    }

    std::memset(&b[0], 0, 16U * sizeof(double));
    b[15] = 1.0;

    //  v is nx6 each row represents a twist
    for (i1 = 0; i1 < 7; i1++) {
      sa[i1] = 0.0;
    }

    w_hat_idx_0 = 0.0;
    scale = 3.3121686421112381E-170;
    for (k = 0; k < 6; k++) {
      absxk = std::abs(y[k]);
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

      for (k = 0; k < 3; k++) {
        b_I[k + 3 * k] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          b_so_w_hat[k + 3 * i1] = (so_w_hat[k] * so_w_hat[3 * i1] + so_w_hat[k
            + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[k + 6] * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        so_w_hat[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) +
          w_hat_idx_0 * b_so_w_hat[i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      k = i1 << 2;
      b[k] = so_w_hat[3 * i1];
      b[k + 1] = so_w_hat[3 * i1 + 1];
      b[k + 2] = so_w_hat[3 * i1 + 2];
    }

    scale = sa[6] - std::sin(sa[6]);
    absxk = 1.0 - std::cos(sa[6]);
    for (i1 = 0; i1 < 3; i1++) {
      t = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        k = i1 + 3 * i2;
        t += ((static_cast<double>(a[k]) * sa[6] + absxk * W[k]) + ((scale *
                W[i1] * W[3 * i2] + scale * W[i1 + 3] * W[3 * i2 + 1]) + scale *
               W[i1 + 6] * W[3 * i2 + 2])) * sa[i2 + 3];
      }

      b[i1 + 12] = t;
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        k = i2 << 2;
        b_robot[i1 + k] = ((robot->M[i1 + 16 * (static_cast<int>(c_i) - 1)] *
                            b[k] + robot->M[(i1 + 16 * (static_cast<int>(c_i) -
          1)) + 4] * b[k + 1]) + robot->M[(i1 + 16 * (static_cast<int>(c_i) - 1))
                           + 8] * b[k + 2]) + robot->M[(i1 + 16 * (static_cast<
          int>(c_i) - 1)) + 12] * b[k + 3];
      }
    }

    for (i1 = 0; i1 < 4; i1++) {
      absxk = b_robot[i1];
      t = b_robot[i1 + 4];
      w_hat_idx_0 = b_robot[i1 + 8];
      scale = b_robot[i1 + 12];
      for (i2 = 0; i2 < 4; i2++) {
        k = i2 << 2;
        b[i1 + k] = ((absxk * T[k] + t * T[k + 1]) + w_hat_idx_0 * T[k + 2]) +
          scale * T[k + 3];
      }
    }

    std::memcpy(&T[0], &b[0], 16U * sizeof(double));
  }
}

//
// File trailer for forward_kin_general.cpp
//
// [EOF]
//
