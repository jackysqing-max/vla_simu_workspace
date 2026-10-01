//
// File: exp_twist.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 19-Apr-2022 00:27:47
//

// Include Files
#include "exp_twist.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double v[6]
//                double tform[16]
// Return Type  : void
//
void exp_twist(const double v[6], double tform[16])
{
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double W[9];
  double b_so_w_hat[9];
  double so_w_hat[9];
  double sa[7];
  double absxk;
  double scale;
  double t;
  double theta;
  double w_hat_idx_1;
  int i;
  int k;
  signed char b_I[9];
  std::memset(&tform[0], 0, 16U * sizeof(double));
  tform[15] = 1.0;

  //  v is nx6 each row represents a twist
  for (i = 0; i < 7; i++) {
    sa[i] = 0.0;
  }

  w_hat_idx_1 = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 6; k++) {
    absxk = std::abs(v[k]);
    if (absxk > scale) {
      t = scale / absxk;
      w_hat_idx_1 = w_hat_idx_1 * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      w_hat_idx_1 += t * t;
    }
  }

  w_hat_idx_1 = scale * std::sqrt(w_hat_idx_1);
  if (w_hat_idx_1 <= 2.2204460492503131E-16) {
    sa[5] = 1.0;
  } else {
    scale = 3.3121686421112381E-170;
    absxk = std::abs(v[0]);
    if (absxk > 3.3121686421112381E-170) {
      theta = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      theta = t * t;
    }

    absxk = std::abs(v[1]);
    if (absxk > scale) {
      t = scale / absxk;
      theta = theta * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      theta += t * t;
    }

    absxk = std::abs(v[2]);
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
      absxk = std::abs(v[3]);
      if (absxk > 3.3121686421112381E-170) {
        w_hat_idx_1 = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        w_hat_idx_1 = t * t;
      }

      absxk = std::abs(v[4]);
      if (absxk > scale) {
        t = scale / absxk;
        w_hat_idx_1 = w_hat_idx_1 * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        w_hat_idx_1 += t * t;
      }

      absxk = std::abs(v[5]);
      if (absxk > scale) {
        t = scale / absxk;
        w_hat_idx_1 = w_hat_idx_1 * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        w_hat_idx_1 += t * t;
      }

      w_hat_idx_1 = scale * std::sqrt(w_hat_idx_1);
      sa[6] = w_hat_idx_1;
      sa[3] = v[3] / w_hat_idx_1;
      sa[4] = v[4] / w_hat_idx_1;
      sa[5] = v[5] / w_hat_idx_1;
    } else {
      sa[6] = theta;
      for (i = 0; i < 6; i++) {
        sa[i] = v[i] / theta;
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
  absxk = std::abs(v[0]);
  if (absxk > 3.3121686421112381E-170) {
    theta = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    theta = t * t;
  }

  absxk = std::abs(v[1]);
  if (absxk > scale) {
    t = scale / absxk;
    theta = theta * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    theta += t * t;
  }

  absxk = std::abs(v[2]);
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
    scale = v[0] / theta;
    w_hat_idx_1 = v[1] / theta;
    absxk = v[2] / theta;

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    so_w_hat[0] = 0.0;
    so_w_hat[3] = -absxk;
    so_w_hat[6] = w_hat_idx_1;
    so_w_hat[1] = absxk;
    so_w_hat[4] = 0.0;
    so_w_hat[7] = -scale;
    so_w_hat[2] = -w_hat_idx_1;
    so_w_hat[5] = scale;
    so_w_hat[8] = 0.0;
    absxk = std::sin(theta);
    w_hat_idx_1 = 1.0 - std::cos(theta);
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    for (k = 0; k < 3; k++) {
      b_I[k + 3 * k] = 1;
      for (i = 0; i < 3; i++) {
        b_so_w_hat[k + 3 * i] = (so_w_hat[k] * so_w_hat[3 * i] + so_w_hat[k + 3]
          * so_w_hat[3 * i + 1]) + so_w_hat[k + 6] * so_w_hat[3 * i + 2];
      }
    }

    for (i = 0; i < 9; i++) {
      so_w_hat[i] = (static_cast<double>(b_I[i]) + absxk * so_w_hat[i]) +
        w_hat_idx_1 * b_so_w_hat[i];
    }
  }

  for (i = 0; i < 3; i++) {
    k = i << 2;
    tform[k] = so_w_hat[3 * i];
    tform[k + 1] = so_w_hat[3 * i + 1];
    tform[k + 2] = so_w_hat[3 * i + 2];
  }

  absxk = sa[6] - std::sin(sa[6]);
  w_hat_idx_1 = 1.0 - std::cos(sa[6]);
  for (i = 0; i < 3; i++) {
    scale = 0.0;
    for (int i1 = 0; i1 < 3; i1++) {
      k = i + 3 * i1;
      scale += ((static_cast<double>(a[k]) * sa[6] + w_hat_idx_1 * W[k]) +
                ((absxk * W[i] * W[3 * i1] + absxk * W[i + 3] * W[3 * i1 + 1]) +
                 absxk * W[i + 6] * W[3 * i1 + 2])) * sa[i1 + 3];
    }

    tform[i + 12] = scale;
  }
}

//
// File trailer for exp_twist.cpp
//
// [EOF]
//
