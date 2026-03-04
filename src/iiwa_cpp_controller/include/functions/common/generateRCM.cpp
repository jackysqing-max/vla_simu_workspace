//
// File: generateRCM.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 30-Apr-2022 18:23:06
//

// Include Files
#include "generateRCM.h"
#include "exp_twist.h"

// Function Definitions
//
// d in m, a b c in rad, p_rcm in m
// Arguments    : double d
//                double a
//                double b
//                double c
//                const double p_rcm[3]
//                double T[16]
// Return Type  : void
//
void generateRCM(double d, double a, double b, double c, const double p_rcm[3],
                 double T[16])
{
  double dv[16];
  double dv1[16];
  double dv2[16];
  double dv3[16];
  double dv4[16];
  double b_a[6];
  double c_a[6];
  double d_a[6];
  double e_a[6];
  int i;

  //  sm (screw motion): [q (point), s (u-axis), h (pitch), theta]
  b_a[0] = 0.0;
  b_a[1] = 0.0;
  b_a[2] = 0.0;

  //  p_rcm = p_rcm - d * z_rcm;
  //  sm (screw motion): [q (point), s (u-axis), h (pitch), theta]
  b_a[3] = 0.0;
  b_a[4] = 0.0;
  b_a[5] = 1.0;
  c_a[0] = 0.0;
  c_a[1] = 0.0;
  c_a[2] = 1.0;

  //  sm (screw motion): [q (point), s (u-axis), h (pitch), theta]
  c_a[3] = -0.0 * p_rcm[2] - (-p_rcm[1]);
  c_a[4] = -p_rcm[0] - -0.0 * p_rcm[2];
  c_a[5] = -0.0 * p_rcm[1] - -0.0 * p_rcm[0];
  d_a[0] = 1.0;
  d_a[1] = 0.0;
  d_a[2] = 0.0;

  //  sm (screw motion): [q (point), s (u-axis), h (pitch), theta]
  d_a[3] = -0.0 * p_rcm[2] - -0.0 * p_rcm[1];
  d_a[4] = -0.0 * p_rcm[0] - (-p_rcm[2]);
  d_a[5] = -p_rcm[1] - -0.0 * p_rcm[0];
  e_a[0] = 0.0;
  e_a[1] = 1.0;
  e_a[2] = 0.0;
  e_a[3] = -p_rcm[2] - -0.0 * p_rcm[1];
  e_a[4] = -0.0 * p_rcm[0] - -0.0 * p_rcm[2];
  e_a[5] = -0.0 * p_rcm[1] - (-p_rcm[0]);
  for (i = 0; i < 6; i++) {
    d_a[i] *= b;
    e_a[i] *= -c;
    c_a[i] *= -a;
    b_a[i] *= d;
  }

  exp_twist(d_a, dv);
  exp_twist(e_a, dv1);
  exp_twist(c_a, dv2);
  exp_twist(b_a, dv3);
  for (i = 0; i < 4; i++) {
    double b_d;
    double d1;
    double d2;
    double d3;
    int i1;
    int i2;
    b_d = dv[i];
    d1 = dv[i + 4];
    d2 = dv[i + 8];
    d3 = dv[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      dv4[i + i2] = ((b_d * dv1[i2] + d1 * dv1[i2 + 1]) + d2 * dv1[i2 + 2]) + d3
        * dv1[i2 + 3];
    }

    b_d = dv4[i];
    d1 = dv4[i + 4];
    d2 = dv4[i + 8];
    d3 = dv4[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      dv[i + i2] = ((b_d * dv2[i2] + d1 * dv2[i2 + 1]) + d2 * dv2[i2 + 2]) + d3 *
        dv2[i2 + 3];
    }

    b_d = dv[i];
    d1 = dv[i + 4];
    d2 = dv[i + 8];
    d3 = dv[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      T[i + i2] = ((b_d * dv3[i2] + d1 * dv3[i2 + 1]) + d2 * dv3[i2 + 2]) + d3 *
        dv3[i2 + 3];
    }
  }
}

//
// File trailer for generateRCM.cpp
//
// [EOF]
//
