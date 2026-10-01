//
// File: m_c_g_matrix.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 09-Mar-2024 19:04:26
//

// Include Files
#include "m_c_g_matrix.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace coder {
    namespace internal {
        namespace blas {
            static void mtimes(const ::coder::array<double, 2U>& A, const double B[36],
                ::coder::array<double, 2U>& C)
            {
                int m;
                m = A.size(1);
                C.set_size(A.size(1), 6);
                for (int j{ 0 }; j < 6; j++) {
                    int boffset;
                    int coffset;
                    coffset = j * m;
                    boffset = j * 6;
                    for (int i{ 0 }; i < m; i++) {
                        double s;
                        int aoffset;
                        aoffset = i * 6;
                        s = 0.0;
                        for (int k{ 0 }; k < 6; k++) {
                            s += A[aoffset + k] * B[boffset + k];
                        }
                        C[coffset + i] = s;
                    }
                }
            }

            //
            // Arguments    : const ::coder::array<double, 2U> &A
            //                const ::coder::array<double, 2U> &B
            //                ::coder::array<double, 2U> &C
            // Return Type  : void
            //
           static void mtimes(const ::coder::array<double, 2U>& A,
                const ::coder::array<double, 2U>& B, ::coder::array<double, 2U>& C)
            {
                int m;
                int n;
                m = A.size(0);
                n = B.size(1);
                C.set_size(A.size(0), B.size(1));
                for (int j{ 0 }; j < n; j++) {
                    int boffset;
                    int coffset;
                    coffset = j * m;
                    boffset = j * 6;
                    for (int i{ 0 }; i < m; i++) {
                        double s;
                        s = 0.0;
                        for (int k{ 0 }; k < 6; k++) {
                            s += A[k * A.size(0) + i] * B[boffset + k];
                        }
                        C[coffset + i] = s;
                    }
                }
            }

        } // namespace blas
    } // namespace internal
} // namespace coder

// Function Declarations
static void binary_expand_op(coder::array<double, 2U> &in1,
                             const coder::array<double, 2U> &in2,
                             const coder::array<double, 2U> &in3);

static void binary_expand_op(coder::array<double, 3U> &in1, int in2,
                             const coder::array<double, 2U> &in3,
                             const coder::array<double, 2U> &in4);

static void plus(coder::array<double, 2U> &in1,
                 const coder::array<double, 2U> &in2);

// Function Definitions
//
// Arguments    : coder::array<double, 2U> &in1
//                const coder::array<double, 2U> &in2
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
static void binary_expand_op(coder::array<double, 2U> &in1,
                             const coder::array<double, 2U> &in2,
                             const coder::array<double, 2U> &in3)
{
  coder::array<double, 2U> b_in1;
  int aux_0_1;
  int aux_1_1;
  int aux_2_1;
  int b_loop_ub;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  int stride_2_0;
  int stride_2_1;
  if (in3.size(0) == 1) {
    if (in2.size(0) == 1) {
      loop_ub = in1.size(0);
    } else {
      loop_ub = in2.size(0);
    }
  } else {
    loop_ub = in3.size(0);
  }
  if (in3.size(1) == 1) {
    if (in2.size(1) == 1) {
      b_loop_ub = in1.size(1);
    } else {
      b_loop_ub = in2.size(1);
    }
  } else {
    b_loop_ub = in3.size(1);
  }
  b_in1.set_size(loop_ub, b_loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_0 = (in2.size(0) != 1);
  stride_1_1 = (in2.size(1) != 1);
  stride_2_0 = (in3.size(0) != 1);
  stride_2_1 = (in3.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  aux_2_1 = 0;
  for (int i{0}; i < b_loop_ub; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in1[i1 + b_in1.size(0) * i] =
          (in1[i1 * stride_0_0 + in1.size(0) * aux_0_1] +
           in2[i1 * stride_1_0 + in2.size(0) * aux_1_1]) +
          in3[i1 * stride_2_0 + in3.size(0) * aux_2_1];
    }
    aux_2_1 += stride_2_1;
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1.set_size(b_in1.size(0), b_in1.size(1));
  loop_ub = b_in1.size(1);
  for (int i{0}; i < loop_ub; i++) {
    b_loop_ub = b_in1.size(0);
    for (int i1{0}; i1 < b_loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = b_in1[i1 + b_in1.size(0) * i];
    }
  }
}

//
// Arguments    : coder::array<double, 3U> &in1
//                int in2
//                const coder::array<double, 2U> &in3
//                const coder::array<double, 2U> &in4
// Return Type  : void
//
static void binary_expand_op(coder::array<double, 3U> &in1, int in2,
                             const coder::array<double, 2U> &in3,
                             const coder::array<double, 2U> &in4)
{
  coder::array<double, 2U> b_in1;
  int aux_0_1;
  int aux_1_1;
  int aux_2_1;
  int b_loop_ub;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  int stride_2_0;
  int stride_2_1;
  if (in4.size(0) == 1) {
    if (in3.size(0) == 1) {
      loop_ub = in1.size(0);
    } else {
      loop_ub = in3.size(0);
    }
  } else {
    loop_ub = in4.size(0);
  }
  if (in4.size(1) == 1) {
    if (in3.size(1) == 1) {
      b_loop_ub = in1.size(1);
    } else {
      b_loop_ub = in3.size(1);
    }
  } else {
    b_loop_ub = in4.size(1);
  }
  b_in1.set_size(loop_ub, b_loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_0 = (in3.size(0) != 1);
  stride_1_1 = (in3.size(1) != 1);
  stride_2_0 = (in4.size(0) != 1);
  stride_2_1 = (in4.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  aux_2_1 = 0;
  for (int i{0}; i < b_loop_ub; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in1[i1 + b_in1.size(0) * i] =
          (in1[(i1 * stride_0_0 + in1.size(0) * aux_0_1) +
               in1.size(0) * in1.size(1) * in2] +
           in3[i1 * stride_1_0 + in3.size(0) * aux_1_1]) +
          in4[i1 * stride_2_0 + in4.size(0) * aux_2_1];
    }
    aux_2_1 += stride_2_1;
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  loop_ub = b_in1.size(1);
  for (int i{0}; i < loop_ub; i++) {
    b_loop_ub = b_in1.size(0);
    for (int i1{0}; i1 < b_loop_ub; i1++) {
      in1[(i1 + in1.size(0) * i) + in1.size(0) * in1.size(1) * in2] =
          b_in1[i1 + b_in1.size(0) * i];
    }
  }
}

//
// Arguments    : coder::array<double, 2U> &in1
//                const coder::array<double, 2U> &in2
// Return Type  : void
//
static void plus(coder::array<double, 2U> &in1,
                 const coder::array<double, 2U> &in2)
{
  coder::array<double, 2U> b_in1;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  if (in2.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in2.size(0);
  }
  if (in2.size(1) == 1) {
    b_loop_ub = in1.size(1);
  } else {
    b_loop_ub = in2.size(1);
  }
  b_in1.set_size(loop_ub, b_loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_0 = (in2.size(0) != 1);
  stride_1_1 = (in2.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (int i{0}; i < b_loop_ub; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in1[i1 + b_in1.size(0) * i] =
          in1[i1 * stride_0_0 + in1.size(0) * aux_0_1] +
          in2[i1 * stride_1_0 + in2.size(0) * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1.set_size(b_in1.size(0), b_in1.size(1));
  loop_ub = b_in1.size(1);
  for (int i{0}; i < loop_ub; i++) {
    b_loop_ub = b_in1.size(0);
    for (int i1{0}; i1 < b_loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = b_in1[i1 + b_in1.size(0) * i];
    }
  }
}

//
// Jb, dJb expressed in TCP
//
// Arguments    : const struct0_T *robot
//                const coder::array<double, 1U> &q
//                const coder::array<double, 1U> &qd
//                coder::array<double, 2U> &Mq
//                coder::array<double, 2U> &C
//                coder::array<double, 1U> &g
//                coder::array<double, 2U> &Jb
//                coder::array<double, 2U> &dJb
//                coder::array<double, 2U> &dMq
//                double dTcp[16]
//                double Tcp[16]
// Return Type  : void
//
void m_c_g_matrix(const Robot *robot, const coder::array<double, 1U> &q,
                  const coder::array<double, 1U> &qd,
                  coder::array<double, 2U> &Mq, coder::array<double, 2U> &C,
                  coder::array<double, 1U> &g, coder::array<double, 2U> &Jb,
                  coder::array<double, 2U> &dJb, coder::array<double, 2U> &dMq,
                  double dTcp[16], double Tcp[16])
{
  static const signed char b[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  coder::array<double, 4U> pdJ;
  coder::array<double, 3U> G;
  coder::array<double, 3U> J;
  coder::array<double, 3U> dJ;
  coder::array<double, 3U> pdMq;
  coder::array<double, 3U> pdT;
  coder::array<double, 2U> P;
  coder::array<double, 2U> b_J;
  coder::array<double, 2U> b_pdMq;
  coder::array<double, 2U> r;
  coder::array<double, 2U> r1;
  double adTb[36];
  double ME[16];
  double T[16];
  double a[16];
  double b_T[16];
  double so_w_hat[9];
  double n;
  int b_loop_ub_tmp;
  int c_j;
  int c_loop_ub_tmp;
  int d_i;
  int d_j;
  int i;
  int i1;
  int i2;
  int loop_ub;
  int loop_ub_tmp;
  int loop_ub_tmp_tmp;
  signed char Tcp_tmp[16];
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      ME[i + i2] = ((robot->ME[i] * robot->TCP[i2] +
                     robot->ME[i + 4] * robot->TCP[i2 + 1]) +
                    robot->ME[i + 8] * robot->TCP[i2 + 2]) +
                   robot->ME[i + 12] * robot->TCP[i2 + 3];
    }
  }
  n = robot->dof;
  loop_ub_tmp = static_cast<int>(robot->dof);
  J.set_size(6, loop_ub_tmp, loop_ub_tmp);
  loop_ub_tmp_tmp = 6 * loop_ub_tmp;
  b_loop_ub_tmp = loop_ub_tmp_tmp * loop_ub_tmp;
  dJ.set_size(6, loop_ub_tmp, loop_ub_tmp);
  for (i = 0; i < b_loop_ub_tmp; i++) {
    J[i] = 0.0;
    dJ[i] = 0.0;
  }
  Mq.set_size(loop_ub_tmp, loop_ub_tmp);
  c_loop_ub_tmp = loop_ub_tmp * loop_ub_tmp;
  dMq.set_size(loop_ub_tmp, loop_ub_tmp);
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Mq[i] = 0.0;
    dMq[i] = 0.0;
  }
  pdJ.set_size(6, loop_ub_tmp, loop_ub_tmp, loop_ub_tmp);
  b_loop_ub_tmp *= loop_ub_tmp;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    pdJ[i] = 0.0;
  }
  // 关于关节角的偏导数
  pdMq.set_size(loop_ub_tmp, loop_ub_tmp, loop_ub_tmp);
  b_loop_ub_tmp = c_loop_ub_tmp * loop_ub_tmp;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    pdMq[i] = 0.0;
  }
  // 关于关节角的偏导数
  C.set_size(loop_ub_tmp, loop_ub_tmp);
  for (i = 0; i < c_loop_ub_tmp; i++) {
    C[i] = 0.0;
  }
  G.set_size(6, 6, loop_ub_tmp);
  b_loop_ub_tmp = 36 * loop_ub_tmp;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    G[i] = 0.0;
  }
  P.set_size(loop_ub_tmp, loop_ub_tmp);
  for (i = 0; i < c_loop_ub_tmp; i++) {
    P[i] = 0.0;
  }
  g.set_size(loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp; i++) {
    g[i] = 0.0;
  }
  Jb.set_size(6, loop_ub_tmp);
  dJb.set_size(6, loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp_tmp; i++) {
    Jb[i] = 0.0;
    dJb[i] = 0.0;
  }
  for (i = 0; i < 16; i++) {
    Tcp_tmp[i] = 0;
  }
  Tcp_tmp[0] = 1;
  Tcp_tmp[5] = 1;
  Tcp_tmp[10] = 1;
  Tcp_tmp[15] = 1;
  for (i = 0; i < 16; i++) {
    Tcp[i] = Tcp_tmp[i];
    dTcp[i] = 0.0;
  }
  i = static_cast<int>(-((-1.0 - robot->dof) + 1.0));
  if (i - 1 >= 0) {
    loop_ub = loop_ub_tmp << 4;
  }
  for (int b_i{0}; b_i < i; b_i++) {
    double c_robot[16];
    double dT[16];
    double b_skcom[9];
    double skcom[9];
    double absxk;
    double c_i;
    double scale;
    double t;
    double w_hat_idx_0;
    int i3;
    c_i = n - static_cast<double>(b_i);
    //  return the spatial matrix G with respect to the frame of I
    //  I is not necessary defined in center mass
    //  com is a row vector representing the center mass in I's frame
    std::memset(&adTb[0], 0, 36U * sizeof(double));
    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    skcom[0] = 0.0;
    scale = robot->com[(static_cast<int>(c_i) + robot->com.size(0) * 2) - 1];
    skcom[3] = -scale;
    absxk = robot->com[(static_cast<int>(c_i) + robot->com.size(0)) - 1];
    skcom[6] = absxk;
    skcom[1] = scale;
    skcom[4] = 0.0;
    w_hat_idx_0 = robot->com[static_cast<int>(c_i) - 1];
    skcom[7] = -w_hat_idx_0;
    skcom[2] = -absxk;
    skcom[5] = w_hat_idx_0;
    skcom[8] = 0.0;
    w_hat_idx_0 = (w_hat_idx_0 * w_hat_idx_0 + absxk * absxk) + scale * scale;
    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_loop_ub_tmp = i2 + 3 * i1;
        so_w_hat[b_loop_ub_tmp] =
            w_hat_idx_0 * static_cast<double>(b[b_loop_ub_tmp]) -
            robot->com[(static_cast<int>(c_i) + robot->com.size(0) * i2) - 1] *
                robot->com[(static_cast<int>(c_i) + robot->com.size(0) * i1) -
                           1];
        b_skcom[i1 + 3 * i2] =
            (skcom[i1] * skcom[3 * i2] + skcom[i1 + 3] * skcom[3 * i2 + 1]) +
            skcom[i1 + 6] * skcom[3 * i2 + 2];
      }
    }
    for (i1 = 0; i1 < 3; i1++) {
      w_hat_idx_0 = robot->mass[static_cast<int>(c_i) - 1];
      adTb[6 * i1] = robot->inertia[3 * i1 + 9 * (static_cast<int>(c_i) - 1)] -
                     w_hat_idx_0 * (so_w_hat[3 * i1] + b_skcom[3 * i1]);
      t = w_hat_idx_0 * skcom[3 * i1];
      c_loop_ub_tmp = 6 * (i1 + 3);
      adTb[c_loop_ub_tmp] = t;
      so_w_hat[3 * i1] = -t;
      adTb[6 * i1 + 3] = -t;
      adTb[c_loop_ub_tmp + 3] = w_hat_idx_0 * static_cast<double>(b[3 * i1]);
      loop_ub_tmp_tmp = 3 * i1 + 1;
      adTb[6 * i1 + 1] =
          robot->inertia[(3 * i1 + 9 * (static_cast<int>(c_i) - 1)) + 1] -
          w_hat_idx_0 * (so_w_hat[loop_ub_tmp_tmp] + b_skcom[loop_ub_tmp_tmp]);
      t = w_hat_idx_0 * skcom[loop_ub_tmp_tmp];
      adTb[c_loop_ub_tmp + 1] = t;
      so_w_hat[loop_ub_tmp_tmp] = -t;
      adTb[6 * i1 + 4] = -t;
      adTb[c_loop_ub_tmp + 4] =
          w_hat_idx_0 * static_cast<double>(b[loop_ub_tmp_tmp]);
      loop_ub_tmp_tmp = 3 * i1 + 2;
      adTb[6 * i1 + 2] =
          robot->inertia[(3 * i1 + 9 * (static_cast<int>(c_i) - 1)) + 2] -
          w_hat_idx_0 * (so_w_hat[loop_ub_tmp_tmp] + b_skcom[loop_ub_tmp_tmp]);
      t = w_hat_idx_0 * skcom[loop_ub_tmp_tmp];
      adTb[c_loop_ub_tmp + 2] = t;
      so_w_hat[loop_ub_tmp_tmp] = -t;
      adTb[6 * i1 + 5] = -t;
      adTb[c_loop_ub_tmp + 5] =
          w_hat_idx_0 * static_cast<double>(b[loop_ub_tmp_tmp]);
    }
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        G[(i2 + 6 * i1) + 36 * (static_cast<int>(c_i) - 1)] = adTb[i2 + 6 * i1];
      }
    }
    //  连杆i的坐标系的逆
    for (i1 = 0; i1 < 16; i1++) {
      T[i1] = Tcp_tmp[i1];
      dT[i1] = 0.0;
    }
    //  T对时间的导数
    pdT.set_size(4, 4, loop_ub_tmp);
    for (i1 = 0; i1 < loop_ub; i1++) {
      pdT[i1] = 0.0;
    }
    //  T对关节变量的偏导数
    i1 = static_cast<int>(-((-1.0 - c_i) + 1.0));
    for (int j{0}; j < i1; j++) {
      double tform[16];
      double dv[9];
      double sa[7];
      double y[6];
      double b_j;
      double theta;
      b_j = c_i - static_cast<double>(j);
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      skcom[0] = 0.0;
      skcom[3] = -T[14];
      skcom[6] = T[13];
      skcom[1] = T[14];
      skcom[4] = 0.0;
      skcom[7] = -T[12];
      skcom[2] = -T[13];
      skcom[5] = T[12];
      skcom[8] = 0.0;
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      for (i2 = 0; i2 < 3; i2++) {
        t = skcom[i2];
        absxk = skcom[i2 + 3];
        scale = skcom[i2 + 6];
        for (i3 = 0; i3 < 3; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          b_skcom[i2 + 3 * i3] =
              (t * T[loop_ub_tmp_tmp] + absxk * T[loop_ub_tmp_tmp + 1]) +
              scale * T[loop_ub_tmp_tmp + 2];
          adTb[i3 + 6 * i2] = T[i3 + (i2 << 2)];
          adTb[i3 + 6 * (i2 + 3)] = 0.0;
        }
      }
      for (i2 = 0; i2 < 3; i2++) {
        adTb[6 * i2 + 3] = b_skcom[3 * i2];
        c_loop_ub_tmp = i2 << 2;
        loop_ub_tmp_tmp = 6 * (i2 + 3);
        adTb[loop_ub_tmp_tmp + 3] = T[c_loop_ub_tmp];
        adTb[6 * i2 + 4] = b_skcom[3 * i2 + 1];
        adTb[loop_ub_tmp_tmp + 4] = T[c_loop_ub_tmp + 1];
        adTb[6 * i2 + 5] = b_skcom[3 * i2 + 2];
        adTb[loop_ub_tmp_tmp + 5] = T[c_loop_ub_tmp + 2];
      }
      for (i2 = 0; i2 < 6; i2++) {
        J[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
          6 * J.size(1) * (static_cast<int>(c_i) - 1)] = 0.0;
        for (i3 = 0; i3 < 6; i3++) {
          J[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
            6 * J.size(1) * (static_cast<int>(c_i) - 1)] =
              J[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
                6 * J.size(1) * (static_cast<int>(c_i) - 1)] +
              adTb[i2 + 6 * i3] *
                  robot->A[(static_cast<int>(b_j) + robot->A.size(0) * i3) - 1];
        }
      }
      b_skcom[0] = 0.0;
      b_skcom[3] = -dT[14];
      b_skcom[6] = dT[13];
      b_skcom[1] = dT[14];
      b_skcom[4] = 0.0;
      b_skcom[7] = -dT[12];
      b_skcom[2] = -dT[13];
      b_skcom[5] = dT[12];
      b_skcom[8] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        t = b_skcom[i2];
        absxk = b_skcom[i2 + 3];
        scale = b_skcom[i2 + 6];
        for (i3 = 0; i3 < 3; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          dv[i2 + 3 * i3] =
              (t * T[loop_ub_tmp_tmp] + absxk * T[loop_ub_tmp_tmp + 1]) +
              scale * T[loop_ub_tmp_tmp + 2];
        }
        t = skcom[i2];
        absxk = skcom[i2 + 3];
        scale = skcom[i2 + 6];
        for (i3 = 0; i3 < 3; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          b_skcom[i2 + 3 * i3] =
              (t * dT[loop_ub_tmp_tmp] + absxk * dT[loop_ub_tmp_tmp + 1]) +
              scale * dT[loop_ub_tmp_tmp + 2];
          adTb[i3 + 6 * i2] = dT[i3 + (i2 << 2)];
          adTb[i3 + 6 * (i2 + 3)] = 0.0;
        }
      }
      for (i2 = 0; i2 < 3; i2++) {
        adTb[6 * i2 + 3] = dv[3 * i2] + b_skcom[3 * i2];
        c_loop_ub_tmp = i2 << 2;
        loop_ub_tmp_tmp = 6 * (i2 + 3);
        adTb[loop_ub_tmp_tmp + 3] = dT[c_loop_ub_tmp];
        b_loop_ub_tmp = 3 * i2 + 1;
        adTb[6 * i2 + 4] = dv[b_loop_ub_tmp] + b_skcom[b_loop_ub_tmp];
        adTb[loop_ub_tmp_tmp + 4] = dT[c_loop_ub_tmp + 1];
        b_loop_ub_tmp = 3 * i2 + 2;
        adTb[6 * i2 + 5] = dv[b_loop_ub_tmp] + b_skcom[b_loop_ub_tmp];
        adTb[loop_ub_tmp_tmp + 5] = dT[c_loop_ub_tmp + 2];
      }
      for (i2 = 0; i2 < 6; i2++) {
        dJ[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
           6 * dJ.size(1) * (static_cast<int>(c_i) - 1)] = 0.0;
        for (i3 = 0; i3 < 6; i3++) {
          dJ[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
             6 * dJ.size(1) * (static_cast<int>(c_i) - 1)] =
              dJ[(i2 + 6 * (static_cast<int>(b_j) - 1)) +
                 6 * dJ.size(1) * (static_cast<int>(c_i) - 1)] +
              adTb[i2 + 6 * i3] *
                  robot->A[(static_cast<int>(b_j) + robot->A.size(0) * i3) - 1];
        }
        y[i2] = -robot->A[(static_cast<int>(b_j) + robot->A.size(0) * i2) - 1] *
                q[static_cast<int>(b_j) - 1];
      }
      std::memset(&a[0], 0, 16U * sizeof(double));
      a[15] = 1.0;
      //  v is nx6 each row represents a twist
      for (i2 = 0; i2 < 7; i2++) {
        sa[i2] = 0.0;
      }
      w_hat_idx_0 = 0.0;
      scale = 3.3121686421112381E-170;
      for (int k{0}; k < 6; k++) {
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
          for (i2 = 0; i2 < 6; i2++) {
            sa[i2] = y[i2] / theta;
          }
        }
      }
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      skcom[0] = 0.0;
      skcom[3] = -sa[2];
      skcom[6] = sa[1];
      skcom[1] = sa[2];
      skcom[4] = 0.0;
      skcom[7] = -sa[0];
      skcom[2] = -sa[1];
      skcom[5] = sa[0];
      skcom[8] = 0.0;
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
        signed char b_I[9];
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
        for (i2 = 0; i2 < 9; i2++) {
          b_I[i2] = 0;
        }
        for (int k{0}; k < 3; k++) {
          b_I[k + 3 * k] = 1;
          for (i2 = 0; i2 < 3; i2++) {
            b_skcom[k + 3 * i2] = (so_w_hat[k] * so_w_hat[3 * i2] +
                                   so_w_hat[k + 3] * so_w_hat[3 * i2 + 1]) +
                                  so_w_hat[k + 6] * so_w_hat[3 * i2 + 2];
          }
        }
        for (i2 = 0; i2 < 9; i2++) {
          so_w_hat[i2] = (static_cast<double>(b_I[i2]) + scale * so_w_hat[i2]) +
                         w_hat_idx_0 * b_skcom[i2];
        }
      }
      for (i2 = 0; i2 < 3; i2++) {
        b_loop_ub_tmp = i2 << 2;
        a[b_loop_ub_tmp] = so_w_hat[3 * i2];
        a[b_loop_ub_tmp + 1] = so_w_hat[3 * i2 + 1];
        a[b_loop_ub_tmp + 2] = so_w_hat[3 * i2 + 2];
      }
      scale = sa[6] - std::sin(sa[6]);
      t = 1.0 - std::cos(sa[6]);
      for (i2 = 0; i2 < 3; i2++) {
        absxk = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          b_loop_ub_tmp = i2 + 3 * i3;
          absxk += ((static_cast<double>(b[b_loop_ub_tmp]) * sa[6] +
                     t * skcom[b_loop_ub_tmp]) +
                    ((scale * skcom[i2] * skcom[3 * i3] +
                      scale * skcom[i2 + 3] * skcom[3 * i3 + 1]) +
                     scale * skcom[i2 + 6] * skcom[3 * i3 + 2])) *
                   sa[i3 + 3];
        }
        a[i2 + 12] = absxk;
      }
      //  T: 4x4xn
      for (i2 = 0; i2 < 3; i2++) {
        so_w_hat[3 * i2] = -robot->M[i2 + 16 * (static_cast<int>(b_j) - 1)];
        so_w_hat[3 * i2 + 1] =
            -robot->M[(i2 + 16 * (static_cast<int>(b_j) - 1)) + 4];
        so_w_hat[3 * i2 + 2] =
            -robot->M[(i2 + 16 * (static_cast<int>(b_j) - 1)) + 8];
      }
      for (i2 = 0; i2 < 3; i2++) {
        b_loop_ub_tmp = i2 << 2;
        c_robot[b_loop_ub_tmp] =
            robot->M[i2 + 16 * (static_cast<int>(b_j) - 1)];
        c_robot[b_loop_ub_tmp + 1] =
            robot->M[(i2 + 16 * (static_cast<int>(b_j) - 1)) + 4];
        c_robot[b_loop_ub_tmp + 2] =
            robot->M[(i2 + 16 * (static_cast<int>(b_j) - 1)) + 8];
        c_robot[i2 + 12] =
            (so_w_hat[i2] * robot->M[16 * (static_cast<int>(b_j) - 1) + 12] +
             so_w_hat[i2 + 3] *
                 robot->M[16 * (static_cast<int>(b_j) - 1) + 13]) +
            so_w_hat[i2 + 6] * robot->M[16 * (static_cast<int>(b_j) - 1) + 14];
      }
      c_robot[3] = 0.0;
      c_robot[7] = 0.0;
      c_robot[11] = 0.0;
      c_robot[15] = 1.0;
      for (i2 = 0; i2 < 4; i2++) {
        t = a[i2];
        absxk = a[i2 + 4];
        scale = a[i2 + 8];
        w_hat_idx_0 = a[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          tform[i2 + loop_ub_tmp_tmp] =
              ((t * c_robot[loop_ub_tmp_tmp] +
                absxk * c_robot[loop_ub_tmp_tmp + 1]) +
               scale * c_robot[loop_ub_tmp_tmp + 2]) +
              w_hat_idx_0 * c_robot[loop_ub_tmp_tmp + 3];
        }
      }
      //  tform 对qj的偏导数
      for (i2 = 0; i2 < 6; i2++) {
        y[i2] = -robot->A[(static_cast<int>(b_j) + robot->A.size(0) * i2) - 1];
      }
      //  v -> [v]
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      //  雅克比矩阵对关节变量的偏导数
      c_robot[0] = 0.0;
      c_robot[4] = -y[2];
      c_robot[8] = y[1];
      c_robot[1] = y[2];
      c_robot[5] = 0.0;
      c_robot[9] = -y[0];
      c_robot[2] = -y[1];
      c_robot[6] = y[0];
      c_robot[10] = 0.0;
      c_robot[12] = y[3];
      c_robot[13] = y[4];
      c_robot[14] = y[5];
      c_robot[3] = 0.0;
      c_robot[7] = 0.0;
      c_robot[11] = 0.0;
      c_robot[15] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        t = c_robot[i2];
        absxk = c_robot[i2 + 4];
        scale = c_robot[i2 + 8];
        w_hat_idx_0 = c_robot[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          b_T[i2 + loop_ub_tmp_tmp] = ((t * tform[loop_ub_tmp_tmp] +
                                        absxk * tform[loop_ub_tmp_tmp + 1]) +
                                       scale * tform[loop_ub_tmp_tmp + 2]) +
                                      w_hat_idx_0 * tform[loop_ub_tmp_tmp + 3];
        }
      }
      for (i2 = 0; i2 < 4; i2++) {
        t = T[i2];
        absxk = T[i2 + 4];
        scale = T[i2 + 8];
        w_hat_idx_0 = T[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] = 0.0;
          b_loop_ub_tmp = i3 << 2;
          pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] =
              pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] +
              t * b_T[b_loop_ub_tmp];
          pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] =
              pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] +
              absxk * b_T[b_loop_ub_tmp + 1];
          pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] =
              pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] +
              scale * b_T[b_loop_ub_tmp + 2];
          pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] =
              pdT[(i2 + 4 * i3) + 16 * (static_cast<int>(b_j) - 1)] +
              w_hat_idx_0 * b_T[b_loop_ub_tmp + 3];
        }
      }
      i2 = static_cast<int>(c_i + (1.0 - (b_j + 1.0)));
      if (i2 - 1 >= 0) {
        dv[0] = 0.0;
        dv[3] = -T[14];
        dv[6] = T[13];
        dv[1] = T[14];
        dv[4] = 0.0;
        dv[7] = -T[12];
        dv[2] = -T[13];
        dv[5] = T[12];
        dv[8] = 0.0;
        c_j = static_cast<int>(b_j);
        d_j = static_cast<int>(b_j) - 1;
        d_i = static_cast<int>(c_i) - 1;
      }
      for (int k{0}; k < i2; k++) {
        w_hat_idx_0 = (b_j + 1.0) + static_cast<double>(k);
        //  w - > [w]
        //  x: vector or m by 3 matrix
        //  X: 3 by 3 by m matrix
        //  w - > [w]
        //  x: vector or m by 3 matrix
        //  X: 3 by 3 by m matrix
        b_skcom[0] = 0.0;
        scale = pdT[16 * (static_cast<int>(w_hat_idx_0) - 1) + 14];
        b_skcom[3] = -scale;
        absxk = pdT[16 * (static_cast<int>(w_hat_idx_0) - 1) + 13];
        b_skcom[6] = absxk;
        b_skcom[1] = scale;
        b_skcom[4] = 0.0;
        scale = pdT[16 * (static_cast<int>(w_hat_idx_0) - 1) + 12];
        b_skcom[7] = -scale;
        b_skcom[2] = -absxk;
        b_skcom[5] = scale;
        b_skcom[8] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          t = b_skcom[i3];
          absxk = b_skcom[i3 + 3];
          scale = b_skcom[i3 + 6];
          for (loop_ub_tmp_tmp = 0; loop_ub_tmp_tmp < 3; loop_ub_tmp_tmp++) {
            b_loop_ub_tmp = loop_ub_tmp_tmp << 2;
            so_w_hat[i3 + 3 * loop_ub_tmp_tmp] =
                (t * T[b_loop_ub_tmp] + absxk * T[b_loop_ub_tmp + 1]) +
                scale * T[b_loop_ub_tmp + 2];
          }
          t = dv[i3];
          absxk = dv[i3 + 3];
          scale = dv[i3 + 6];
          for (loop_ub_tmp_tmp = 0; loop_ub_tmp_tmp < 3; loop_ub_tmp_tmp++) {
            b_skcom[i3 + 3 * loop_ub_tmp_tmp] =
                (t * pdT[4 * loop_ub_tmp_tmp +
                         16 * (static_cast<int>(w_hat_idx_0) - 1)] +
                 absxk * pdT[(4 * loop_ub_tmp_tmp +
                              16 * (static_cast<int>(w_hat_idx_0) - 1)) +
                             1]) +
                scale * pdT[(4 * loop_ub_tmp_tmp +
                             16 * (static_cast<int>(w_hat_idx_0) - 1)) +
                            2];
            adTb[loop_ub_tmp_tmp + 6 * i3] =
                pdT[(loop_ub_tmp_tmp + 4 * i3) +
                    16 * (static_cast<int>(w_hat_idx_0) - 1)];
            adTb[loop_ub_tmp_tmp + 6 * (i3 + 3)] = 0.0;
          }
        }
        for (i3 = 0; i3 < 3; i3++) {
          adTb[6 * i3 + 3] = so_w_hat[3 * i3] + b_skcom[3 * i3];
          c_loop_ub_tmp = 6 * (i3 + 3);
          adTb[c_loop_ub_tmp + 3] =
              pdT[4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)];
          loop_ub_tmp_tmp = 3 * i3 + 1;
          adTb[6 * i3 + 4] =
              so_w_hat[loop_ub_tmp_tmp] + b_skcom[loop_ub_tmp_tmp];
          adTb[c_loop_ub_tmp + 4] =
              pdT[(4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 1];
          loop_ub_tmp_tmp = 3 * i3 + 2;
          adTb[6 * i3 + 5] =
              so_w_hat[loop_ub_tmp_tmp] + b_skcom[loop_ub_tmp_tmp];
          adTb[c_loop_ub_tmp + 5] =
              pdT[(4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 2];
        }
        for (i3 = 0; i3 < 6; i3++) {
          pdJ[((i3 + 6 * d_j) + 6 * pdJ.size(1) * d_i) +
              6 * pdJ.size(1) * pdJ.size(2) *
                  (static_cast<int>(w_hat_idx_0) - 1)] = 0.0;
          for (loop_ub_tmp_tmp = 0; loop_ub_tmp_tmp < 6; loop_ub_tmp_tmp++) {
            pdJ[((i3 + 6 * d_j) + 6 * pdJ.size(1) * d_i) +
                6 * pdJ.size(1) * pdJ.size(2) *
                    (static_cast<int>(w_hat_idx_0) - 1)] =
                pdJ[((i3 + 6 * d_j) + 6 * pdJ.size(1) * d_i) +
                    6 * pdJ.size(1) * pdJ.size(2) *
                        (static_cast<int>(w_hat_idx_0) - 1)] +
                adTb[i3 + 6 * loop_ub_tmp_tmp] *
                    robot->A[(c_j + robot->A.size(0) * loop_ub_tmp_tmp) - 1];
          }
        }
        for (i3 = 0; i3 < 4; i3++) {
          for (loop_ub_tmp_tmp = 0; loop_ub_tmp_tmp < 4; loop_ub_tmp_tmp++) {
            b_loop_ub_tmp = loop_ub_tmp_tmp << 2;
            c_robot[i3 + b_loop_ub_tmp] =
                ((pdT[i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)] *
                      tform[b_loop_ub_tmp] +
                  pdT[(i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 4] *
                      tform[b_loop_ub_tmp + 1]) +
                 pdT[(i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 8] *
                     tform[b_loop_ub_tmp + 2]) +
                pdT[(i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 12] *
                    tform[b_loop_ub_tmp + 3];
          }
        }
        for (i3 = 0; i3 < 4; i3++) {
          b_loop_ub_tmp = i3 << 2;
          pdT[4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)] =
              c_robot[b_loop_ub_tmp];
          pdT[(4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 1] =
              c_robot[b_loop_ub_tmp + 1];
          pdT[(4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 2] =
              c_robot[b_loop_ub_tmp + 2];
          pdT[(4 * i3 + 16 * (static_cast<int>(w_hat_idx_0) - 1)) + 3] =
              c_robot[b_loop_ub_tmp + 3];
        }
        //  if k == j
        //      pdT(:,:,k) = pdT(:,:,k) + T * pdtform;
        //  end
      }
      for (i2 = 0; i2 < 6; i2++) {
        y[i2] = -robot->A[(static_cast<int>(b_j) + robot->A.size(0) * i2) - 1];
      }
      //  v -> [v]
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      c_robot[0] = 0.0;
      c_robot[4] = -y[2];
      c_robot[8] = y[1];
      c_robot[1] = y[2];
      c_robot[5] = 0.0;
      c_robot[9] = -y[0];
      c_robot[2] = -y[1];
      c_robot[6] = y[0];
      c_robot[10] = 0.0;
      c_robot[12] = y[3];
      c_robot[13] = y[4];
      c_robot[14] = y[5];
      c_robot[3] = 0.0;
      c_robot[7] = 0.0;
      c_robot[11] = 0.0;
      c_robot[15] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        t = T[i2];
        absxk = T[i2 + 4];
        scale = T[i2 + 8];
        w_hat_idx_0 = T[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          b_T[i2 + loop_ub_tmp_tmp] =
              ((t * c_robot[loop_ub_tmp_tmp] +
                absxk * c_robot[loop_ub_tmp_tmp + 1]) +
               scale * c_robot[loop_ub_tmp_tmp + 2]) +
              w_hat_idx_0 * c_robot[loop_ub_tmp_tmp + 3];
        }
      }
      for (i2 = 0; i2 < 16; i2++) {
        b_T[i2] = dT[i2] + b_T[i2] * qd[static_cast<int>(b_j) - 1];
      }
      for (i2 = 0; i2 < 4; i2++) {
        t = b_T[i2];
        absxk = b_T[i2 + 4];
        scale = b_T[i2 + 8];
        w_hat_idx_0 = b_T[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          dT[i2 + loop_ub_tmp_tmp] = ((t * tform[loop_ub_tmp_tmp] +
                                       absxk * tform[loop_ub_tmp_tmp + 1]) +
                                      scale * tform[loop_ub_tmp_tmp + 2]) +
                                     w_hat_idx_0 * tform[loop_ub_tmp_tmp + 3];
        }
        t = T[i2];
        absxk = T[i2 + 4];
        scale = T[i2 + 8];
        w_hat_idx_0 = T[i2 + 12];
        for (i3 = 0; i3 < 4; i3++) {
          loop_ub_tmp_tmp = i3 << 2;
          b_T[i2 + loop_ub_tmp_tmp] = ((t * tform[loop_ub_tmp_tmp] +
                                        absxk * tform[loop_ub_tmp_tmp + 1]) +
                                       scale * tform[loop_ub_tmp_tmp + 2]) +
                                      w_hat_idx_0 * tform[loop_ub_tmp_tmp + 3];
        }
      }
      std::copy(&b_T[0], &b_T[16], &T[0]);
    }
    b_loop_ub_tmp = J.size(1);
    b_J.set_size(6, J.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            J[(i2 + 6 * i1) + 6 * J.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    std::copy(&(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[0],
              &(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[36],
              &adTb[0]);
    coder::internal::blas::mtimes(b_J, adTb, r);
    b_loop_ub_tmp = J.size(1);
    b_J.set_size(6, J.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            J[(i2 + 6 * i1) + 6 * J.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    coder::internal::blas::mtimes(r, b_J, b_pdMq);
    if ((Mq.size(0) == b_pdMq.size(0)) && (Mq.size(1) == b_pdMq.size(1))) {
      b_loop_ub_tmp = Mq.size(0) * Mq.size(1);
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        Mq[i1] = Mq[i1] + b_pdMq[i1];
      }
    } else {
      plus(Mq, b_pdMq);
    }
    b_loop_ub_tmp = dJ.size(1);
    b_J.set_size(6, dJ.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            dJ[(i2 + 6 * i1) + 6 * dJ.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    std::copy(&(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[0],
              &(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[36],
              &adTb[0]);
    coder::internal::blas::mtimes(b_J, adTb, r);
    b_loop_ub_tmp = J.size(1);
    b_J.set_size(6, J.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            J[(i2 + 6 * i1) + 6 * J.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    coder::internal::blas::mtimes(r, b_J, b_pdMq);
    b_loop_ub_tmp = J.size(1);
    b_J.set_size(6, J.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            J[(i2 + 6 * i1) + 6 * J.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    std::copy(&(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[0],
              &(*(double(*)[36]) & G[36 * (static_cast<int>(c_i) - 1)])[36],
              &adTb[0]);
    coder::internal::blas::mtimes(b_J, adTb, r);
    b_loop_ub_tmp = dJ.size(1);
    b_J.set_size(6, dJ.size(1));
    for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_J[i2 + 6 * i1] =
            dJ[(i2 + 6 * i1) + 6 * dJ.size(1) * (static_cast<int>(c_i) - 1)];
      }
    }
    coder::internal::blas::mtimes(r, b_J, r1);
    if (dMq.size(0) == 1) {
      i1 = b_pdMq.size(0);
    } else {
      i1 = dMq.size(0);
    }
    if (dMq.size(1) == 1) {
      i2 = b_pdMq.size(1);
    } else {
      i2 = dMq.size(1);
    }
    if ((dMq.size(0) == b_pdMq.size(0)) && (dMq.size(1) == b_pdMq.size(1)) &&
        (i1 == r1.size(0)) && (i2 == r1.size(1))) {
      b_loop_ub_tmp = dMq.size(0) * dMq.size(1);
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        dMq[i1] = (dMq[i1] + b_pdMq[i1]) + r1[i1];
      }
    } else {
      binary_expand_op(dMq, b_pdMq, r1);
    }
    i1 = static_cast<int>(c_i);
    for (int k{0}; k < i1; k++) {
      double b_robot[3];
      for (i2 = 0; i2 < 3; i2++) {
        so_w_hat[3 * i2] = -pdT[i2 + 16 * k];
        so_w_hat[3 * i2 + 1] = -pdT[(i2 + 16 * k) + 4];
        so_w_hat[3 * i2 + 2] = -pdT[(i2 + 16 * k) + 8];
        b_robot[i2] =
            T[i2 + 12] -
            robot->com[(static_cast<int>(c_i) + robot->com.size(0) * i2) - 1];
      }
      t = b_robot[0];
      absxk = b_robot[1];
      scale = b_robot[2];
      for (i2 = 0; i2 < 3; i2++) {
        b_robot[i2] = (so_w_hat[i2] * t + so_w_hat[i2 + 3] * absxk) +
                      so_w_hat[i2 + 6] * scale;
      }
      // T的逆是第i个连杆的坐标系矩阵
      w_hat_idx_0 = 0.0;
      for (b_loop_ub_tmp = 0; b_loop_ub_tmp < 3; b_loop_ub_tmp++) {
        i2 = b_loop_ub_tmp << 2;
        w_hat_idx_0 +=
            robot->gravity[b_loop_ub_tmp] *
            (b_robot[b_loop_ub_tmp] -
             ((T[i2] * pdT[16 * k + 12] + T[i2 + 1] * pdT[16 * k + 13]) +
              T[i2 + 2] * pdT[16 * k + 14]));
      }
      P[(static_cast<int>(c_i) + P.size(0) * k) - 1] =
          -robot->mass[static_cast<int>(c_i) - 1] * w_hat_idx_0;
      // 注意负号
    }
    if (c_i == n) {
      //  T: 4x4xn
      for (i1 = 0; i1 < 3; i1++) {
        so_w_hat[3 * i1] = -T[i1];
        so_w_hat[3 * i1 + 1] = -T[i1 + 4];
        so_w_hat[3 * i1 + 2] = -T[i1 + 8];
      }
      for (i1 = 0; i1 < 3; i1++) {
        b_loop_ub_tmp = i1 << 2;
        a[b_loop_ub_tmp] = T[i1];
        a[b_loop_ub_tmp + 1] = T[i1 + 4];
        a[b_loop_ub_tmp + 2] = T[i1 + 8];
        a[i1 + 12] = (so_w_hat[i1] * T[12] + so_w_hat[i1 + 3] * T[13]) +
                     so_w_hat[i1 + 6] * T[14];
      }
      a[3] = 0.0;
      a[7] = 0.0;
      a[11] = 0.0;
      a[15] = 1.0;
      for (i1 = 0; i1 < 4; i1++) {
        t = a[i1];
        absxk = a[i1 + 4];
        scale = a[i1 + 8];
        w_hat_idx_0 = a[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          Tcp[i1 + i3] =
              ((t * ME[i3] + absxk * ME[i3 + 1]) + scale * ME[i3 + 2]) +
              w_hat_idx_0 * ME[i3 + 3];
        }
      }
      for (i1 = 0; i1 < 16; i1++) {
        b_T[i1] = -a[i1];
      }
      for (i1 = 0; i1 < 4; i1++) {
        t = b_T[i1];
        absxk = b_T[i1 + 4];
        scale = b_T[i1 + 8];
        w_hat_idx_0 = b_T[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          c_robot[i1 + i3] =
              ((t * dT[i3] + absxk * dT[i3 + 1]) + scale * dT[i3 + 2]) +
              w_hat_idx_0 * dT[i3 + 3];
        }
        t = c_robot[i1];
        absxk = c_robot[i1 + 4];
        scale = c_robot[i1 + 8];
        w_hat_idx_0 = c_robot[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          b_T[i1 + i3] = ((t * a[i3] + absxk * a[i3 + 1]) + scale * a[i3 + 2]) +
                         w_hat_idx_0 * a[i3 + 3];
        }
        t = b_T[i1];
        absxk = b_T[i1 + 4];
        scale = b_T[i1 + 8];
        w_hat_idx_0 = b_T[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          i3 = i2 << 2;
          dTcp[i1 + i3] =
              ((t * ME[i3] + absxk * ME[i3 + 1]) + scale * ME[i3 + 2]) +
              w_hat_idx_0 * ME[i3 + 3];
        }
      }
      //  T: 4x4xn
      for (i1 = 0; i1 < 3; i1++) {
        so_w_hat[3 * i1] = -ME[i1];
        so_w_hat[3 * i1 + 1] = -ME[i1 + 4];
        so_w_hat[3 * i1 + 2] = -ME[i1 + 8];
      }
      for (i1 = 0; i1 < 3; i1++) {
        b_loop_ub_tmp = i1 << 2;
        a[b_loop_ub_tmp] = ME[i1];
        a[b_loop_ub_tmp + 1] = ME[i1 + 4];
        a[b_loop_ub_tmp + 2] = ME[i1 + 8];
        a[i1 + 12] = (so_w_hat[i1] * ME[12] + so_w_hat[i1 + 3] * ME[13]) +
                     so_w_hat[i1 + 6] * ME[14];
      }
      a[3] = 0.0;
      a[7] = 0.0;
      a[11] = 0.0;
      a[15] = 1.0;
      //  AdT operator T -> 6x6 mapping
      std::memset(&adTb[0], 0, 36U * sizeof(double));
      for (i1 = 0; i1 < 3; i1++) {
        b_loop_ub_tmp = i1 << 2;
        w_hat_idx_0 = a[b_loop_ub_tmp];
        adTb[6 * i1] = w_hat_idx_0;
        c_loop_ub_tmp = 6 * (i1 + 3);
        adTb[c_loop_ub_tmp + 3] = w_hat_idx_0;
        w_hat_idx_0 = a[b_loop_ub_tmp + 1];
        adTb[6 * i1 + 1] = w_hat_idx_0;
        adTb[c_loop_ub_tmp + 4] = w_hat_idx_0;
        w_hat_idx_0 = a[b_loop_ub_tmp + 2];
        adTb[6 * i1 + 2] = w_hat_idx_0;
        adTb[c_loop_ub_tmp + 5] = w_hat_idx_0;
      }
      //  w - > [w]
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      b_skcom[0] = 0.0;
      b_skcom[3] = -a[14];
      b_skcom[6] = a[13];
      b_skcom[1] = a[14];
      b_skcom[4] = 0.0;
      b_skcom[7] = -a[12];
      b_skcom[2] = -a[13];
      b_skcom[5] = a[12];
      b_skcom[8] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        t = b_skcom[i1];
        absxk = b_skcom[i1 + 3];
        scale = b_skcom[i1 + 6];
        for (i2 = 0; i2 < 3; i2++) {
          i3 = i2 << 2;
          adTb[(i1 + 6 * i2) + 3] =
              (t * a[i3] + absxk * a[i3 + 1]) + scale * a[i3 + 2];
        }
      }
      b_loop_ub_tmp = J.size(1);
      b_J.set_size(6, J.size(1));
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        for (i2 = 0; i2 < 6; i2++) {
          b_J[i2 + 6 * i1] =
              J[(i2 + 6 * i1) + 6 * J.size(1) * (loop_ub_tmp - 1)];
        }
      }
      b_loop_ub_tmp = J.size(1);
      Jb.set_size(6, J.size(1));
      for (int j{0}; j < b_loop_ub_tmp; j++) {
        loop_ub_tmp_tmp = j * 6;
        for (c_loop_ub_tmp = 0; c_loop_ub_tmp < 6; c_loop_ub_tmp++) {
          w_hat_idx_0 = 0.0;
          for (int k{0}; k < 6; k++) {
            w_hat_idx_0 +=
                adTb[k * 6 + c_loop_ub_tmp] * b_J[loop_ub_tmp_tmp + k];
          }
          Jb[loop_ub_tmp_tmp + c_loop_ub_tmp] = w_hat_idx_0;
        }
      }
      b_loop_ub_tmp = dJ.size(1);
      b_J.set_size(6, dJ.size(1));
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        for (i2 = 0; i2 < 6; i2++) {
          b_J[i2 + 6 * i1] =
              dJ[(i2 + 6 * i1) + 6 * dJ.size(1) * (static_cast<int>(n) - 1)];
        }
      }
      b_loop_ub_tmp = dJ.size(1);
      dJb.set_size(6, dJ.size(1));
      for (int j{0}; j < b_loop_ub_tmp; j++) {
        loop_ub_tmp_tmp = j * 6;
        for (c_loop_ub_tmp = 0; c_loop_ub_tmp < 6; c_loop_ub_tmp++) {
          w_hat_idx_0 = 0.0;
          for (int k{0}; k < 6; k++) {
            w_hat_idx_0 +=
                adTb[k * 6 + c_loop_ub_tmp] * b_J[loop_ub_tmp_tmp + k];
          }
          dJb[loop_ub_tmp_tmp + c_loop_ub_tmp] = w_hat_idx_0;
        }
      }
    }
  }
  //  g matrix
  for (int b_i{0}; b_i < loop_ub_tmp; b_i++) {
    for (int j{0}; j < loop_ub_tmp; j++) {
      b_loop_ub_tmp = pdJ.size(1);
      b_J.set_size(6, pdJ.size(1));
      for (i = 0; i < b_loop_ub_tmp; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          b_J[i1 + 6 * i] = pdJ[((i1 + 6 * i) + 6 * pdJ.size(1) * j) +
                                6 * pdJ.size(1) * pdJ.size(2) * b_i];
        }
      }
      std::copy(&(*(double(*)[36]) & G[36 * j])[0],
                &(*(double(*)[36]) & G[36 * j])[36], &adTb[0]);
      coder::internal::blas::mtimes(b_J, adTb, r);
      b_loop_ub_tmp = J.size(1);
      b_J.set_size(6, J.size(1));
      for (i = 0; i < b_loop_ub_tmp; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          b_J[i1 + 6 * i] = J[(i1 + 6 * i) + 6 * J.size(1) * j];
        }
      }
      coder::internal::blas::mtimes(r, b_J, b_pdMq);
      b_loop_ub_tmp = J.size(1);
      b_J.set_size(6, J.size(1));
      for (i = 0; i < b_loop_ub_tmp; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          b_J[i1 + 6 * i] = J[(i1 + 6 * i) + 6 * J.size(1) * j];
        }
      }
      std::copy(&(*(double(*)[36]) & G[36 * j])[0],
                &(*(double(*)[36]) & G[36 * j])[36], &adTb[0]);
      coder::internal::blas::mtimes(b_J, adTb, r);
      b_loop_ub_tmp = pdJ.size(1);
      b_J.set_size(6, pdJ.size(1));
      for (i = 0; i < b_loop_ub_tmp; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          b_J[i1 + 6 * i] = pdJ[((i1 + 6 * i) + 6 * pdJ.size(1) * j) +
                                6 * pdJ.size(1) * pdJ.size(2) * b_i];
        }
      }
      coder::internal::blas::mtimes(r, b_J, r1);
      b_loop_ub_tmp = pdMq.size(1);
      if (pdMq.size(0) == 1) {
        i = b_pdMq.size(0);
      } else {
        i = pdMq.size(0);
      }
      if (pdMq.size(1) == 1) {
        i1 = b_pdMq.size(1);
      } else {
        i1 = pdMq.size(1);
      }
      if ((pdMq.size(0) == b_pdMq.size(0)) &&
          (pdMq.size(1) == b_pdMq.size(1)) && (i == r1.size(0)) &&
          (i1 == r1.size(1))) {
        b_pdMq.set_size(pdMq.size(0), pdMq.size(1));
        for (i = 0; i < b_loop_ub_tmp; i++) {
          loop_ub = pdMq.size(0);
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_pdMq[i1 + b_pdMq.size(0) * i] =
                (pdMq[(i1 + pdMq.size(0) * i) +
                      pdMq.size(0) * pdMq.size(1) * b_i] +
                 b_pdMq[i1 + b_pdMq.size(0) * i]) +
                r1[i1 + r1.size(0) * i];
          }
        }
        b_loop_ub_tmp = b_pdMq.size(1);
        for (i = 0; i < b_loop_ub_tmp; i++) {
          loop_ub = b_pdMq.size(0);
          for (i1 = 0; i1 < loop_ub; i1++) {
            pdMq[(i1 + pdMq.size(0) * i) + pdMq.size(0) * pdMq.size(1) * b_i] =
                b_pdMq[i1 + b_pdMq.size(0) * i];
          }
        }
      } else {
        binary_expand_op(pdMq, b_i, b_pdMq, r1);
      }
      g[b_i] = g[b_i] + P[j + P.size(0) * b_i];
    }
  }
  //  C matrix
  for (int k{0}; k < loop_ub_tmp; k++) {
    for (int j{0}; j < loop_ub_tmp; j++) {
      for (int b_i{0}; b_i < loop_ub_tmp; b_i++) {
        C[k + C.size(0) * j] = C[k + C.size(0) * j] +
                               0.5 *
                                   ((pdMq[(k + pdMq.size(0) * j) +
                                          pdMq.size(0) * pdMq.size(1) * b_i] +
                                     pdMq[(k + pdMq.size(0) * b_i) +
                                          pdMq.size(0) * pdMq.size(1) * j]) -
                                    pdMq[(b_i + pdMq.size(0) * j) +
                                         pdMq.size(0) * pdMq.size(1) * k]) *
                                   qd[b_i];
      }
    }
  }
}

//
// File trailer for m_c_g_matrix.cpp
//
// [EOF]
//
