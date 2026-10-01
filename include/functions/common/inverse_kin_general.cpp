//
// File: inverse_kin_general.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 22:44:18
//

// Include Files
#include "inverse_kin_general.h"
#include "logR.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <math.h>



static double rt_hypotd_snf(double u0, double u1)
{
    double a;
    double y;
    a = std::abs(u0);
    y = std::abs(u1);
    if (a < y) {
        a /= y;
        y *= std::sqrt(a * a + 1.0);
    }
    else if (a > y) {
        y /= a;
        y = a * std::sqrt(y * y + 1.0);
    }
    else {
        if (!rtIsNaN(y)) {
            y = a * 1.4142135623730951;
        }
    }

    return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
        y = rtNaN;
    }
    else {
        double d;
        double d1;
        d = std::abs(u0);
        d1 = std::abs(u1);
        if (rtIsInf(u1)) {
            if (d == 1.0) {
                y = 1.0;
            }
            else if (d > 1.0) {
                if (u1 > 0.0) {
                    y = rtInf;
                }
                else {
                    y = 0.0;
                }
            }
            else if (u1 > 0.0) {
                y = 0.0;
            }
            else {
                y = rtInf;
            }
        }
        else if (d1 == 0.0) {
            y = 1.0;
        }
        else if (d1 == 1.0) {
            if (u1 > 0.0) {
                y = u0;
            }
            else {
                y = 1.0 / u0;
            }
        }
        else if (u1 == 2.0) {
            y = u0 * u0;
        }
        else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = std::sqrt(u0);
        }
        else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
            y = rtNaN;
        }
        else {
            y = std::pow(u0, u1);
        }
    }

    return y;
}

static void jacobian_matrix(double robot_dof, const coder::array<double, 2U>& robot_A,
    const coder::array<double, 3U>& robot_M, const double
    robot_ME[16], const coder::array<double, 2U>& q, coder::
    array<double, 2U>& Jb, double T[16])
{
    static const signed char b_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    double a[36];
    double b_robot_M[16];
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
    std::memcpy(&T[0], &robot_ME[0], 16U * sizeof(double));
    Jb.set_size(6, (static_cast<int>(robot_dof)));
    i = static_cast<int>(((-1.0 - robot_dof) + 1.0) / -1.0);
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
        c_i = robot_dof + -static_cast<double>(b_i);

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
                    (c_i)-1)] + a[i1 + 6 * a_tmp] * robot_A[(static_cast<int>(c_i) +
                        robot_A.size(0) * a_tmp) - 1];
            }

            y[i1] = robot_A[(static_cast<int>(c_i) + robot_A.size(0) * i1) - 1] *
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
            }
            else {
                t = absxk / scale;
                w_hat_idx_0 += t * t;
            }
        }

        w_hat_idx_0 = scale * std::sqrt(w_hat_idx_0);
        if (w_hat_idx_0 <= 2.2204460492503131E-16) {
            sa[5] = 1.0;
        }
        else {
            scale = 3.3121686421112381E-170;
            absxk = std::abs(y[0]);
            if (absxk > 3.3121686421112381E-170) {
                theta = 1.0;
                scale = absxk;
            }
            else {
                t = absxk / 3.3121686421112381E-170;
                theta = t * t;
            }

            absxk = std::abs(y[1]);
            if (absxk > scale) {
                t = scale / absxk;
                theta = theta * t * t + 1.0;
                scale = absxk;
            }
            else {
                t = absxk / scale;
                theta += t * t;
            }

            absxk = std::abs(y[2]);
            if (absxk > scale) {
                t = scale / absxk;
                theta = theta * t * t + 1.0;
                scale = absxk;
            }
            else {
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
                }
                else {
                    t = absxk / 3.3121686421112381E-170;
                    w_hat_idx_0 = t * t;
                }

                absxk = std::abs(y[4]);
                if (absxk > scale) {
                    t = scale / absxk;
                    w_hat_idx_0 = w_hat_idx_0 * t * t + 1.0;
                    scale = absxk;
                }
                else {
                    t = absxk / scale;
                    w_hat_idx_0 += t * t;
                }

                absxk = std::abs(y[5]);
                if (absxk > scale) {
                    t = scale / absxk;
                    w_hat_idx_0 = w_hat_idx_0 * t * t + 1.0;
                    scale = absxk;
                }
                else {
                    t = absxk / scale;
                    w_hat_idx_0 += t * t;
                }

                w_hat_idx_0 = scale * std::sqrt(w_hat_idx_0);
                sa[6] = w_hat_idx_0;
                sa[3] = y[3] / w_hat_idx_0;
                sa[4] = y[4] / w_hat_idx_0;
                sa[5] = y[5] / w_hat_idx_0;
            }
            else {
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
        }
        else {
            t = absxk / 3.3121686421112381E-170;
            theta = t * t;
        }

        absxk = std::abs(y[1]);
        if (absxk > scale) {
            t = scale / absxk;
            theta = theta * t * t + 1.0;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            theta += t * t;
        }

        absxk = std::abs(y[2]);
        if (absxk > scale) {
            t = scale / absxk;
            theta = theta * t * t + 1.0;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            theta += t * t;
        }

        theta = scale * std::sqrt(theta);
        if (theta <= 2.2204460492503131E-16) {
            std::memset(&so_w_hat[0], 0, 9U * sizeof(double));
            so_w_hat[0] = 1.0;
            so_w_hat[4] = 1.0;
            so_w_hat[8] = 1.0;
        }
        else {
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
                b_robot_M[i1 + invT_tmp] = ((robot_M[i1 + 16 * (static_cast<int>(c_i) -
                    1)] * invT[invT_tmp] + robot_M[(i1 + 16 * (static_cast<int>(c_i) - 1))
                    + 4] * invT[invT_tmp + 1]) + robot_M[(i1 + 16 * (static_cast<int>(c_i)
                        - 1)) + 8] * invT[invT_tmp + 2]) + robot_M[(i1 + 16 * (static_cast<int>
                            (c_i)-1)) + 12] * invT[invT_tmp + 3];
            }
        }

        for (i1 = 0; i1 < 4; i1++) {
            absxk = b_robot_M[i1];
            t = b_robot_M[i1 + 4];
            scale = b_robot_M[i1 + 8];
            w_hat_idx_0 = b_robot_M[i1 + 12];
            for (a_tmp = 0; a_tmp < 4; a_tmp++) {
                invT_tmp = a_tmp << 2;
                invT[i1 + invT_tmp] = ((absxk * T[invT_tmp] + t * T[invT_tmp + 1]) +
                    scale * T[invT_tmp + 2]) + w_hat_idx_0 * T[invT_tmp + 3];
            }
        }

        std::memcpy(&T[0], &invT[0], 16U * sizeof(double));
    }
}

// Function Definitions
//
// Arguments    : const double Td[16]
//                double robot_dof
//                const coder::array<double, 2U> &robot_A
//                const coder::array<double, 3U> &robot_M
//                const double robot_ME[16]
//                const coder::array<double, 2U> &x
//                double varargout_1[6]
//                coder::array<double, 2U> &varargout_2
// Return Type  : void
//
static void anon(const double Td[16], double robot_dof, const coder::array<double, 2U>
    & robot_A, const coder::array<double, 3U>& robot_M, const double
    robot_ME[16], const coder::array<double, 2U>& x, double varargout_1[6],
    coder::array<double, 2U>& varargout_2)
{
    coder::array<double, 2U> Jb;
    double a[36];
    double T[16];
    double b_I[9];
    double b_Td[9];
    double sr[9];
    double r[6];
    double dv[3];
    double absx11;
    double absx31;
    double absxk;
    double norm_rd;
    double rd2_idx_0;
    double rd2_idx_1;
    double rd2_idx_2;
    double rd_idx_0;
    double rd_idx_1;
    double rd_idx_2;
    double scale;
    double t;
    int itmp;
    int p1;
    int p2;
    int p3;
    for (p3 = 0; p3 < 3; p3++) {
        p1 = p3 << 2;
        b_Td[3 * p3] = Td[p1];
        b_Td[3 * p3 + 1] = Td[p1 + 1];
        b_Td[3 * p3 + 2] = Td[p1 + 2];
    }

    logR(b_Td, dv);
    scale = 3.3121686421112381E-170;
    rd_idx_0 = dv[0];
    absxk = std::abs(dv[0]);
    if (absxk > 3.3121686421112381E-170) {
        norm_rd = 1.0;
        scale = absxk;
    }
    else {
        t = absxk / 3.3121686421112381E-170;
        norm_rd = t * t;
    }

    rd_idx_1 = dv[1];
    absxk = std::abs(dv[1]);
    if (absxk > scale) {
        t = scale / absxk;
        norm_rd = norm_rd * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        norm_rd += t * t;
    }

    rd_idx_2 = dv[2];
    absxk = std::abs(dv[2]);
    if (absxk > scale) {
        t = scale / absxk;
        norm_rd = norm_rd * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        norm_rd += t * t;
    }

    norm_rd = scale * std::sqrt(norm_rd);
    if (norm_rd != 0.0) {
        rd2_idx_0 = (6.2831853071795862 - norm_rd) * (-dv[0] / norm_rd);
        rd2_idx_1 = (6.2831853071795862 - norm_rd) * (-dv[1] / norm_rd);
        rd2_idx_2 = (6.2831853071795862 - norm_rd) * (-dv[2] / norm_rd);
    }
    else {
        rd2_idx_0 = dv[0];
        rd2_idx_1 = dv[1];
        rd2_idx_2 = dv[2];
    }

    jacobian_matrix(robot_dof, robot_A, robot_M, robot_ME, x, Jb, T);
    for (p3 = 0; p3 < 3; p3++) {
        p1 = p3 << 2;
        b_Td[3 * p3] = T[p1];
        b_Td[3 * p3 + 1] = T[p1 + 1];
        b_Td[3 * p3 + 2] = T[p1 + 2];
    }

    logR(b_Td, dv);

    //  wb = A(r)dr/dt
    scale = 3.3121686421112381E-170;
    absxk = std::abs(dv[0]);
    if (absxk > 3.3121686421112381E-170) {
        norm_rd = 1.0;
        scale = absxk;
    }
    else {
        t = absxk / 3.3121686421112381E-170;
        norm_rd = t * t;
    }

    absxk = std::abs(dv[1]);
    if (absxk > scale) {
        t = scale / absxk;
        norm_rd = norm_rd * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        norm_rd += t * t;
    }

    absxk = std::abs(dv[2]);
    if (absxk > scale) {
        t = scale / absxk;
        norm_rd = norm_rd * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        norm_rd += t * t;
    }

    norm_rd = scale * std::sqrt(norm_rd);
    if (norm_rd != 0.0) {
        //  x: vector or m by 3 matrix
        //  X: 3 by 3 by m matrix
        sr[0] = 0.0;
        sr[3] = -dv[2];
        sr[6] = dv[1];
        sr[1] = dv[2];
        sr[4] = 0.0;
        sr[7] = -dv[0];
        sr[2] = -dv[1];
        sr[5] = dv[0];
        sr[8] = 0.0;
        absx11 = (1.0 - std::cos(norm_rd)) / (norm_rd * norm_rd);
        norm_rd = (norm_rd - std::sin(norm_rd)) / rt_powd_snf(norm_rd, 3.0);
        std::memset(&b_I[0], 0, 9U * sizeof(double));
        b_I[0] = 1.0;
        b_I[4] = 1.0;
        b_I[8] = 1.0;
        for (p3 = 0; p3 < 3; p3++) {
            for (p2 = 0; p2 < 3; p2++) {
                p1 = p3 + 3 * p2;
                b_Td[p1] = (b_I[p1] - absx11 * sr[p1]) + ((norm_rd * sr[p3] * sr[3 * p2]
                    + norm_rd * sr[p3 + 3] * sr[3 * p2 + 1]) + norm_rd * sr[p3 + 6] * sr[3
                    * p2 + 2]);
            }
        }
    }
    else {
        std::memset(&b_Td[0], 0, 9U * sizeof(double));
        b_Td[0] = 1.0;
        b_Td[4] = 1.0;
        b_Td[8] = 1.0;
    }

    std::memcpy(&b_I[0], &b_Td[0], 9U * sizeof(double));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = std::abs(b_Td[0]);
    norm_rd = std::abs(b_Td[1]);
    absx31 = std::abs(b_Td[2]);
    if ((norm_rd > absx11) && (norm_rd > absx31)) {
        p1 = 3;
        p2 = 0;
        b_I[0] = b_Td[1];
        b_I[1] = b_Td[0];
        b_I[3] = b_Td[4];
        b_I[4] = b_Td[3];
        b_I[6] = b_Td[7];
        b_I[7] = b_Td[6];
    }
    else {
        if (absx31 > absx11) {
            p1 = 6;
            p3 = 0;
            b_I[0] = b_Td[2];
            b_I[2] = b_Td[0];
            b_I[3] = b_Td[5];
            b_I[5] = b_Td[3];
            b_I[6] = b_Td[8];
            b_I[8] = b_Td[6];
        }
    }

    b_I[1] /= b_I[0];
    b_I[2] /= b_I[0];
    b_I[4] -= b_I[1] * b_I[3];
    b_I[5] -= b_I[2] * b_I[3];
    b_I[7] -= b_I[1] * b_I[6];
    b_I[8] -= b_I[2] * b_I[6];
    if (std::abs(b_I[5]) > std::abs(b_I[4])) {
        itmp = p2;
        p2 = p3;
        p3 = itmp;
        norm_rd = b_I[1];
        b_I[1] = b_I[2];
        b_I[2] = norm_rd;
        norm_rd = b_I[4];
        b_I[4] = b_I[5];
        b_I[5] = norm_rd;
        norm_rd = b_I[7];
        b_I[7] = b_I[8];
        b_I[8] = norm_rd;
    }

    b_I[5] /= b_I[4];
    b_I[8] -= b_I[5] * b_I[7];
    norm_rd = (b_I[5] * b_I[1] - b_I[2]) / b_I[8];
    absx11 = -(b_I[1] + b_I[7] * norm_rd) / b_I[4];
    sr[p1] = ((1.0 - b_I[3] * absx11) - b_I[6] * norm_rd) / b_I[0];
    sr[p1 + 1] = absx11;
    sr[p1 + 2] = norm_rd;
    norm_rd = -b_I[5] / b_I[8];
    absx11 = (1.0 - b_I[7] * norm_rd) / b_I[4];
    sr[p2] = -(b_I[3] * absx11 + b_I[6] * norm_rd) / b_I[0];
    sr[p2 + 1] = absx11;
    sr[p2 + 2] = norm_rd;
    norm_rd = 1.0 / b_I[8];
    absx11 = -b_I[7] * norm_rd / b_I[4];
    sr[p3] = -(b_I[3] * absx11 + b_I[6] * norm_rd) / b_I[0];
    sr[p3 + 1] = absx11;
    sr[p3 + 2] = norm_rd;
    for (p3 = 0; p3 < 3; p3++) {
        a[6 * p3] = sr[3 * p3];
        p1 = 6 * (p3 + 3);
        a[p1] = 0.0;
        a[6 * p3 + 3] = 0.0;
        p2 = p3 << 2;
        a[p1 + 3] = T[p2];
        a[6 * p3 + 1] = sr[3 * p3 + 1];
        a[p1 + 1] = 0.0;
        a[6 * p3 + 4] = 0.0;
        a[p1 + 4] = T[p2 + 1];
        a[6 * p3 + 2] = sr[3 * p3 + 2];
        a[p1 + 2] = 0.0;
        a[6 * p3 + 5] = 0.0;
        a[p1 + 5] = T[p2 + 2];
    }

    p1 = Jb.size(1);
    varargout_2.set_size(6, Jb.size(1));
    for (p3 = 0; p3 < p1; p3++) {
        p2 = p3 * 6;
        for (itmp = 0; itmp < 6; itmp++) {
            norm_rd = 0.0;
            for (int k = 0; k < 6; k++) {
                norm_rd += a[k * 6 + itmp] * Jb[p2 + k];
            }

            varargout_2[p2 + itmp] = norm_rd;
        }
    }

    p1 = varargout_2.size(0) * varargout_2.size(1);
    varargout_2.set_size(6, varargout_2.size(1));
    for (p3 = 0; p3 < p1; p3++) {
        varargout_2[p3] = -varargout_2[p3];
    }

    for (p3 = 0; p3 < 3; p3++) {
        p1 = p3 << 2;
        b_Td[3 * p3] = T[p1];
        b_Td[3 * p3 + 1] = T[p1 + 1];
        b_Td[3 * p3 + 2] = T[p1 + 2];
    }

    logR(b_Td, dv);
    scale = 3.3121686421112381E-170;
    norm_rd = 3.3121686421112381E-170;
    absxk = std::abs(rd2_idx_0 - dv[0]);
    if (absxk > 3.3121686421112381E-170) {
        absx11 = 1.0;
        scale = absxk;
    }
    else {
        t = absxk / 3.3121686421112381E-170;
        absx11 = t * t;
    }

    absxk = std::abs(rd_idx_0 - dv[0]);
    if (absxk > 3.3121686421112381E-170) {
        absx31 = 1.0;
        norm_rd = absxk;
    }
    else {
        t = absxk / 3.3121686421112381E-170;
        absx31 = t * t;
    }

    absxk = std::abs(rd2_idx_1 - dv[1]);
    if (absxk > scale) {
        t = scale / absxk;
        absx11 = absx11 * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        absx11 += t * t;
    }

    absxk = std::abs(rd_idx_1 - dv[1]);
    if (absxk > norm_rd) {
        t = norm_rd / absxk;
        absx31 = absx31 * t * t + 1.0;
        norm_rd = absxk;
    }
    else {
        t = absxk / norm_rd;
        absx31 += t * t;
    }

    absxk = std::abs(rd2_idx_2 - dv[2]);
    if (absxk > scale) {
        t = scale / absxk;
        absx11 = absx11 * t * t + 1.0;
        scale = absxk;
    }
    else {
        t = absxk / scale;
        absx11 += t * t;
    }

    absxk = std::abs(rd_idx_2 - dv[2]);
    if (absxk > norm_rd) {
        t = norm_rd / absxk;
        absx31 = absx31 * t * t + 1.0;
        norm_rd = absxk;
    }
    else {
        t = absxk / norm_rd;
        absx31 += t * t;
    }

    absx11 = scale * std::sqrt(absx11);
    absx31 = norm_rd * std::sqrt(absx31);
    if (absx11 > absx31) {
        varargout_1[0] = rd_idx_0;
        varargout_1[3] = Td[12];
        r[0] = dv[0];
        r[3] = T[12];
        varargout_1[1] = rd_idx_1;
        varargout_1[4] = Td[13];
        r[1] = dv[1];
        r[4] = T[13];
        varargout_1[2] = rd_idx_2;
        varargout_1[5] = Td[14];
        r[2] = dv[2];
        r[5] = T[14];
        for (p3 = 0; p3 < 6; p3++) {
            varargout_1[p3] -= r[p3];
        }
    }
    else {
        varargout_1[0] = rd2_idx_0;
        varargout_1[3] = Td[12];
        r[0] = dv[0];
        r[3] = T[12];
        varargout_1[1] = rd2_idx_1;
        varargout_1[4] = Td[13];
        r[1] = dv[1];
        r[4] = T[13];
        varargout_1[2] = rd2_idx_2;
        varargout_1[5] = Td[14];
        r[2] = dv[2];
        r[5] = T[14];
        for (p3 = 0; p3 < 6; p3++) {
            varargout_1[p3] -= r[p3];
        }
    }
}

struct cell_1
{
    double f1[16];
    Robot f2;
};

namespace coder
{
    namespace internal
    {
        

        namespace blas
        {
            static void b_xswap(double x_data[], int ix0, int iy0)
            {
                int ix;
                int iy;
                ix = ix0 - 1;
                iy = iy0 - 1;
                for (int k = 0; k < 6; k++) {
                    double temp;
                    temp = x_data[ix];
                    x_data[ix] = x_data[iy];
                    x_data[iy] = temp;
                    ix++;
                    iy++;
                }
            }

            //
            // Arguments    : double x[36]
            //                int ix0
            //                int iy0
            // Return Type  : void
            //
            static void xswap(double x[36], int ix0, int iy0)
            {
                int ix;
                int iy;
                ix = ix0 - 1;
                iy = iy0 - 1;
                for (int k = 0; k < 6; k++) {
                    double temp;
                    temp = x[ix];
                    x[ix] = x[iy];
                    x[iy] = temp;
                    ix++;
                    iy++;
                }
            }

            static void xrotg(double* a, double* b, double* c, double* s)
            {
                double absa;
                double absb;
                double roe;
                double scale;
                roe = *b;
                absa = std::abs(*a);
                absb = std::abs(*b);
                if (absa > absb) {
                    roe = *a;
                }

                scale = absa + absb;
                if (scale == 0.0) {
                    *s = 0.0;
                    *c = 1.0;
                    *a = 0.0;
                    *b = 0.0;
                }
                else {
                    double ads;
                    double bds;
                    ads = absa / scale;
                    bds = absb / scale;
                    scale *= std::sqrt(ads * ads + bds * bds);
                    if (roe < 0.0) {
                        scale = -scale;
                    }

                    *c = *a / scale;
                    *s = *b / scale;
                    if (absa > absb) {
                        *b = *s;
                    }
                    else if (*c != 0.0) {
                        *b = 1.0 / *c;
                    }
                    else {
                        *b = 1.0;
                    }

                    *a = scale;
                }
            }

            static void b_xrot(double x_data[], int ix0, int iy0, double c, double s)
            {
                int ix;
                int iy;
                ix = ix0 - 1;
                iy = iy0 - 1;
                for (int k = 0; k < 6; k++) {
                    double temp;
                    temp = c * x_data[ix] + s * x_data[iy];
                    x_data[iy] = c * x_data[iy] - s * x_data[ix];
                    x_data[ix] = temp;
                    iy++;
                    ix++;
                }
            }

            //
            // Arguments    : double x[36]
            //                int ix0
            //                int iy0
            //                double c
            //                double s
            // Return Type  : void
            //
            static void xrot(double x[36], int ix0, int iy0, double c, double s)
            {
                int ix;
                int iy;
                ix = ix0 - 1;
                iy = iy0 - 1;
                for (int k = 0; k < 6; k++) {
                    double temp;
                    temp = c * x[ix] + s * x[iy];
                    x[iy] = c * x[iy] - s * x[ix];
                    x[ix] = temp;
                    iy++;
                    ix++;
                }
            }

            static double b_xnrm2(int n, const ::coder::array<double, 2U>& x, int ix0)
            {
                double y;
                y = 0.0;
                if (n >= 1) {
                    if (n == 1) {
                        y = std::abs(x[ix0 - 1]);
                    }
                    else {
                        double scale;
                        int kend;
                        scale = 3.3121686421112381E-170;
                        kend = (ix0 + n) - 1;
                        for (int k = ix0; k <= kend; k++) {
                            double absxk;
                            absxk = std::abs(x[k - 1]);
                            if (absxk > scale) {
                                double t;
                                t = scale / absxk;
                                y = y * t * t + 1.0;
                                scale = absxk;
                            }
                            else {
                                double t;
                                t = absxk / scale;
                                y += t * t;
                            }
                        }

                        y = scale * std::sqrt(y);
                    }
                }

                return y;
            }

            //
            // Arguments    : int n
            //                const double x_data[]
            //                int ix0
            // Return Type  : double
            //
            static double b_xnrm2(int n, const double x_data[], int ix0)
            {
                double scale;
                double y;
                int kend;
                y = 0.0;
                scale = 3.3121686421112381E-170;
                kend = (ix0 + n) - 1;
                for (int k = ix0; k <= kend; k++) {
                    double absxk;
                    absxk = std::abs(x_data[k - 1]);
                    if (absxk > scale) {
                        double t;
                        t = scale / absxk;
                        y = y * t * t + 1.0;
                        scale = absxk;
                    }
                    else {
                        double t;
                        t = absxk / scale;
                        y += t * t;
                    }
                }

                return scale * std::sqrt(y);
            }

            //
            // Arguments    : int n
            //                const double x_data[]
            //                int ix0
            // Return Type  : double
            //
            static double c_xnrm2(int n, const double x_data[], int ix0)
            {
                double y;
                y = 0.0;
                if (n >= 1) {
                    if (n == 1) {
                        y = std::abs(x_data[ix0 - 1]);
                    }
                    else {
                        double scale;
                        int kend;
                        scale = 3.3121686421112381E-170;
                        kend = (ix0 + n) - 1;
                        for (int k = ix0; k <= kend; k++) {
                            double absxk;
                            absxk = std::abs(x_data[k - 1]);
                            if (absxk > scale) {
                                double t;
                                t = scale / absxk;
                                y = y * t * t + 1.0;
                                scale = absxk;
                            }
                            else {
                                double t;
                                t = absxk / scale;
                                y += t * t;
                            }
                        }

                        y = scale * std::sqrt(y);
                    }
                }

                return y;
            }

            //
            // Arguments    : int n
            //                const ::coder::array<double, 2U> &x
            //                int ix0
            // Return Type  : double
            //
            static double xnrm2(int n, const ::coder::array<double, 2U>& x, int ix0)
            {
                double scale;
                double y;
                int kend;
                y = 0.0;
                scale = 3.3121686421112381E-170;
                kend = (ix0 + n) - 1;
                for (int k = ix0; k <= kend; k++) {
                    double absxk;
                    absxk = std::abs(x[k - 1]);
                    if (absxk > scale) {
                        double t;
                        t = scale / absxk;
                        y = y * t * t + 1.0;
                        scale = absxk;
                    }
                    else {
                        double t;
                        t = absxk / scale;
                        y += t * t;
                    }
                }

                return scale * std::sqrt(y);
            }

            //
            // Arguments    : int n
            //                const double x[6]
            //                int ix0
            // Return Type  : double
            //
            static double xnrm2(int n, const double x[6], int ix0)
            {
                double scale;
                double y;
                int kend;
                y = 0.0;
                scale = 3.3121686421112381E-170;
                kend = (ix0 + n) - 1;
                for (int k = ix0; k <= kend; k++) {
                    double absxk;
                    absxk = std::abs(x[k - 1]);
                    if (absxk > scale) {
                        double t;
                        t = scale / absxk;
                        y = y * t * t + 1.0;
                        scale = absxk;
                    }
                    else {
                        double t;
                        t = absxk / scale;
                        y += t * t;
                    }
                }

                return scale * std::sqrt(y);
            }

            static void xgemv(int n, const ::coder::array<double, 2U>& A, const double x[6], ::
                coder::array<double, 1U>& y)
            {
                double c;
                int iac;
                if (n != 0) {
                    int i;
                    int iy;
                    for (iy = 0; iy < n; iy++) {
                        y[iy] = 0.0;
                    }

                    iy = 0;
                    i = 6 * (n - 1) + 1;
                    for (iac = 1; iac <= i; iac += 6) {
                        int i1;
                        int ix;
                        ix = 0;
                        c = 0.0;
                        i1 = iac + 5;
                        for (int ia = iac; ia <= i1; ia++) {
                            c += A[ia - 1] * x[ix];
                            ix++;
                        }

                        y[iy] = y[iy] + c;
                        iy++;
                    }
                }
            }

            static double b_xdotc(int n, const double x_data[], int ix0, const double y_data[],
                int iy0)
            {
                double d;
                int ix;
                int iy;
                ix = ix0;
                iy = iy0;
                d = 0.0;
                for (int k = 0; k < n; k++) {
                    d += x_data[ix - 1] * y_data[iy - 1];
                    ix++;
                    iy++;
                }

                return d;
            }

            //
            // Arguments    : int n
            //                const double x[36]
            //                int ix0
            //                const double y[36]
            //                int iy0
            // Return Type  : double
            //
            static double xdotc(int n, const double x[36], int ix0, const double y[36], int
                iy0)
            {
                double d;
                int ix;
                int iy;
                ix = ix0;
                iy = iy0;
                d = 0.0;
                for (int k = 0; k < n; k++) {
                    d += x[ix - 1] * y[iy - 1];
                    ix++;
                    iy++;
                }

                return d;
            }
            static void b_xaxpy(int n, double a, const double x[6], int ix0, double y_data[],
                int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y_data[iy] += a * x[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                int ix0
            //                double y_data[]
            //                int iy0
            // Return Type  : void
            //
            static void b_xaxpy(int n, double a, int ix0, double y_data[], int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y_data[iy] += a * y_data[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                int ix0
            //                ::coder::array<double, 2U> &y
            //                int iy0
            // Return Type  : void
            //
            static void b_xaxpy(int n, double a, int ix0, ::coder::array<double, 2U>& y, int
                iy0)
            {
                if ((n >= 1) && (!(a == 0.0))) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] = y[iy] + a * y[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                const double x_data[]
            //                int ix0
            //                double y[6]
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, const double x_data[], int ix0, double y[6],
                int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] += a * x_data[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                int ix0
            //                ::coder::array<double, 2U> &y
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, int ix0, ::coder::array<double, 2U>& y, int
                iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] = y[iy] + a * y[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                const ::coder::array<double, 2U> &x
            //                int ix0
            //                ::coder::array<double, 1U> &y
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, const ::coder::array<double, 2U>& x, int ix0, ::
                coder::array<double, 1U>& y, int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] = y[iy] + a * x[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                const ::coder::array<double, 1U> &x
            //                int ix0
            //                ::coder::array<double, 2U> &y
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, const ::coder::array<double, 1U>& x, int ix0, ::
                coder::array<double, 2U>& y, int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] = y[iy] + a * x[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                int ix0
            //                double y[36]
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, int ix0, double y[36], int iy0)
            {
                if (!(a == 0.0)) {
                    int i;
                    int ix;
                    int iy;
                    ix = ix0 - 1;
                    iy = iy0 - 1;
                    i = n - 1;
                    for (int k = 0; k <= i; k++) {
                        y[iy] += a * y[ix];
                        ix++;
                        iy++;
                    }
                }
            }
        }
        namespace reflapack
        {
            static double xzlarfg(int n, double* alpha1, ::coder::array<double, 2U>& x, int
                ix0)
            {
                double beta1;
                double tau;
                tau = 0.0;
                if (n > 0) {
                    double xnorm;
                    xnorm = blas::b_xnrm2(n - 1, x, ix0);
                    if (xnorm != 0.0) {
                        beta1 = rt_hypotd_snf(*alpha1, xnorm);
                        if (*alpha1 >= 0.0) {
                            beta1 = -beta1;
                        }

                        if (std::abs(beta1) < 1.0020841800044864E-292) {
                            int i;
                            int k;
                            int knt;
                            knt = -1;
                            i = (ix0 + n) - 2;
                            do {
                                knt++;
                                for (k = ix0; k <= i; k++) {
                                    x[k - 1] = 9.9792015476736E+291 * x[k - 1];
                                }

                                beta1 *= 9.9792015476736E+291;
                                *alpha1 *= 9.9792015476736E+291;
                            } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

                            beta1 = rt_hypotd_snf(*alpha1, blas::b_xnrm2(n - 1, x, ix0));
                            if (*alpha1 >= 0.0) {
                                beta1 = -beta1;
                            }

                            tau = (beta1 - *alpha1) / beta1;
                            xnorm = 1.0 / (*alpha1 - beta1);
                            for (k = ix0; k <= i; k++) {
                                x[k - 1] = xnorm * x[k - 1];
                            }

                            for (k = 0; k <= knt; k++) {
                                beta1 *= 1.0020841800044864E-292;
                            }

                            *alpha1 = beta1;
                        }
                        else {
                            int i;
                            tau = (beta1 - *alpha1) / beta1;
                            xnorm = 1.0 / (*alpha1 - beta1);
                            i = (ix0 + n) - 2;
                            for (int k = ix0; k <= i; k++) {
                                x[k - 1] = xnorm * x[k - 1];
                            }

                            *alpha1 = beta1;
                        }
                    }
                }

                return tau;
            }

            static void xzlarf(int m, int n, int iv0, double tau, ::coder::array<double, 2U>
                & C, int ic0, int ldc, ::coder::array<double, 1U>& work)
            {
                double c;
                int i;
                int ia;
                int iac;
                int lastc;
                int lastv;
                if (tau != 0.0) {
                    boolean_T exitg2;
                    lastv = m;
                    i = iv0 + m;
                    while ((lastv > 0) && (C[i - 2] == 0.0)) {
                        lastv--;
                        i--;
                    }

                    lastc = n - 1;
                    exitg2 = false;
                    while ((!exitg2) && (lastc + 1 > 0)) {
                        int exitg1;
                        i = ic0 + lastc * ldc;
                        ia = i;
                        do {
                            exitg1 = 0;
                            if (ia <= (i + lastv) - 1) {
                                if (C[ia - 1] != 0.0) {
                                    exitg1 = 1;
                                }
                                else {
                                    ia++;
                                }
                            }
                            else {
                                lastc--;
                                exitg1 = 2;
                            }
                        } while (exitg1 == 0);

                        if (exitg1 == 1) {
                            exitg2 = true;
                        }
                    }
                }
                else {
                    lastv = 0;
                    lastc = -1;
                }

                if (lastv > 0) {
                    int b_i;
                    int ix;
                    int jy;
                    if (lastc + 1 != 0) {
                        for (jy = 0; jy <= lastc; jy++) {
                            work[jy] = 0.0;
                        }

                        jy = 0;
                        b_i = ic0 + ldc * lastc;
                        for (iac = ic0; ldc < 0 ? iac >= b_i : iac <= b_i; iac += ldc) {
                            ix = iv0;
                            c = 0.0;
                            i = (iac + lastv) - 1;
                            for (ia = iac; ia <= i; ia++) {
                                c += C[ia - 1] * C[ix - 1];
                                ix++;
                            }

                            work[jy] = work[jy] + c;
                            jy++;
                        }
                    }

                    if (!(-tau == 0.0)) {
                        i = ic0;
                        jy = 0;
                        for (iac = 0; iac <= lastc; iac++) {
                            if (work[jy] != 0.0) {
                                c = work[jy] * -tau;
                                ix = iv0;
                                b_i = lastv + i;
                                for (ia = i; ia < b_i; ia++) {
                                    C[ia - 1] = C[ia - 1] + C[ix - 1] * c;
                                    ix++;
                                }
                            }

                            jy++;
                            i += ldc;
                        }
                    }
                }
            }
        }

        static void svd(const double A_data[], const int A_size[2], double U_data[], int
            U_size[2], double s_data[], int s_size[1], double V_data[], int
            V_size[2])
        {
            array<double, 2U> x;
            double Vf_data[36];
            double b_A_data[36];
            double b_s_data[6];
            double e_data[6];
            double work[6];
            double nrm;
            double scale;
            double sqds;
            double ztest;
            int Vf_size_idx_0;
            int Vf_size_idx_1;
            int i;
            int k;
            int loop_ub;
            int p;
            loop_ub = A_size[0] * A_size[1];
            if (0 <= loop_ub - 1) {
                std::memcpy(&b_A_data[0], &A_data[0], loop_ub * sizeof(double));
            }

            p = A_size[1];
            loop_ub = static_cast<signed char>(A_size[1]);
            if (0 <= loop_ub - 1) {
                std::memset(&b_s_data[0], 0, loop_ub * sizeof(double));
            }

            loop_ub = static_cast<signed char>(A_size[1]);
            if (0 <= loop_ub - 1) {
                std::memset(&e_data[0], 0, loop_ub * sizeof(double));
            }

            for (i = 0; i < 6; i++) {
                work[i] = 0.0;
            }

            U_size[0] = 6;
            U_size[1] = static_cast<signed char>(A_size[1]);
            loop_ub = 6 * static_cast<signed char>(A_size[1]);
            if (0 <= loop_ub - 1) {
                std::memset(&U_data[0], 0, loop_ub * sizeof(double));
            }

            Vf_size_idx_0 = static_cast<signed char>(A_size[1]);
            Vf_size_idx_1 = static_cast<signed char>(A_size[1]);
            loop_ub = static_cast<signed char>(A_size[1]) * static_cast<signed char>
                (A_size[1]);
            if (0 <= loop_ub - 1) {
                std::memset(&Vf_data[0], 0, loop_ub * sizeof(double));
            }

            if (A_size[1] == 0) {
                for (int iy = 0; iy < p; iy++) {
                    U_data[iy + 6 * iy] = 1.0;
                }
            }
            else {
                double rt;
                double snorm;
                int b_i;
                int iy;
                int jj;
                int m;
                int nct;
                int nctp1;
                int q;
                int qp1;
                int qq;
                int y;
                if (A_size[1] > 2) {
                    y = A_size[1] - 2;
                }
                else {
                    y = 0;
                }

                nct = A_size[1];
                if (5 < nct) {
                    nct = 5;
                }

                nctp1 = nct + 1;
                if (nct > y) {
                    b_i = nct;
                }
                else {
                    b_i = y;
                }

                for (q = 0; q < b_i; q++) {
                    boolean_T apply_transform;
                    qp1 = q + 2;
                    qq = (q + 6 * q) + 1;
                    apply_transform = false;
                    if (q + 1 <= nct) {
                        nrm = blas::b_xnrm2(6 - q, b_A_data, qq);
                        if (nrm > 0.0) {
                            apply_transform = true;
                            if (b_A_data[qq - 1] < 0.0) {
                                ztest = -nrm;
                                b_s_data[q] = -nrm;
                            }
                            else {
                                ztest = nrm;
                                b_s_data[q] = nrm;
                            }

                            if (std::abs(ztest) >= 1.0020841800044864E-292) {
                                nrm = 1.0 / ztest;
                                i = (qq - q) + 5;
                                for (k = qq; k <= i; k++) {
                                    b_A_data[k - 1] *= nrm;
                                }
                            }
                            else {
                                i = (qq - q) + 5;
                                for (k = qq; k <= i; k++) {
                                    b_A_data[k - 1] /= b_s_data[q];
                                }
                            }

                            b_A_data[qq - 1]++;
                            b_s_data[q] = -b_s_data[q];
                        }
                        else {
                            b_s_data[q] = 0.0;
                        }
                    }

                    for (jj = qp1; jj <= p; jj++) {
                        i = q + 6 * (jj - 1);
                        if (apply_transform) {
                            blas::b_xaxpy(6 - q, -(blas::b_xdotc(6 - q, b_A_data, qq, b_A_data,
                                i + 1) / b_A_data[q + 6 * q]), qq, b_A_data, i + 1);
                        }

                        e_data[jj - 1] = b_A_data[i];
                    }

                    if (q + 1 <= nct) {
                        for (iy = q + 1; iy < 7; iy++) {
                            i = (iy + 6 * q) - 1;
                            U_data[i] = b_A_data[i];
                        }
                    }

                    if (q + 1 <= y) {
                        qq = p - q;
                        nrm = blas::c_xnrm2(qq - 1, e_data, q + 2);
                        if (nrm == 0.0) {
                            e_data[q] = 0.0;
                        }
                        else {
                            if (e_data[q + 1] < 0.0) {
                                e_data[q] = -nrm;
                            }
                            else {
                                e_data[q] = nrm;
                            }

                            nrm = e_data[q];
                            if (std::abs(e_data[q]) >= 1.0020841800044864E-292) {
                                nrm = 1.0 / e_data[q];
                                i = q + qq;
                                for (k = qp1; k <= i; k++) {
                                    e_data[k - 1] *= nrm;
                                }
                            }
                            else {
                                i = q + qq;
                                for (k = qp1; k <= i; k++) {
                                    e_data[k - 1] /= nrm;
                                }
                            }

                            e_data[q + 1]++;
                            e_data[q] = -e_data[q];
                            for (iy = qp1; iy < 7; iy++) {
                                work[iy - 1] = 0.0;
                            }

                            for (jj = qp1; jj <= p; jj++) {
                                blas::xaxpy(5 - q, e_data[jj - 1], b_A_data, (q + 6 * (jj - 1))
                                    + 2, work, q + 2);
                            }

                            for (jj = qp1; jj <= p; jj++) {
                                blas::b_xaxpy(5 - q, -e_data[jj - 1] / e_data[q + 1], work, q +
                                    2, b_A_data, (q + 6 * (jj - 1)) + 2);
                            }
                        }

                        for (iy = qp1; iy <= p; iy++) {
                            Vf_data[(iy + Vf_size_idx_0 * q) - 1] = e_data[iy - 1];
                        }
                    }
                }

                m = A_size[1] - 2;
                if (nct < A_size[1]) {
                    b_s_data[nct] = b_A_data[nct + 6 * nct];
                }

                if (y + 1 < A_size[1]) {
                    e_data[y] = b_A_data[y + 6 * (A_size[1] - 1)];
                }

                e_data[A_size[1] - 1] = 0.0;
                if (nct + 1 <= A_size[1]) {
                    for (jj = nctp1; jj <= p; jj++) {
                        for (iy = 0; iy < 6; iy++) {
                            U_data[iy + 6 * (jj - 1)] = 0.0;
                        }

                        U_data[(jj + 6 * (jj - 1)) - 1] = 1.0;
                    }
                }

                for (q = nct; q >= 1; q--) {
                    qp1 = q + 1;
                    nctp1 = 6 * (q - 1);
                    qq = (q + nctp1) - 1;
                    if (b_s_data[q - 1] != 0.0) {
                        for (jj = qp1; jj <= p; jj++) {
                            i = q + 6 * (jj - 1);
                            blas::b_xaxpy(7 - q, -(blas::b_xdotc(7 - q, U_data, qq + 1, U_data,
                                i) / U_data[qq]), qq + 1, U_data, i);
                        }

                        for (iy = q; iy < 7; iy++) {
                            i = (iy + nctp1) - 1;
                            U_data[i] = -U_data[i];
                        }

                        U_data[qq]++;
                        for (iy = 0; iy <= q - 2; iy++) {
                            U_data[iy + nctp1] = 0.0;
                        }
                    }
                    else {
                        for (iy = 0; iy < 6; iy++) {
                            U_data[iy + nctp1] = 0.0;
                        }

                        U_data[qq] = 1.0;
                    }
                }

                for (q = p; q >= 1; q--) {
                    if ((q <= y) && (e_data[q - 1] != 0.0)) {
                        qp1 = q + 1;
                        qq = p - q;
                        i = q + p * (q - 1);
                        for (jj = qp1; jj <= p; jj++) {
                            nctp1 = q + p * (jj - 1);
                            nrm = 0.0;
                            if (qq >= 1) {
                                nct = i;
                                iy = nctp1;
                                for (k = 0; k < qq; k++) {
                                    nrm += Vf_data[nct] * Vf_data[iy];
                                    nct++;
                                    iy++;
                                }
                            }

                            nrm = -(nrm / Vf_data[i]);
                            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                x[b_i] = Vf_data[b_i];
                            }

                            blas::b_xaxpy(qq, nrm, i + 1, x, nctp1 + 1);
                            Vf_size_idx_0 = x.size(0);
                            Vf_size_idx_1 = x.size(1);
                            loop_ub = x.size(0) * x.size(1);
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                Vf_data[b_i] = x[b_i];
                            }
                        }
                    }

                    for (iy = 0; iy < p; iy++) {
                        Vf_data[iy + Vf_size_idx_0 * (q - 1)] = 0.0;
                    }

                    Vf_data[(q + Vf_size_idx_0 * (q - 1)) - 1] = 1.0;
                }

                qq = 0;
                snorm = 0.0;
                for (q = 0; q < p; q++) {
                    ztest = b_s_data[q];
                    if (ztest != 0.0) {
                        rt = std::abs(ztest);
                        nrm = ztest / rt;
                        b_s_data[q] = rt;
                        if (q + 1 < p) {
                            e_data[q] /= nrm;
                        }

                        i = 6 * q;
                        b_i = i + 6;
                        for (k = i + 1; k <= b_i; k++) {
                            U_data[k - 1] *= nrm;
                        }
                    }

                    if (q + 1 < p) {
                        ztest = e_data[q];
                        if (ztest != 0.0) {
                            rt = std::abs(ztest);
                            nrm = rt / ztest;
                            e_data[q] = rt;
                            b_s_data[q + 1] *= nrm;
                            i = p * (q + 1);
                            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                x[b_i] = Vf_data[b_i];
                            }

                            b_i = i + p;
                            for (k = i + 1; k <= b_i; k++) {
                                x[k - 1] = nrm * x[k - 1];
                            }

                            Vf_size_idx_0 = x.size(0);
                            Vf_size_idx_1 = x.size(1);
                            loop_ub = x.size(0) * x.size(1);
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                Vf_data[b_i] = x[b_i];
                            }
                        }
                    }

                    nrm = std::abs(b_s_data[q]);
                    ztest = std::abs(e_data[q]);
                    if ((nrm > ztest) || rtIsNaN(ztest)) {
                        ztest = nrm;
                    }

                    if ((!(snorm > ztest)) && (!rtIsNaN(ztest))) {
                        snorm = ztest;
                    }
                }

                while ((m + 2 > 0) && (qq < 75)) {
                    iy = m;
                    int exitg1;
                    do {
                        exitg1 = 0;
                        q = iy + 1;
                        if (iy + 1 == 0) {
                            exitg1 = 1;
                        }
                        else {
                            nrm = std::abs(e_data[iy]);
                            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[iy]) + std::
                                abs(b_s_data[iy + 1]))) || (nrm <= 1.0020841800044864E-292) ||
                                ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                                e_data[iy] = 0.0;
                                exitg1 = 1;
                            }
                            else {
                                iy--;
                            }
                        }
                    } while (exitg1 == 0);

                    if (iy + 1 == m + 1) {
                        i = 4;
                    }
                    else {
                        boolean_T exitg2;
                        nctp1 = m + 2;
                        i = m + 2;
                        exitg2 = false;
                        while ((!exitg2) && (i >= iy + 1)) {
                            nctp1 = i;
                            if (i == iy + 1) {
                                exitg2 = true;
                            }
                            else {
                                nrm = 0.0;
                                if (i < m + 2) {
                                    nrm = std::abs(e_data[i - 1]);
                                }

                                if (i > iy + 2) {
                                    nrm += std::abs(e_data[i - 2]);
                                }

                                ztest = std::abs(b_s_data[i - 1]);
                                if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                                    1.0020841800044864E-292)) {
                                    b_s_data[i - 1] = 0.0;
                                    exitg2 = true;
                                }
                                else {
                                    i--;
                                }
                            }
                        }

                        if (nctp1 == iy + 1) {
                            i = 3;
                        }
                        else if (nctp1 == m + 2) {
                            i = 1;
                        }
                        else {
                            i = 2;
                            q = nctp1;
                        }
                    }

                    switch (i) {
                    case 1:
                    {
                        ztest = e_data[m];
                        e_data[m] = 0.0;
                        b_i = m + 1;
                        for (k = b_i; k >= q + 1; k--) {
                            blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
                            if (k > q + 1) {
                                double sm;
                                sm = e_data[k - 2];
                                ztest = -scale * sm;
                                e_data[k - 2] = sm * sqds;
                            }

                            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                            for (i = 0; i < loop_ub; i++) {
                                x[i] = Vf_data[i];
                            }

                            if (p >= 1) {
                                nct = p * (k - 1);
                                iy = p * (m + 1);
                                for (i = 0; i < p; i++) {
                                    rt = sqds * x[nct] + scale * x[iy];
                                    x[iy] = sqds * x[iy] - scale * x[nct];
                                    x[nct] = rt;
                                    iy++;
                                    nct++;
                                }
                            }

                            Vf_size_idx_0 = x.size(0);
                            Vf_size_idx_1 = x.size(1);
                            loop_ub = x.size(0) * x.size(1);
                            for (i = 0; i < loop_ub; i++) {
                                Vf_data[i] = x[i];
                            }
                        }
                    }
                    break;

                    case 2:
                    {
                        ztest = e_data[q - 1];
                        e_data[q - 1] = 0.0;
                        for (k = q + 1; k <= m + 2; k++) {
                            double sm;
                            blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
                            sm = e_data[k - 1];
                            ztest = -scale * sm;
                            e_data[k - 1] = sm * sqds;
                            blas::b_xrot(U_data, 6 * (k - 1) + 1, 6 * (q - 1) + 1, sqds,
                                scale);
                        }
                    }
                    break;

                    case 3:
                    {
                        double sm;
                        nctp1 = m + 1;
                        nrm = b_s_data[m + 1];
                        scale = std::abs(nrm);
                        ztest = std::abs(b_s_data[m]);
                        if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                            scale = ztest;
                        }

                        ztest = std::abs(e_data[m]);
                        if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                            scale = ztest;
                        }

                        ztest = std::abs(b_s_data[q]);
                        if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                            scale = ztest;
                        }

                        ztest = std::abs(e_data[q]);
                        if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                            scale = ztest;
                        }

                        sm = nrm / scale;
                        nrm = b_s_data[m] / scale;
                        ztest = e_data[m] / scale;
                        sqds = b_s_data[q] / scale;
                        rt = ((nrm + sm) * (nrm - sm) + ztest * ztest) / 2.0;
                        nrm = sm * ztest;
                        nrm *= nrm;
                        if ((rt != 0.0) || (nrm != 0.0)) {
                            ztest = std::sqrt(rt * rt + nrm);
                            if (rt < 0.0) {
                                ztest = -ztest;
                            }

                            ztest = nrm / (rt + ztest);
                        }
                        else {
                            ztest = 0.0;
                        }

                        ztest += (sqds + sm) * (sqds - sm);
                        nrm = sqds * (e_data[q] / scale);
                        for (k = q + 1; k <= nctp1; k++) {
                            blas::xrotg(&ztest, &nrm, &sqds, &scale);
                            if (k > q + 1) {
                                e_data[k - 2] = ztest;
                            }

                            nrm = e_data[k - 1];
                            sm = b_s_data[k - 1];
                            e_data[k - 1] = sqds * nrm - scale * sm;
                            ztest = scale * b_s_data[k];
                            b_s_data[k] *= sqds;
                            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                x[b_i] = Vf_data[b_i];
                            }

                            if (p >= 1) {
                                nct = p * (k - 1);
                                iy = p * k;
                                for (i = 0; i < p; i++) {
                                    rt = sqds * x[nct] + scale * x[iy];
                                    x[iy] = sqds * x[iy] - scale * x[nct];
                                    x[nct] = rt;
                                    iy++;
                                    nct++;
                                }
                            }

                            Vf_size_idx_0 = x.size(0);
                            Vf_size_idx_1 = x.size(1);
                            loop_ub = x.size(0) * x.size(1);
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                Vf_data[b_i] = x[b_i];
                            }

                            b_s_data[k - 1] = sqds * sm + scale * nrm;
                            blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
                            ztest = sqds * e_data[k - 1] + scale * b_s_data[k];
                            b_s_data[k] = -scale * e_data[k - 1] + sqds * b_s_data[k];
                            nrm = scale * e_data[k];
                            e_data[k] *= sqds;
                            blas::b_xrot(U_data, 6 * (k - 1) + 1, 6 * k + 1, sqds, scale);
                        }

                        e_data[m] = ztest;
                        qq++;
                    }
                    break;

                    default:
                        if (b_s_data[q] < 0.0) {
                            b_s_data[q] = -b_s_data[q];
                            i = p * q;
                            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                x[b_i] = Vf_data[b_i];
                            }

                            b_i = i + p;
                            for (k = i + 1; k <= b_i; k++) {
                                x[k - 1] = -x[k - 1];
                            }

                            Vf_size_idx_0 = x.size(0);
                            Vf_size_idx_1 = x.size(1);
                            loop_ub = x.size(0) * x.size(1);
                            for (b_i = 0; b_i < loop_ub; b_i++) {
                                Vf_data[b_i] = x[b_i];
                            }
                        }

                        qp1 = q + 1;
                        while ((q + 1 < p) && (b_s_data[q] < b_s_data[qp1])) {
                            rt = b_s_data[q];
                            b_s_data[q] = b_s_data[qp1];
                            b_s_data[qp1] = rt;
                            if (q + 1 < p) {
                                nct = p * q;
                                iy = p * (q + 1);
                                x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                                loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                                for (b_i = 0; b_i < loop_ub; b_i++) {
                                    x[b_i] = Vf_data[b_i];
                                }

                                for (k = 0; k < p; k++) {
                                    rt = x[nct];
                                    x[nct] = x[iy];
                                    x[iy] = rt;
                                    nct++;
                                    iy++;
                                }

                                Vf_size_idx_0 = x.size(0);
                                Vf_size_idx_1 = x.size(1);
                                loop_ub = x.size(0) * x.size(1);
                                for (b_i = 0; b_i < loop_ub; b_i++) {
                                    Vf_data[b_i] = x[b_i];
                                }
                            }

                            blas::b_xswap(U_data, 6 * q + 1, 6 * (q + 1) + 1);
                            q = qp1;
                            qp1++;
                        }

                        qq = 0;
                        m--;
                        break;
                    }
                }
            }

            s_size[0] = A_size[1];
            V_size[0] = static_cast<signed char>(A_size[1]);
            V_size[1] = static_cast<signed char>(A_size[1]);
            for (k = 0; k < p; k++) {
                s_data[k] = b_s_data[k];
                for (i = 0; i < p; i++) {
                    V_data[i + V_size[0] * k] = Vf_data[i + Vf_size_idx_0 * k];
                }
            }
        }

        //
        // Arguments    : const ::coder::array<double, 2U> &A
        //                ::coder::array<double, 2U> &U
        //                double s_data[]
        //                int s_size[1]
        //                double V_data[]
        //                int V_size[2]
        // Return Type  : void
        //
        static void svd(const ::coder::array<double, 2U>& A, ::coder::array<double, 2U>& U,
            double s_data[], int s_size[1], double V_data[], int V_size[2])
        {
            array<double, 2U> b_A;
            array<double, 1U> work;
            double Vf[36];
            double b_s_data[6];
            double e[6];
            double f;
            double nrm;
            double rt;
            double scale;
            double snorm;
            double sqds;
            double ztest;
            int i;
            int ix;
            int iy;
            int jj;
            int k;
            int loop_ub;
            int m;
            int n;
            int nmq;
            int q;
            int qjj;
            int qp1;
            int qp1jj;
            int qq;
            b_A.set_size(A.size(0), 6);
            loop_ub = A.size(0) * A.size(1);
            for (i = 0; i < loop_ub; i++) {
                b_A[i] = A[i];
            }

            n = A.size(0);
            for (nmq = 0; nmq < 6; nmq++) {
                b_s_data[nmq] = 0.0;
                e[nmq] = 0.0;
            }

            work.set_size(A.size(0));
            loop_ub = A.size(0);
            for (i = 0; i < loop_ub; i++) {
                work[i] = 0.0;
            }

            U.set_size(A.size(0), 6);
            loop_ub = A.size(0) * 6;
            for (i = 0; i < loop_ub; i++) {
                U[i] = 0.0;
            }

            std::memset(&Vf[0], 0, 36U * sizeof(double));
            for (q = 0; q < 6; q++) {
                boolean_T apply_transform;
                qp1 = q + 2;
                qq = (q + n * q) + 1;
                nmq = (n - q) - 1;
                apply_transform = false;
                nrm = blas::xnrm2(nmq + 1, b_A, qq);
                if (nrm > 0.0) {
                    apply_transform = true;
                    if (b_A[qq - 1] < 0.0) {
                        ztest = -nrm;
                        b_s_data[q] = -nrm;
                    }
                    else {
                        ztest = nrm;
                        b_s_data[q] = nrm;
                    }

                    if (std::abs(ztest) >= 1.0020841800044864E-292) {
                        nrm = 1.0 / ztest;
                        i = qq + nmq;
                        for (k = qq; k <= i; k++) {
                            b_A[k - 1] = nrm * b_A[k - 1];
                        }
                    }
                    else {
                        i = qq + nmq;
                        for (k = qq; k <= i; k++) {
                            b_A[k - 1] = b_A[k - 1] / b_s_data[q];
                        }
                    }

                    b_A[qq - 1] = b_A[qq - 1] + 1.0;
                    b_s_data[q] = -b_s_data[q];
                }
                else {
                    b_s_data[q] = 0.0;
                }

                for (jj = qp1; jj < 7; jj++) {
                    qjj = q + n * (jj - 1);
                    if (apply_transform) {
                        ix = qq;
                        iy = qjj;
                        nrm = 0.0;
                        for (k = 0; k <= nmq; k++) {
                            nrm += b_A[ix - 1] * b_A[iy];
                            ix++;
                            iy++;
                        }

                        nrm = -(nrm / b_A[q + b_A.size(0) * q]);
                        blas::xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
                    }

                    e[jj - 1] = b_A[qjj];
                }

                for (qjj = q + 1; qjj <= n; qjj++) {
                    U[(qjj + U.size(0) * q) - 1] = b_A[(qjj + b_A.size(0) * q) - 1];
                }

                if (q + 1 <= 4) {
                    nrm = blas::xnrm2(5 - q, e, q + 2);
                    if (nrm == 0.0) {
                        e[q] = 0.0;
                    }
                    else {
                        if (e[q + 1] < 0.0) {
                            e[q] = -nrm;
                        }
                        else {
                            e[q] = nrm;
                        }

                        nrm = e[q];
                        if (std::abs(e[q]) >= 1.0020841800044864E-292) {
                            nrm = 1.0 / e[q];
                            for (k = qp1; k < 7; k++) {
                                e[k - 1] *= nrm;
                            }
                        }
                        else {
                            for (k = qp1; k < 7; k++) {
                                e[k - 1] /= nrm;
                            }
                        }

                        e[q + 1]++;
                        e[q] = -e[q];
                        for (qjj = qp1; qjj <= n; qjj++) {
                            work[qjj - 1] = 0.0;
                        }

                        for (jj = qp1; jj < 7; jj++) {
                            blas::xaxpy(nmq, e[jj - 1], b_A, (q + n * (jj - 1)) + 2, work, q +
                                2);
                        }

                        for (jj = qp1; jj < 7; jj++) {
                            blas::xaxpy(nmq, -e[jj - 1] / e[q + 1], work, q + 2, b_A, (q + n *
                                (jj - 1)) + 2);
                        }
                    }

                    for (qjj = qp1; qjj < 7; qjj++) {
                        Vf[(qjj + 6 * q) - 1] = e[qjj - 1];
                    }
                }
            }

            m = 4;
            e[4] = b_A[b_A.size(0) * 5 + 4];
            e[5] = 0.0;
            for (q = 5; q >= 0; q--) {
                qp1 = q + 2;
                qp1jj = n - q;
                qq = q + n * q;
                if (b_s_data[q] != 0.0) {
                    for (jj = qp1; jj < 7; jj++) {
                        qjj = q + n * (jj - 1);
                        nrm = 0.0;
                        if (qp1jj >= 1) {
                            ix = qq;
                            iy = qjj;
                            for (k = 0; k < qp1jj; k++) {
                                nrm += U[ix] * U[iy];
                                ix++;
                                iy++;
                            }
                        }

                        nrm = -(nrm / U[qq]);
                        b_A.set_size(U.size(0), 6);
                        loop_ub = U.size(0) * U.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            b_A[i] = U[i];
                        }

                        blas::b_xaxpy(qp1jj, nrm, qq + 1, b_A, qjj + 1);
                        U.set_size(b_A.size(0), b_A.size(1));
                        loop_ub = b_A.size(0) * b_A.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            U[i] = b_A[i];
                        }
                    }

                    for (qjj = q + 1; qjj <= n; qjj++) {
                        U[(qjj + U.size(0) * q) - 1] = -U[(qjj + U.size(0) * q) - 1];
                    }

                    U[qq] = U[qq] + 1.0;
                    for (qjj = 0; qjj < q; qjj++) {
                        U[qjj + U.size(0) * q] = 0.0;
                    }
                }
                else {
                    for (qjj = 0; qjj < n; qjj++) {
                        U[qjj + U.size(0) * q] = 0.0;
                    }

                    U[qq] = 1.0;
                }
            }

            for (q = 5; q >= 0; q--) {
                if ((q + 1 <= 4) && (e[q] != 0.0)) {
                    qp1 = q + 2;
                    nmq = (q + 6 * q) + 2;
                    for (jj = qp1; jj < 7; jj++) {
                        qp1jj = (q + 6 * (jj - 1)) + 2;
                        blas::xaxpy(5 - q, -(blas::xdotc(5 - q, Vf, nmq, Vf, qp1jj) / Vf[nmq
                            - 1]), nmq, Vf, qp1jj);
                    }
                }

                for (qjj = 0; qjj < 6; qjj++) {
                    Vf[qjj + 6 * q] = 0.0;
                }

                Vf[q + 6 * q] = 1.0;
            }

            qq = 0;
            snorm = 0.0;
            for (q = 0; q < 6; q++) {
                ztest = b_s_data[q];
                if (ztest != 0.0) {
                    rt = std::abs(ztest);
                    nrm = ztest / rt;
                    b_s_data[q] = rt;
                    if (q + 1 < 6) {
                        e[q] /= nrm;
                    }

                    nmq = n * q;
                    b_A.set_size(U.size(0), 6);
                    loop_ub = U.size(0) * U.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        b_A[i] = U[i];
                    }

                    i = nmq + n;
                    for (k = nmq + 1; k <= i; k++) {
                        b_A[k - 1] = nrm * b_A[k - 1];
                    }

                    U.set_size(b_A.size(0), b_A.size(1));
                    loop_ub = b_A.size(0) * b_A.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        U[i] = b_A[i];
                    }
                }

                if (q + 1 < 6) {
                    ztest = e[q];
                    if (ztest != 0.0) {
                        rt = std::abs(ztest);
                        nrm = rt / ztest;
                        e[q] = rt;
                        b_s_data[q + 1] *= nrm;
                        nmq = 6 * (q + 1);
                        i = nmq + 6;
                        for (k = nmq + 1; k <= i; k++) {
                            Vf[k - 1] *= nrm;
                        }
                    }
                }

                nrm = std::abs(b_s_data[q]);
                ztest = std::abs(e[q]);
                if ((nrm > ztest) || rtIsNaN(ztest)) {
                    ztest = nrm;
                }

                if ((!(snorm > ztest)) && (!rtIsNaN(ztest))) {
                    snorm = ztest;
                }
            }

            while ((m + 2 > 0) && (qq < 75)) {
                qjj = m;
                int exitg1;
                do {
                    exitg1 = 0;
                    q = qjj + 1;
                    if (qjj + 1 == 0) {
                        exitg1 = 1;
                    }
                    else {
                        nrm = std::abs(e[qjj]);
                        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[qjj]) + std::
                            abs(b_s_data[qjj + 1]))) || (nrm <= 1.0020841800044864E-292) ||
                            ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                            e[qjj] = 0.0;
                            exitg1 = 1;
                        }
                        else {
                            qjj--;
                        }
                    }
                } while (exitg1 == 0);

                if (qjj + 1 == m + 1) {
                    nmq = 4;
                }
                else {
                    boolean_T exitg2;
                    qp1jj = m + 2;
                    nmq = m + 2;
                    exitg2 = false;
                    while ((!exitg2) && (nmq >= qjj + 1)) {
                        qp1jj = nmq;
                        if (nmq == qjj + 1) {
                            exitg2 = true;
                        }
                        else {
                            nrm = 0.0;
                            if (nmq < m + 2) {
                                nrm = std::abs(e[nmq - 1]);
                            }

                            if (nmq > qjj + 2) {
                                nrm += std::abs(e[nmq - 2]);
                            }

                            ztest = std::abs(b_s_data[nmq - 1]);
                            if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                                1.0020841800044864E-292)) {
                                b_s_data[nmq - 1] = 0.0;
                                exitg2 = true;
                            }
                            else {
                                nmq--;
                            }
                        }
                    }

                    if (qp1jj == qjj + 1) {
                        nmq = 3;
                    }
                    else if (qp1jj == m + 2) {
                        nmq = 1;
                    }
                    else {
                        nmq = 2;
                        q = qp1jj;
                    }
                }

                switch (nmq) {
                case 1:
                    f = e[m];
                    e[m] = 0.0;
                    i = m + 1;
                    for (k = i; k >= q + 1; k--) {
                        blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
                        if (k > q + 1) {
                            rt = e[k - 2];
                            f = -scale * rt;
                            e[k - 2] = rt * sqds;
                        }

                        blas::xrot(Vf, 6 * (k - 1) + 1, 6 * (m + 1) + 1, sqds, scale);
                    }
                    break;

                case 2:
                    f = e[q - 1];
                    e[q - 1] = 0.0;
                    for (k = q + 1; k <= m + 2; k++) {
                        blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
                        rt = e[k - 1];
                        f = -scale * rt;
                        e[k - 1] = rt * sqds;
                        b_A.set_size(U.size(0), 6);
                        loop_ub = U.size(0) * U.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            b_A[i] = U[i];
                        }

                        if (n >= 1) {
                            ix = n * (k - 1);
                            iy = n * (q - 1);
                            for (qp1jj = 0; qp1jj < n; qp1jj++) {
                                nrm = sqds * b_A[ix] + scale * b_A[iy];
                                b_A[iy] = sqds * b_A[iy] - scale * b_A[ix];
                                b_A[ix] = nrm;
                                iy++;
                                ix++;
                            }
                        }

                        U.set_size(b_A.size(0), b_A.size(1));
                        loop_ub = b_A.size(0) * b_A.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            U[i] = b_A[i];
                        }
                    }
                    break;

                case 3:
                    nmq = m + 1;
                    nrm = b_s_data[m + 1];
                    scale = std::abs(nrm);
                    ztest = std::abs(b_s_data[m]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(e[m]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(b_s_data[q]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(e[q]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    f = nrm / scale;
                    nrm = b_s_data[m] / scale;
                    ztest = e[m] / scale;
                    sqds = b_s_data[q] / scale;
                    rt = ((nrm + f) * (nrm - f) + ztest * ztest) / 2.0;
                    nrm = f * ztest;
                    nrm *= nrm;
                    if ((rt != 0.0) || (nrm != 0.0)) {
                        ztest = std::sqrt(rt * rt + nrm);
                        if (rt < 0.0) {
                            ztest = -ztest;
                        }

                        ztest = nrm / (rt + ztest);
                    }
                    else {
                        ztest = 0.0;
                    }

                    f = (sqds + f) * (sqds - f) + ztest;
                    ztest = sqds * (e[q] / scale);
                    for (k = q + 1; k <= nmq; k++) {
                        blas::xrotg(&f, &ztest, &sqds, &scale);
                        if (k > q + 1) {
                            e[k - 2] = f;
                        }

                        nrm = e[k - 1];
                        rt = b_s_data[k - 1];
                        e[k - 1] = sqds * nrm - scale * rt;
                        ztest = scale * b_s_data[k];
                        b_s_data[k] *= sqds;
                        blas::xrot(Vf, 6 * (k - 1) + 1, 6 * k + 1, sqds, scale);
                        b_s_data[k - 1] = sqds * rt + scale * nrm;
                        blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
                        f = sqds * e[k - 1] + scale * b_s_data[k];
                        b_s_data[k] = -scale * e[k - 1] + sqds * b_s_data[k];
                        ztest = scale * e[k];
                        e[k] *= sqds;
                        b_A.set_size(U.size(0), 6);
                        loop_ub = U.size(0) * U.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            b_A[i] = U[i];
                        }

                        if (n >= 1) {
                            ix = n * (k - 1);
                            iy = n * k;
                            for (qp1jj = 0; qp1jj < n; qp1jj++) {
                                nrm = sqds * b_A[ix] + scale * b_A[iy];
                                b_A[iy] = sqds * b_A[iy] - scale * b_A[ix];
                                b_A[ix] = nrm;
                                iy++;
                                ix++;
                            }
                        }

                        U.set_size(b_A.size(0), b_A.size(1));
                        loop_ub = b_A.size(0) * b_A.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            U[i] = b_A[i];
                        }
                    }

                    e[m] = f;
                    qq++;
                    break;

                default:
                    if (b_s_data[q] < 0.0) {
                        b_s_data[q] = -b_s_data[q];
                        nmq = 6 * q;
                        i = nmq + 6;
                        for (k = nmq + 1; k <= i; k++) {
                            Vf[k - 1] = -Vf[k - 1];
                        }
                    }

                    qp1 = q + 1;
                    while ((q + 1 < 6) && (b_s_data[q] < b_s_data[qp1])) {
                        rt = b_s_data[q];
                        b_s_data[q] = b_s_data[qp1];
                        b_s_data[qp1] = rt;
                        blas::xswap(Vf, 6 * q + 1, 6 * (q + 1) + 1);
                        ix = n * q;
                        iy = n * (q + 1);
                        b_A.set_size(U.size(0), 6);
                        loop_ub = U.size(0) * U.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            b_A[i] = U[i];
                        }

                        for (k = 0; k < n; k++) {
                            nrm = b_A[ix];
                            b_A[ix] = b_A[iy];
                            b_A[iy] = nrm;
                            ix++;
                            iy++;
                        }

                        U.set_size(b_A.size(0), b_A.size(1));
                        loop_ub = b_A.size(0) * b_A.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            U[i] = b_A[i];
                        }

                        q = qp1;
                        qp1++;
                    }

                    qq = 0;
                    m--;
                    break;
                }
            }

            s_size[0] = 6;
            V_size[0] = 6;
            V_size[1] = 6;
            for (k = 0; k < 6; k++) {
                s_data[k] = b_s_data[k];
                for (nmq = 0; nmq < 6; nmq++) {
                    i = nmq + 6 * k;
                    V_data[i] = Vf[i];
                }
            }
        }
    }

    static double b_norm(const ::coder::array<double, 1U>& x)
    {
        double y;
        if (x.size(0) == 0) {
            y = 0.0;
        }
        else {
            y = 0.0;
            if (x.size(0) == 1) {
                y = std::abs(x[0]);
            }
            else {
                double scale;
                int kend;
                scale = 3.3121686421112381E-170;
                kend = x.size(0);
                for (int k = 0; k < kend; k++) {
                    double absxk;
                    absxk = std::abs(x[k]);
                    if (absxk > scale) {
                        double t;
                        t = scale / absxk;
                        y = y * t * t + 1.0;
                        scale = absxk;
                    }
                    else {
                        double t;
                        t = absxk / scale;
                        y += t * t;
                    }
                }

                y = scale * std::sqrt(y);
            }
        }

        return y;
    }

    //
    // Arguments    : const double x[3]
    // Return Type  : double
    //
    static double b_norm(const double x[3])
    {
        double absxk;
        double scale;
        double t;
        double y;
        scale = 3.3121686421112381E-170;
        absxk = std::abs(x[0]);
        if (absxk > 3.3121686421112381E-170) {
            y = 1.0;
            scale = absxk;
        }
        else {
            t = absxk / 3.3121686421112381E-170;
            y = t * t;
        }

        absxk = std::abs(x[1]);
        if (absxk > scale) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            y += t * t;
        }

        absxk = std::abs(x[2]);
        if (absxk > scale) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            y += t * t;
        }

        return scale * std::sqrt(y);
    }
    class anonymous_function
    {
    public:
        cell_1 tunableEnvironment;
    };

    static void b_mod(const ::coder::array<double, 2U>& x, ::coder::array<double, 2U>& r)
    {
        double b_r;
        int nx;
        r.set_size(1, x.size(1));
        nx = x.size(1);
        for (int k = 0; k < nx; k++) {
            double d;
            d = x[k];
            if (rtIsNaN(d) || rtIsInf(d)) {
                b_r = rtNaN;
            }
            else if (d == 0.0) {
                b_r = 0.0;
            }
            else {
                boolean_T rEQ0;
                b_r = std::fmod(d, 6.2831853071795862);
                rEQ0 = (b_r == 0.0);
                if (!rEQ0) {
                    double q;
                    q = std::abs(d / 6.2831853071795862);
                    rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 *
                        q);
                }

                if (rEQ0) {
                    b_r = 0.0;
                }
                else {
                    if (d < 0.0) {
                        b_r += 6.2831853071795862;
                    }
                }
            }

            r[k] = b_r;
        }
    }

    namespace optim
    {
        namespace coder
        {
            namespace levenbergMarquardt
            {
                static double projectBox(const ::coder::array<double, 2U>& x, const ::coder::
                    array<double, 1U>& dx)
                {
                    double dxInfNorm;
                    int n;
                    n = dx.size(0) - 1;
                    dxInfNorm = 0.0;
                    if (x.size(1) == 0) {
                        for (int i = 0; i <= n; i++) {
                            double u1;
                            u1 = std::abs(dx[i]);
                            if ((!(dxInfNorm > u1)) && (!rtIsNaN(u1))) {
                                dxInfNorm = u1;
                            }
                        }
                    }
                    else {
                        for (int i = 0; i <= n; i++) {
                            double u1;
                            u1 = std::abs(dx[i]);
                            if ((!(dxInfNorm > u1)) && (!rtIsNaN(u1))) {
                                dxInfNorm = u1;
                            }
                        }
                    }

                    return dxInfNorm;
                }
                static void linearLeastSquares(::coder::array<double, 2U>& lhs, ::coder::array<
                    double, 1U>& rhs, ::coder::array<double, 2U>& dx, int m, int n)
                {
                    array<double, 1U> tau;
                    array<double, 1U> vn1;
                    array<double, 1U> vn2;
                    array<double, 1U> work;
                    array<int, 1U> jpvt;
                    double temp;
                    int b_i;
                    int i;
                    int ii;
                    int ix;
                    int ma;
                    int minmana;
                    int minmn_tmp;
                    int nfxd;
                    int pvt;
                    jpvt.set_size(n);
                    for (i = 0; i < n; i++) {
                        jpvt[i] = 0;
                    }

                    ma = lhs.size(0);
                    pvt = lhs.size(0);
                    minmana = lhs.size(1);
                    if (pvt < minmana) {
                        minmana = pvt;
                    }

                    if (m < n) {
                        minmn_tmp = m;
                    }
                    else {
                        minmn_tmp = n;
                    }

                    tau.set_size(minmana);
                    for (i = 0; i < minmana; i++) {
                        tau[i] = 0.0;
                    }

                    if ((lhs.size(1) == 0) || (minmn_tmp < 1)) {
                        for (ii = 0; ii < n; ii++) {
                            jpvt[ii] = ii + 1;
                        }
                    }
                    else {
                        double d;
                        int iy;
                        int k;
                        int mmi;
                        nfxd = 0;
                        for (ii = 0; ii < n; ii++) {
                            if (jpvt[ii] != 0) {
                                nfxd++;
                                if (ii + 1 != nfxd) {
                                    ix = ii * ma;
                                    iy = (nfxd - 1) * ma;
                                    for (k = 0; k < m; k++) {
                                        temp = lhs[ix];
                                        lhs[ix] = lhs[iy];
                                        lhs[iy] = temp;
                                        ix++;
                                        iy++;
                                    }

                                    jpvt[ii] = jpvt[nfxd - 1];
                                    jpvt[nfxd - 1] = ii + 1;
                                }
                                else {
                                    jpvt[ii] = ii + 1;
                                }
                            }
                            else {
                                jpvt[ii] = ii + 1;
                            }
                        }

                        if (nfxd >= minmn_tmp) {
                            nfxd = minmn_tmp;
                        }

                        minmana = lhs.size(0);
                        work.set_size(lhs.size(1));
                        pvt = lhs.size(1);
                        for (i = 0; i < pvt; i++) {
                            work[i] = 0.0;
                        }

                        for (b_i = 0; b_i < nfxd; b_i++) {
                            ii = b_i * minmana + b_i;
                            mmi = m - b_i;
                            if (b_i + 1 < m) {
                                temp = lhs[ii];
                                d = internal::reflapack::xzlarfg(mmi, &temp, lhs, ii + 2);
                                tau[b_i] = d;
                                lhs[ii] = temp;
                            }
                            else {
                                d = 0.0;
                                tau[b_i] = 0.0;
                            }

                            if (b_i + 1 < n) {
                                temp = lhs[ii];
                                lhs[ii] = 1.0;
                                internal::reflapack::xzlarf(mmi, (n - b_i) - 1, ii + 1, d, lhs,
                                    (ii + minmana) + 1, minmana, work);
                                lhs[ii] = temp;
                            }
                        }

                        if (nfxd < minmn_tmp) {
                            ma = lhs.size(0);
                            work.set_size(lhs.size(1));
                            pvt = lhs.size(1);
                            for (i = 0; i < pvt; i++) {
                                work[i] = 0.0;
                            }

                            vn1.set_size(lhs.size(1));
                            pvt = lhs.size(1);
                            for (i = 0; i < pvt; i++) {
                                vn1[i] = 0.0;
                            }

                            vn2.set_size(lhs.size(1));
                            pvt = lhs.size(1);
                            for (i = 0; i < pvt; i++) {
                                vn2[i] = 0.0;
                            }

                            i = nfxd + 1;
                            for (ii = i; ii <= n; ii++) {
                                d = internal::blas::b_xnrm2(m - nfxd, lhs, (nfxd + (ii - 1) * ma)
                                    + 1);
                                vn1[ii - 1] = d;
                                vn2[ii - 1] = d;
                            }

                            i = nfxd + 1;
                            for (b_i = i; b_i <= minmn_tmp; b_i++) {
                                double s;
                                int ip1;
                                ip1 = b_i + 1;
                                iy = (b_i - 1) * ma;
                                ii = (iy + b_i) - 1;
                                nfxd = (n - b_i) + 1;
                                mmi = m - b_i;
                                if (nfxd < 1) {
                                    minmana = -2;
                                }
                                else {
                                    minmana = -1;
                                    if (nfxd > 1) {
                                        ix = b_i;
                                        temp = std::abs(vn1[b_i - 1]);
                                        for (k = 2; k <= nfxd; k++) {
                                            ix++;
                                            s = std::abs(vn1[ix - 1]);
                                            if (s > temp) {
                                                minmana = k - 2;
                                                temp = s;
                                            }
                                        }
                                    }
                                }

                                pvt = b_i + minmana;
                                if (pvt + 1 != b_i) {
                                    ix = pvt * ma;
                                    for (k = 0; k < m; k++) {
                                        temp = lhs[ix];
                                        lhs[ix] = lhs[iy];
                                        lhs[iy] = temp;
                                        ix++;
                                        iy++;
                                    }

                                    minmana = jpvt[pvt];
                                    jpvt[pvt] = jpvt[b_i - 1];
                                    jpvt[b_i - 1] = minmana;
                                    vn1[pvt] = vn1[b_i - 1];
                                    vn2[pvt] = vn2[b_i - 1];
                                }

                                if (b_i < m) {
                                    temp = lhs[ii];
                                    d = internal::reflapack::xzlarfg(mmi + 1, &temp, lhs, ii + 2);
                                    tau[b_i - 1] = d;
                                    lhs[ii] = temp;
                                }
                                else {
                                    d = 0.0;
                                    tau[b_i - 1] = 0.0;
                                }

                                if (b_i < n) {
                                    temp = lhs[ii];
                                    lhs[ii] = 1.0;
                                    internal::reflapack::xzlarf(mmi + 1, nfxd - 1, ii + 1, d, lhs,
                                        (ii + ma) + 1, ma, work);
                                    lhs[ii] = temp;
                                }

                                for (ii = ip1; ii <= n; ii++) {
                                    pvt = b_i + (ii - 1) * ma;
                                    d = vn1[ii - 1];
                                    if (d != 0.0) {
                                        temp = std::abs(lhs[pvt - 1]) / d;
                                        temp = 1.0 - temp * temp;
                                        if (temp < 0.0) {
                                            temp = 0.0;
                                        }

                                        s = d / vn2[ii - 1];
                                        s = temp * (s * s);
                                        if (s <= 1.4901161193847656E-8) {
                                            if (b_i < m) {
                                                d = internal::blas::b_xnrm2(mmi, lhs, pvt + 1);
                                                vn1[ii - 1] = d;
                                                vn2[ii - 1] = d;
                                            }
                                            else {
                                                vn1[ii - 1] = 0.0;
                                                vn2[ii - 1] = 0.0;
                                            }
                                        }
                                        else {
                                            vn1[ii - 1] = d * std::sqrt(temp);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    minmana = lhs.size(0);
                    pvt = lhs.size(0);
                    nfxd = lhs.size(1);
                    if (pvt < nfxd) {
                        nfxd = pvt;
                    }

                    for (ii = 0; ii < nfxd; ii++) {
                        if (tau[ii] != 0.0) {
                            temp = rhs[ii];
                            i = ii + 2;
                            for (b_i = i; b_i <= minmana; b_i++) {
                                temp += lhs[(b_i + lhs.size(0) * ii) - 1] * rhs[b_i - 1];
                            }

                            temp *= tau[ii];
                            if (temp != 0.0) {
                                rhs[ii] = rhs[ii] - temp;
                                i = ii + 2;
                                for (b_i = i; b_i <= minmana; b_i++) {
                                    rhs[b_i - 1] = rhs[b_i - 1] - lhs[(b_i + lhs.size(0) * ii) - 1]
                                        * temp;
                                }
                            }
                        }
                    }

                    if (1 > n) {
                        pvt = 0;
                    }
                    else {
                        pvt = n;
                    }

                    tau.set_size(pvt);
                    for (i = 0; i < pvt; i++) {
                        tau[i] = rhs[i];
                    }

                    if ((lhs.size(1) != 0) && (pvt != 0) && (n != 0)) {
                        for (ii = n; ii >= 1; ii--) {
                            pvt = (ii + (ii - 1) * m) - 1;
                            tau[ii - 1] = tau[ii - 1] / lhs[pvt];
                            for (b_i = 0; b_i <= ii - 2; b_i++) {
                                ix = (ii - b_i) - 2;
                                tau[ix] = tau[ix] - tau[ii - 1] * lhs[(pvt - b_i) - 1];
                            }
                        }
                    }

                    pvt = dx.size(1);
                    dx.set_size(1, pvt);
                    for (i = 0; i < pvt; i++) {
                        dx[i] = tau[i];
                    }

                    tau.set_size(jpvt.size(0));
                    pvt = jpvt.size(0);
                    for (i = 0; i < pvt; i++) {
                        tau[i] = dx[i];
                    }

                    pvt = tau.size(0);
                    for (i = 0; i < pvt; i++) {
                        dx[jpvt[i] - 1] = tau[i];
                    }
                }
                static void driver(const anonymous_function* fun, const ::coder::array<double,
                    2U>& x0, ::coder::array<double, 2U>& x, double* resnorm,
                    double fCurrent[6], double* exitflag, double
                    * output_iterations, double* output_funcCount, double
                    * output_stepsize, double* output_firstorderopt, char
                    output_algorithm[19], ::coder::array<double, 1U>
                    & lambda_lower, ::coder::array<double, 1U>& lambda_upper, ::
                    coder::array<double, 2U>& jacobian)
                {
                    static const char cv[19] = { 'l', 'e', 'v', 'e', 'n', 'b', 'e', 'r',
                      'g', '-', 'm', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't' };

                    array<double, 2U> augJacobian;
                    array<double, 2U> dx;
                    array<double, 2U> varargout_2;
                    array<double, 2U> xp;
                    array<double, 1U> b_x0;
                    array<double, 1U> gradf;
                    array<double, 1U> rhs;
                    array<double, 1U> unusedU1;
                    double fNew[6];
                    double varargout_1[6];
                    double absx;
                    double b_gamma;
                    double funDiff;
                    double normGradF;
                    double relFactor;
                    int b_exitflag;
                    int funcCount;
                    int i;
                    int iter;
                    int k;
                    int loop_ub;
                    int n;
                    int options_MaxFunctionEvaluations;
                    boolean_T exitg1;
                    boolean_T stepSuccessful;
                    for (i = 0; i < 19; i++) {
                        output_algorithm[i] = cv[i];
                    }

                    n = x0.size(1);
                    funcCount = 1;
                    gradf.set_size(x0.size(1));
                    dx.set_size(1, x0.size(1));
                    loop_ub = x0.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        dx[i] = rtInf;
                    }

                    funDiff = rtInf;
                    iter = 0;
                    options_MaxFunctionEvaluations = 200 * x0.size(1);
                    x.set_size(1, x0.size(1));
                    loop_ub = x0.size(0) * x0.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        x[i] = x0[i];
                    }

                    anon(fun->tunableEnvironment.f1, fun->tunableEnvironment.f2.dof,
                        fun->tunableEnvironment.f2.A, fun->tunableEnvironment.f2.M,
                        fun->tunableEnvironment.f2.ME, x0, varargout_1, jacobian);
                    for (i = 0; i < 6; i++) {
                        fCurrent[i] = varargout_1[i];
                    }

                    augJacobian.set_size((x0.size(1) + 6), x0.size(1));
                    loop_ub = jacobian.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        for (b_exitflag = 0; b_exitflag < 6; b_exitflag++) {
                            augJacobian[b_exitflag + augJacobian.size(0) * i] =
                                jacobian[b_exitflag + 6 * i];
                        }
                    }

                    rhs.set_size((x0.size(1) + 6));
                    *resnorm = 0.0;
                    for (k = 0; k < 6; k++) {
                        *resnorm += fCurrent[k] * fCurrent[k];
                    }

                    b_gamma = 0.01;
                    for (i = 0; i < n; i++) {
                        b_exitflag = (n + 6) * (i + 1) - n;
                        for (k = 0; k < n; k++) {
                            augJacobian[b_exitflag + k] = 0.0;
                        }

                        augJacobian[(i + augJacobian.size(0) * i) + 6] = 0.1;
                        loop_ub = (n + 6) * i;
                        b_exitflag = 6 * i;
                        for (k = 0; k < 6; k++) {
                            jacobian[b_exitflag + k] = augJacobian[loop_ub + k];
                        }
                    }

                    internal::blas::xgemv(x0.size(1), jacobian, fCurrent, gradf);
                    unusedU1.set_size(gradf.size(0));
                    loop_ub = gradf.size(0);
                    for (i = 0; i < loop_ub; i++) {
                        unusedU1[i] = -gradf[i];
                    }

                    projectBox(x0, unusedU1);
                    if (gradf.size(0) == 0) {
                        relFactor = 0.0;
                        normGradF = 0.0;
                    }
                    else {
                        relFactor = 0.0;
                        i = gradf.size(0);
                        for (k = 0; k < i; k++) {
                            absx = std::abs(gradf[k]);
                            if (rtIsNaN(absx) || (absx > relFactor)) {
                                relFactor = absx;
                            }
                        }

                        normGradF = 0.0;
                        i = gradf.size(0);
                        for (k = 0; k < i; k++) {
                            absx = std::abs(gradf[k]);
                            if (rtIsNaN(absx) || (absx > normGradF)) {
                                normGradF = absx;
                            }
                        }
                    }

                    if (!(relFactor > 1.4901161193847656E-8)) {
                        relFactor = 1.4901161193847656E-8;
                    }

                    stepSuccessful = true;
                    if (normGradF <= 1.0000000000000002E-14 * relFactor) {
                        b_exitflag = 1;
                    }
                    else if (1 >= options_MaxFunctionEvaluations) {
                        b_exitflag = 0;
                    }
                    else {
                        unusedU1.set_size(x0.size(1));
                        loop_ub = x0.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            unusedU1[i] = rtInf;
                        }

                        loop_ub = x0.size(1);
                        b_x0 = x0.reshape(loop_ub);
                        if (b_norm(unusedU1) < 1.0E-6 * (b_norm(b_x0) +
                            1.4901161193847656E-8)) {
                            b_exitflag = 4;
                        }
                        else {
                            b_exitflag = -5;
                        }
                    }

                    exitg1 = false;
                    while ((!exitg1) && (b_exitflag == -5)) {
                        boolean_T evalOK;
                        boolean_T guard1 = false;
                        for (k = 0; k < 6; k++) {
                            rhs[k] = -fCurrent[k];
                        }

                        for (k = 0; k < n; k++) {
                            rhs[k + 6] = 0.0;
                        }

                        linearLeastSquares(augJacobian, rhs, dx, n + 6, n);
                        xp.set_size(1, x.size(1));
                        loop_ub = x.size(0) * x.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            xp[i] = x[i] + dx[i];
                        }

                        anon(fun->tunableEnvironment.f1, fun->tunableEnvironment.f2.dof,
                            fun->tunableEnvironment.f2.A, fun->tunableEnvironment.f2.M,
                            fun->tunableEnvironment.f2.ME, xp, varargout_1, varargout_2);
                        loop_ub = varargout_2.size(1);
                        for (i = 0; i < loop_ub; i++) {
                            for (b_exitflag = 0; b_exitflag < 6; b_exitflag++) {
                                augJacobian[b_exitflag + augJacobian.size(0) * i] =
                                    varargout_2[b_exitflag + 6 * i];
                            }
                        }

                        normGradF = 0.0;
                        evalOK = true;
                        for (i = 0; i < 6; i++) {
                            absx = varargout_1[i];
                            fNew[i] = absx;
                            normGradF += absx * absx;
                            if ((!evalOK) || (rtIsInf(absx) || rtIsNaN(absx))) {
                                evalOK = false;
                            }
                        }

                        funcCount++;
                        guard1 = false;
                        if ((normGradF < *resnorm) && evalOK) {
                            iter++;
                            funDiff = std::abs(normGradF - *resnorm) / *resnorm;
                            for (i = 0; i < 6; i++) {
                                fCurrent[i] = fNew[i];
                            }

                            *resnorm = normGradF;
                            for (i = 0; i < n; i++) {
                                loop_ub = (n + 6) * i;
                                b_exitflag = 6 * i;
                                for (k = 0; k < 6; k++) {
                                    jacobian[b_exitflag + k] = augJacobian[loop_ub + k];
                                }
                            }

                            evalOK = true;
                            loop_ub = 6 * n;
                            for (i = 0; i < loop_ub; i++) {
                                if ((!evalOK) || (rtIsInf(jacobian[i]) || rtIsNaN(jacobian[i])))
                                {
                                    evalOK = false;
                                }
                            }

                            if (evalOK) {
                                b_exitflag = x.size(0);
                                loop_ub = x.size(1);
                                x.set_size(1, loop_ub);
                                loop_ub *= b_exitflag;
                                for (i = 0; i < loop_ub; i++) {
                                    x[i] = xp[i];
                                }

                                if (stepSuccessful) {
                                    b_gamma *= 0.1;
                                }

                                stepSuccessful = true;
                                guard1 = true;
                            }
                            else {
                                b_exitflag = 2;
                                for (k = 0; k < loop_ub; k++) {
                                    jacobian[k] = rtNaN;
                                }

                                exitg1 = true;
                            }
                        }
                        else {
                            b_gamma *= 10.0;
                            stepSuccessful = false;
                            loop_ub = jacobian.size(1);
                            for (i = 0; i < loop_ub; i++) {
                                for (b_exitflag = 0; b_exitflag < 6; b_exitflag++) {
                                    augJacobian[b_exitflag + augJacobian.size(0) * i] =
                                        jacobian[b_exitflag + 6 * i];
                                }
                            }

                            guard1 = true;
                        }

                        if (guard1) {
                            normGradF = std::sqrt(b_gamma);
                            for (i = 0; i < n; i++) {
                                b_exitflag = (n + 6) * (i + 1) - n;
                                for (k = 0; k < n; k++) {
                                    augJacobian[b_exitflag + k] = 0.0;
                                }

                                augJacobian[(i + augJacobian.size(0) * i) + 6] = normGradF;
                            }

                            internal::blas::xgemv(n, jacobian, fCurrent, gradf);
                            unusedU1.set_size(gradf.size(0));
                            loop_ub = gradf.size(0);
                            for (i = 0; i < loop_ub; i++) {
                                unusedU1[i] = -gradf[i];
                            }

                            projectBox(x, unusedU1);
                            if (gradf.size(0) == 0) {
                                normGradF = 0.0;
                            }
                            else {
                                normGradF = 0.0;
                                i = gradf.size(0);
                                for (k = 0; k < i; k++) {
                                    absx = std::abs(gradf[k]);
                                    if (rtIsNaN(absx) || (absx > normGradF)) {
                                        normGradF = absx;
                                    }
                                }
                            }

                            if (normGradF <= 1.0000000000000002E-14 * relFactor) {
                                b_exitflag = 1;
                            }
                            else if (funcCount >= options_MaxFunctionEvaluations) {
                                b_exitflag = 0;
                            }
                            else if (iter >= 400) {
                                b_exitflag = 0;
                            }
                            else {
                                loop_ub = dx.size(1);
                                b_exitflag = x.size(1);
                                unusedU1 = dx.reshape(loop_ub);
                                b_x0 = x.reshape(b_exitflag);
                                if (b_norm(unusedU1) < 1.0E-6 * (b_norm(b_x0) +
                                    1.4901161193847656E-8)) {
                                    b_exitflag = 4;
                                    if (!stepSuccessful) {
                                        iter++;
                                    }
                                }
                                else if (funDiff <= 1.0E-10) {
                                    b_exitflag = 3;
                                }
                                else {
                                    b_exitflag = -5;
                                }
                            }

                            if (b_exitflag != -5) {
                                exitg1 = true;
                            }
                        }
                    }

                    if (gradf.size(0) == 0) {
                        normGradF = 0.0;
                    }
                    else {
                        normGradF = 0.0;
                        i = gradf.size(0);
                        for (k = 0; k < i; k++) {
                            absx = std::abs(gradf[k]);
                            if (rtIsNaN(absx) || (absx > normGradF)) {
                                normGradF = absx;
                            }
                        }
                    }

                    loop_ub = dx.size(1);
                    unusedU1 = dx.reshape(loop_ub);
                    *output_stepsize = b_norm(unusedU1);
                    lambda_lower.set_size(x0.size(1));
                    loop_ub = x0.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        lambda_lower[i] = 0.0;
                    }

                    lambda_upper.set_size(x0.size(1));
                    loop_ub = x0.size(1);
                    for (i = 0; i < loop_ub; i++) {
                        lambda_upper[i] = 0.0;
                    }

                    *exitflag = b_exitflag;
                    *output_iterations = iter;
                    *output_funcCount = funcCount;
                    *output_firstorderopt = normGradF;
                }
            }
        }
    }

}

// Function Definitions
//
// test: q = [0.1328   -1.6864   -0.0698    0.7795    1.1255   -0.6565];
//  test: q =  [-0.0635   -2.0865    3.0076    1.3364    0.0030   -0.1817];
// Arguments    : const struct0_T *robot
//                const double Td[16]
//                const coder::array<double, 2U> &ref
//                const double tol[2]
//                coder::array<double, 2U> &angles
//                double *flag
// Return Type  : void
//
void inverse_kin_general(const Robot *robot, const double Td[16], const
  coder::array<double, 2U> &ref, const double tol[2], coder::array<double, 2U>
  &angles, double *flag)
{
  coder::anonymous_function fun;
  coder::array<double, 2U> Jb;
  coder::array<double, 2U> U;
  coder::array<double, 2U> b_angles;
  coder::array<double, 1U> c_expl_temp;
  coder::array<double, 1U> delta;
  double A[36];
  double U_data[36];
  double V_data[36];
  double b_V_data[36];
  double T[16];
  double b_Td[9];
  double sr[9];
  double b[6];
  double b_r[6];
  double s_data[6];
  double b_rd[3];
  double dv[3];
  double r[3];
  double rd[3];
  double rd2[3];
  double exitflag;
  double expl_temp;
  double norm_rd;
  double rd2_idx_0;
  double rd2_idx_1;
  double rd2_idx_2;
  int U_size[2];
  int V_size[2];
  int s_size[1];
  int ar;
  int b_exponent;
  int cnt;
  int cr;
  int exponent;
  int i;
  int ib;
  int nx;
  int vcol;
  char b_expl_temp[19];
  signed char b_I[9];
  angles.set_size(1, ref.size(1));
  nx = ref.size(0) * ref.size(1);
  for (i = 0; i < nx; i++) {
    angles[i] = ref[i];
  }

  for (i = 0; i < 3; i++) {
    nx = i << 2;
    b_Td[3 * i] = Td[nx];
    b_Td[3 * i + 1] = Td[nx + 1];
    b_Td[3 * i + 2] = Td[nx + 2];
  }

  logR(b_Td, dv);
  rd[0] = dv[0];
  rd[1] = dv[1];
  rd[2] = dv[2];
  norm_rd = coder::b_norm(rd);
  if (norm_rd != 0.0) {
    rd2_idx_0 = (6.2831853071795862 - norm_rd) * (-dv[0] / norm_rd);
    rd2_idx_1 = (6.2831853071795862 - norm_rd) * (-dv[1] / norm_rd);
    rd2_idx_2 = (6.2831853071795862 - norm_rd) * (-dv[2] / norm_rd);
  } else {
    rd2_idx_0 = dv[0];
    rd2_idx_1 = dv[1];
    rd2_idx_2 = dv[2];
  }

  jacobian_matrix(robot->dof, robot->A, robot->M, robot->ME, ref, Jb, T);
  for (i = 0; i < 3; i++) {
    nx = i << 2;
    b_Td[3 * i] = T[nx];
    b_Td[3 * i + 1] = T[nx + 1];
    b_Td[3 * i + 2] = T[nx + 2];
  }

  logR(b_Td, dv);
  cnt = 0;
  r[0] = dv[0];
  b_rd[0] = rd[0] - dv[0];
  rd2[0] = rd2_idx_0 - dv[0];
  r[1] = dv[1];
  b_rd[1] = rd[1] - dv[1];
  rd2[1] = rd2_idx_1 - dv[1];
  r[2] = dv[2];
  b_rd[2] = rd[2] - dv[2];
  rd2[2] = rd2_idx_2 - dv[2];
  if (coder::b_norm(b_rd) < coder::b_norm(rd2)) {
    b[0] = rd[0];
    b[3] = Td[12];
    b_r[0] = dv[0];
    b_r[3] = T[12];
    b[1] = rd[1];
    b[4] = Td[13];
    b_r[1] = dv[1];
    b_r[4] = T[13];
    b[2] = rd[2];
    b[5] = Td[14];
    b_r[2] = dv[2];
    b_r[5] = T[14];
    for (i = 0; i < 6; i++) {
      b[i] -= b_r[i];
    }
  } else {
    b[0] = rd2_idx_0;
    b[3] = Td[12];
    b_r[0] = dv[0];
    b_r[3] = T[12];
    b[1] = rd2_idx_1;
    b[4] = Td[13];
    b_r[1] = dv[1];
    b_r[4] = T[13];
    b[2] = rd2_idx_2;
    b[5] = Td[14];
    b_r[2] = dv[2];
    b_r[5] = T[14];
    for (i = 0; i < 6; i++) {
      b[i] -= b_r[i];
    }
  }

  while (((coder::b_norm(*(double (*)[3])&b[0]) > tol[0]) || (coder::b_norm
           (*(double (*)[3])&b[3]) > tol[1])) && (cnt < 10)) {
    int i1;
    int ia;
    int n;

    //      Ja = analytic_jacobian_matrix(Jb, T);
    //      delta = lsqminnorm(Jb, [w_dr_A(r), zeros(3); zeros(3),T(1:3,1:3)'] * b); 
    //      delta = Jb \ ([w_dr_A(r), zeros(3); zeros(3),T(1:3,1:3)'] * b);
    //  wb = A(r)dr/dt
    norm_rd = coder::b_norm(r);
    if (norm_rd != 0.0) {
      //  x: vector or m by 3 matrix
      //  X: 3 by 3 by m matrix
      sr[0] = 0.0;
      sr[3] = -r[2];
      sr[6] = r[1];
      sr[1] = r[2];
      sr[4] = 0.0;
      sr[7] = -r[0];
      sr[2] = -r[1];
      sr[5] = r[0];
      sr[8] = 0.0;
      exitflag = (1.0 - std::cos(norm_rd)) / (norm_rd * norm_rd);
      norm_rd = (norm_rd - std::sin(norm_rd)) / rt_powd_snf(norm_rd, 3.0);
      for (i = 0; i < 9; i++) {
        b_I[i] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < 3; i1++) {
          nx = i + 3 * i1;
          b_Td[nx] = (static_cast<double>(b_I[nx]) - exitflag * sr[nx]) +
            ((norm_rd * sr[i] * sr[3 * i1] + norm_rd * sr[i + 3] * sr[3 * i1 + 1])
             + norm_rd * sr[i + 6] * sr[3 * i1 + 2]);
        }
      }
    } else {
      std::memset(&b_Td[0], 0, 9U * sizeof(double));
      b_Td[0] = 1.0;
      b_Td[4] = 1.0;
      b_Td[8] = 1.0;
    }

    for (i = 0; i < 3; i++) {
      A[6 * i] = b_Td[3 * i];
      nx = 6 * (i + 3);
      A[nx] = 0.0;
      A[6 * i + 3] = 0.0;
      A[nx + 3] = T[i];
      A[6 * i + 1] = b_Td[3 * i + 1];
      A[nx + 1] = 0.0;
      A[6 * i + 4] = 0.0;
      A[nx + 4] = T[i + 4];
      A[6 * i + 2] = b_Td[3 * i + 2];
      A[nx + 2] = 0.0;
      A[6 * i + 5] = 0.0;
      A[nx + 5] = T[i + 8];
    }

    for (i = 0; i < 6; i++) {
      norm_rd = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        norm_rd += A[i + 6 * i1] * b[i1];
      }

      b_r[i] = norm_rd;
    }

    if (6 < Jb.size(1)) {
      boolean_T p;
      b_angles.set_size(Jb.size(1), 6);
      nx = Jb.size(1);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < nx; i1++) {
          b_angles[i1 + b_angles.size(0) * i] = Jb[i + 6 * i1];
        }
      }

      n = b_angles.size(0);
      Jb.set_size(6, b_angles.size(0));
      nx = 6 * b_angles.size(0);
      for (i = 0; i < nx; i++) {
        Jb[i] = 0.0;
      }

      nx = b_angles.size(0) * 6;
      p = true;
      for (ia = 0; ia < nx; ia++) {
        if ((!p) || (rtIsInf(b_angles[ia]) || rtIsNaN(b_angles[ia]))) {
          p = false;
        }
      }

      if (!p) {
        Jb.set_size(6, b_angles.size(0));
        nx = 6 * b_angles.size(0);
        for (i = 0; i < nx; i++) {
          Jb[i] = rtNaN;
        }
      } else {
        int c_r;
        coder::internal::svd(b_angles, U, s_data, s_size, b_V_data, V_size);
        norm_rd = std::abs(s_data[0]);
        if ((!rtIsInf(norm_rd)) && (!rtIsNaN(norm_rd))) {
          if (norm_rd <= 2.2250738585072014E-308) {
            norm_rd = 4.94065645841247E-324;
          } else {
            frexp(norm_rd, &b_exponent);
            norm_rd = std::ldexp(1.0, b_exponent - 53);
          }
        } else {
          norm_rd = rtNaN;
        }

        norm_rd *= static_cast<double>(b_angles.size(0));
        c_r = -1;
        ia = 0;
        while ((ia < 6) && (s_data[ia] > norm_rd)) {
          c_r++;
          ia++;
        }

        if (c_r + 1 > 0) {
          int ic;
          vcol = 1;
          if (0 <= c_r) {
            V_size[0] = 6;
            V_size[1] = 6;
          }

          for (ar = 0; ar <= c_r; ar++) {
            exitflag = 1.0 / s_data[ar];
            std::memcpy(&U_data[0], &b_V_data[0], 36U * sizeof(double));
            i = vcol + 5;
            for (ia = vcol; ia <= i; ia++) {
              U_data[ia - 1] *= exitflag;
            }

            std::memcpy(&b_V_data[0], &U_data[0], 36U * sizeof(double));
            vcol += 6;
          }

          nx = 6 * (b_angles.size(0) - 1);
          for (cr = 0; cr <= nx; cr += 6) {
            i = cr + 1;
            i1 = cr + 6;
            for (ic = i; ic <= i1; ic++) {
              Jb[ic - 1] = 0.0;
            }
          }

          vcol = 0;
          for (cr = 0; cr <= nx; cr += 6) {
            ar = -1;
            vcol++;
            i = vcol + n * c_r;
            for (ib = vcol; n < 0 ? ib >= i : ib <= i; ib += n) {
              int i2;
              ia = ar;
              i1 = cr + 1;
              i2 = cr + 6;
              for (ic = i1; ic <= i2; ic++) {
                ia++;
                Jb[ic - 1] = Jb[ic - 1] + U[ib - 1] * b_V_data[ia];
              }

              ar += 6;
            }
          }
        }
      }

      b_angles.set_size(Jb.size(1), 6);
      nx = Jb.size(1);
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < nx; i1++) {
          b_angles[i1 + b_angles.size(0) * i] = Jb[i + 6 * i1];
        }
      }
    } else {
      int X_size_idx_0;
      n = Jb.size(1);
      X_size_idx_0 = Jb.size(1);
      nx = Jb.size(1) * 6;
      if (0 <= nx - 1) {
        std::memset(&A[0], 0, nx * sizeof(double));
      }

      if (Jb.size(1) != 0) {
        boolean_T p;
        nx = 6 * Jb.size(1);
        p = true;
        for (ia = 0; ia < nx; ia++) {
          if ((!p) || (rtIsInf(Jb[ia]) || rtIsNaN(Jb[ia]))) {
            p = false;
          }
        }

        if (!p) {
          X_size_idx_0 = Jb.size(1);
          nx = Jb.size(1) * 6;
          for (i = 0; i < nx; i++) {
            A[i] = rtNaN;
          }
        } else {
          int c_r;
          coder::internal::svd((double *)Jb.data(), Jb.size(), U_data, U_size, b,
                               s_size, V_data, V_size);
          norm_rd = std::abs(b[0]);
          if ((!rtIsInf(norm_rd)) && (!rtIsNaN(norm_rd))) {
            if (norm_rd <= 2.2250738585072014E-308) {
              norm_rd = 4.94065645841247E-324;
            } else {
              frexp(norm_rd, &exponent);
              norm_rd = std::ldexp(1.0, exponent - 53);
            }
          } else {
            norm_rd = rtNaN;
          }

          norm_rd *= 6.0;
          c_r = -1;
          ia = 0;
          while ((ia <= n - 1) && (b[ia] > norm_rd)) {
            c_r++;
            ia++;
          }

          if (c_r + 1 > 0) {
            vcol = 1;
            for (ar = 0; ar <= c_r; ar++) {
              exitflag = 1.0 / b[ar];
              b_angles.set_size(V_size[0], V_size[1]);
              nx = V_size[0] * V_size[1];
              for (i = 0; i < nx; i++) {
                b_angles[i] = V_data[i];
              }

              i = vcol + n;
              i1 = i - 1;
              for (ia = vcol; ia <= i1; ia++) {
                b_angles[ia - 1] = exitflag * b_angles[ia - 1];
              }

              V_size[0] = b_angles.size(0);
              V_size[1] = b_angles.size(1);
              nx = b_angles.size(0) * b_angles.size(1);
              for (i1 = 0; i1 < nx; i1++) {
                V_data[i1] = b_angles[i1];
              }

              vcol = i;
            }

            nx = Jb.size(1) * 5;
            for (cr = 0; n < 0 ? cr >= nx : cr <= nx; cr += n) {
              i = cr + 1;
              i1 = cr + n;
              if (i <= i1) {
                std::memset(&A[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
              }
            }

            vcol = 0;
            for (cr = 0; n < 0 ? cr >= nx : cr <= nx; cr += n) {
              ar = -1;
              vcol++;
              i = vcol + 6 * c_r;
              for (ib = vcol; ib <= i; ib += 6) {
                int i2;
                ia = ar;
                i1 = cr + 1;
                i2 = cr + n;
                for (int ic = i1; ic <= i2; ic++) {
                  ia++;
                  A[ic - 1] += U_data[ib - 1] * V_data[ia];
                }

                ar += n;
              }
            }
          }
        }
      }

      b_angles.set_size(X_size_idx_0, 6);
      nx = X_size_idx_0 * 6;
      for (i = 0; i < nx; i++) {
        b_angles[i] = A[i];
      }
    }

    n = b_angles.size(0);
    delta.set_size(b_angles.size(0));
    for (nx = 0; nx < n; nx++) {
      norm_rd = 0.0;
      for (ia = 0; ia < 6; ia++) {
        norm_rd += b_angles[ia * b_angles.size(0) + nx] * b_r[ia];
      }

      delta[nx] = norm_rd;
    }

    //      delta = mod(delta + pi, 2*pi) - pi;
    angles.set_size(1, angles.size(1));
    nx = angles.size(1);
    for (i = 0; i < nx; i++) {
      angles[i] = angles[i] + delta[i];
    }

    cnt++;
    jacobian_matrix(robot->dof, robot->A, robot->M, robot->ME, angles, Jb, T);
    for (i = 0; i < 3; i++) {
      nx = i << 2;
      b_Td[3 * i] = T[nx];
      b_Td[3 * i + 1] = T[nx + 1];
      b_Td[3 * i + 2] = T[nx + 2];
    }

    logR(b_Td, dv);
    r[0] = dv[0];
    b_rd[0] = rd[0] - dv[0];
    rd2[0] = rd2_idx_0 - dv[0];
    r[1] = dv[1];
    b_rd[1] = rd[1] - dv[1];
    rd2[1] = rd2_idx_1 - dv[1];
    r[2] = dv[2];
    b_rd[2] = rd[2] - dv[2];
    rd2[2] = rd2_idx_2 - dv[2];
    if (coder::b_norm(b_rd) < coder::b_norm(rd2)) {
      b[0] = rd[0];
      b[3] = Td[12];
      b_r[0] = dv[0];
      b_r[3] = T[12];
      b[1] = rd[1];
      b[4] = Td[13];
      b_r[1] = dv[1];
      b_r[4] = T[13];
      b[2] = rd[2];
      b[5] = Td[14];
      b_r[2] = dv[2];
      b_r[5] = T[14];
      for (i = 0; i < 6; i++) {
        b[i] -= b_r[i];
      }
    } else {
      b[0] = rd2_idx_0;
      b[3] = Td[12];
      b_r[0] = dv[0];
      b_r[3] = T[12];
      b[1] = rd2_idx_1;
      b[4] = Td[13];
      b_r[1] = dv[1];
      b_r[4] = T[13];
      b[2] = rd2_idx_2;
      b[5] = Td[14];
      b_r[2] = dv[2];
      b_r[5] = T[14];
      for (i = 0; i < 6; i++) {
        b[i] -= b_r[i];
      }
    }
  }

  b_angles.set_size(1, angles.size(1));
  nx = angles.size(0) * angles.size(1);
  for (i = 0; i < nx; i++) {
    b_angles[i] = angles[i] + 3.1415926535897931;
  }

  coder::b_mod(b_angles, angles);
  i = angles.size(0) * angles.size(1);
  angles.set_size(1, angles.size(1));
  nx = i - 1;
  for (i = 0; i <= nx; i++) {
    angles[i] = angles[i] - 3.1415926535897931;
  }

  if (cnt < 10) {
    nx = 1;
  } else {
    //  tol = [1e-5, 1e-5]
    std::memcpy(&fun.tunableEnvironment.f1[0], &Td[0], 16U * sizeof(double));
    fun.tunableEnvironment.f2 = *robot;
    coder::optim::coder::levenbergMarquardt::driver(&fun, ref, angles, &norm_rd,
      b, &exitflag, &rd2_idx_2, &rd2_idx_0, &rd2_idx_1, &expl_temp, b_expl_temp,
      delta, c_expl_temp, Jb);
    b_angles.set_size(1, angles.size(1));
    nx = angles.size(0) * angles.size(1);
    for (i = 0; i < nx; i++) {
      b_angles[i] = angles[i] + 3.1415926535897931;
    }

    coder::b_mod(b_angles, angles);
    i = angles.size(0) * angles.size(1);
    angles.set_size(1, angles.size(1));
    nx = i - 1;
    for (i = 0; i <= nx; i++) {
      angles[i] = angles[i] - 3.1415926535897931;
    }

    if ((coder::b_norm(*(double (*)[3])&b[0]) < tol[0]) && (coder::b_norm
         (*(double (*)[3])&b[3]) < tol[1])) {
      nx = 1;
    } else {
      nx = 0;
    }
  }

  *flag = nx;
}

//
// File trailer for inverse_kin_general.cpp
//
// [EOF]
//
