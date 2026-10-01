//
// File: logR.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Apr-2023 20:33:26
//

// Include Files
#include "logR.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>


namespace coder
{
    namespace internal
    {
        namespace blas
        {

            static void xswap(double x[9], int ix0, int iy0)
            {
                double temp;
                temp = x[ix0 - 1];
                x[ix0 - 1] = x[iy0 - 1];
                x[iy0 - 1] = temp;
                temp = x[ix0];
                x[ix0] = x[iy0];
                x[iy0] = temp;
                temp = x[ix0 + 1];
                x[ix0 + 1] = x[iy0 + 1];
                x[iy0 + 1] = temp;
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
            static void xrot(double x[9], int ix0, int iy0, double c, double s)
            {
                double temp;
                double temp_tmp;
                temp = x[iy0 - 1];
                temp_tmp = x[ix0 - 1];
                x[iy0 - 1] = c * temp - s * temp_tmp;
                x[ix0 - 1] = c * temp_tmp + s * temp;
                temp = c * x[ix0] + s * x[iy0];
                x[iy0] = c * x[iy0] - s * x[ix0];
                x[ix0] = temp;
                temp = x[iy0 + 1];
                temp_tmp = x[ix0 + 1];
                x[iy0 + 1] = c * temp - s * temp_tmp;
                x[ix0 + 1] = c * temp_tmp + s * temp;
            }

            static double xnrm2(const double x[3], int ix0)
            {
                double scale;
                double y;
                int kend;
                y = 0.0;
                scale = 3.3121686421112381E-170;
                kend = ix0 + 1;
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
            //                const double x[9]
            //                int ix0
            // Return Type  : double
            //
            static double xnrm2(int n, const double x[9], int ix0)
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
            static double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
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

            static void b_xaxpy(int n, double a, const double x[3], int ix0, double y[9], int
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
                        y[iy] += a * x[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                const double x[9]
            //                int ix0
            //                double y[3]
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, const double x[9], int ix0, double y[3], int
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
                        y[iy] += a * x[ix];
                        ix++;
                        iy++;
                    }
                }
            }

            //
            // Arguments    : int n
            //                double a
            //                int ix0
            //                double y[9]
            //                int iy0
            // Return Type  : void
            //
            static void xaxpy(int n, double a, int ix0, double y[9], int iy0)
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

        static void svd(const double A[9], double U[9], double s[3], double V[9])
        {
            double b_A[9];
            double b_s[3];
            double e[3];
            double work[3];
            double nrm;
            double rt;
            double sm;
            double snorm;
            double sqds;
            double ztest;
            int kase;
            int m;
            int q;
            int qjj;
            int qp1;
            int qq;
            boolean_T apply_transform;
            e[0] = 0.0;
            work[0] = 0.0;
            e[1] = 0.0;
            work[1] = 0.0;
            e[2] = 0.0;
            work[2] = 0.0;
            for (kase = 0; kase < 9; kase++) {
                b_A[kase] = A[kase];
                U[kase] = 0.0;
                V[kase] = 0.0;
            }

            apply_transform = false;
            nrm = blas::xnrm2(3, b_A, 1);
            if (nrm > 0.0) {
                apply_transform = true;
                if (b_A[0] < 0.0) {
                    ztest = -nrm;
                    b_s[0] = -nrm;
                }
                else {
                    ztest = nrm;
                    b_s[0] = nrm;
                }

                if (std::abs(ztest) >= 1.0020841800044864E-292) {
                    nrm = 1.0 / ztest;
                    for (qp1 = 1; qp1 < 4; qp1++) {
                        b_A[qp1 - 1] *= nrm;
                    }
                }
                else {
                    for (qp1 = 1; qp1 < 4; qp1++) {
                        b_A[qp1 - 1] /= b_s[0];
                    }
                }

                b_A[0]++;
                b_s[0] = -b_s[0];
            }
            else {
                b_s[0] = 0.0;
            }

            for (kase = 2; kase < 4; kase++) {
                qjj = 3 * (kase - 1);
                if (apply_transform) {
                    blas::xaxpy(3, -(blas::xdotc(3, b_A, 1, b_A, qjj + 1) / b_A[0]), 1,
                        b_A, qjj + 1);
                }

                e[kase - 1] = b_A[qjj];
            }

            for (qp1 = 1; qp1 < 4; qp1++) {
                U[qp1 - 1] = b_A[qp1 - 1];
            }

            nrm = blas::xnrm2(e, 2);
            if (nrm == 0.0) {
                e[0] = 0.0;
            }
            else {
                if (e[1] < 0.0) {
                    e[0] = -nrm;
                }
                else {
                    e[0] = nrm;
                }

                nrm = e[0];
                if (std::abs(e[0]) >= 1.0020841800044864E-292) {
                    nrm = 1.0 / e[0];
                    for (qp1 = 2; qp1 < 4; qp1++) {
                        e[qp1 - 1] *= nrm;
                    }
                }
                else {
                    for (qp1 = 2; qp1 < 4; qp1++) {
                        e[qp1 - 1] /= nrm;
                    }
                }

                e[1]++;
                e[0] = -e[0];
                for (qp1 = 2; qp1 < 4; qp1++) {
                    work[qp1 - 1] = 0.0;
                }

                for (kase = 2; kase < 4; kase++) {
                    blas::xaxpy(2, e[kase - 1], b_A, 3 * (kase - 1) + 2, work, 2);
                }

                for (kase = 2; kase < 4; kase++) {
                    blas::b_xaxpy(2, -e[kase - 1] / e[1], work, 2, b_A, 3 * (kase - 1) + 2);
                }
            }

            for (qp1 = 2; qp1 < 4; qp1++) {
                V[qp1 - 1] = e[qp1 - 1];
            }

            apply_transform = false;
            nrm = blas::xnrm2(2, b_A, 5);
            if (nrm > 0.0) {
                apply_transform = true;
                if (b_A[4] < 0.0) {
                    ztest = -nrm;
                    b_s[1] = -nrm;
                }
                else {
                    ztest = nrm;
                    b_s[1] = nrm;
                }

                if (std::abs(ztest) >= 1.0020841800044864E-292) {
                    nrm = 1.0 / ztest;
                    for (qp1 = 5; qp1 < 7; qp1++) {
                        b_A[qp1 - 1] *= nrm;
                    }
                }
                else {
                    for (qp1 = 5; qp1 < 7; qp1++) {
                        b_A[qp1 - 1] /= b_s[1];
                    }
                }

                b_A[4]++;
                b_s[1] = -b_s[1];
            }
            else {
                b_s[1] = 0.0;
            }

            for (kase = 3; kase < 4; kase++) {
                if (apply_transform) {
                    blas::xaxpy(2, -(blas::xdotc(2, b_A, 5, b_A, 8) / b_A[4]), 5, b_A, 8);
                }
            }

            for (qp1 = 2; qp1 < 4; qp1++) {
                U[qp1 + 2] = b_A[qp1 + 2];
            }

            m = 1;
            b_s[2] = b_A[8];
            e[1] = b_A[7];
            e[2] = 0.0;
            U[6] = 0.0;
            U[7] = 0.0;
            U[8] = 1.0;
            for (q = 1; q >= 0; q--) {
                qp1 = q + 2;
                qq = q + 3 * q;
                if (b_s[q] != 0.0) {
                    for (kase = qp1; kase < 4; kase++) {
                        qjj = (q + 3 * (kase - 1)) + 1;
                        blas::xaxpy(3 - q, -(blas::xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]),
                            qq + 1, U, qjj);
                    }

                    for (qp1 = q + 1; qp1 < 4; qp1++) {
                        kase = (qp1 + 3 * q) - 1;
                        U[kase] = -U[kase];
                    }

                    U[qq]++;
                    if (0 <= q - 1) {
                        U[3 * q] = 0.0;
                    }
                }
                else {
                    U[3 * q] = 0.0;
                    U[3 * q + 1] = 0.0;
                    U[3 * q + 2] = 0.0;
                    U[qq] = 1.0;
                }
            }

            for (q = 2; q >= 0; q--) {
                if ((q + 1 <= 1) && (e[0] != 0.0)) {
                    blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
                    blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
                }

                V[3 * q] = 0.0;
                V[3 * q + 1] = 0.0;
                V[3 * q + 2] = 0.0;
                V[q + 3 * q] = 1.0;
            }

            qq = 0;
            snorm = 0.0;
            for (q = 0; q < 3; q++) {
                ztest = b_s[q];
                if (ztest != 0.0) {
                    rt = std::abs(ztest);
                    nrm = ztest / rt;
                    b_s[q] = rt;
                    if (q + 1 < 3) {
                        e[q] /= nrm;
                    }

                    qjj = 3 * q;
                    kase = qjj + 3;
                    for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                        U[qp1 - 1] *= nrm;
                    }
                }

                if (q + 1 < 3) {
                    ztest = e[q];
                    if (ztest != 0.0) {
                        rt = std::abs(ztest);
                        nrm = rt / ztest;
                        e[q] = rt;
                        b_s[q + 1] *= nrm;
                        qjj = 3 * (q + 1);
                        kase = qjj + 3;
                        for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                            V[qp1 - 1] *= nrm;
                        }
                    }
                }

                nrm = std::abs(b_s[q]);
                ztest = std::abs(e[q]);
                if ((nrm > ztest) || rtIsNaN(ztest)) {
                    ztest = nrm;
                }

                if ((!(snorm > ztest)) && (!rtIsNaN(ztest))) {
                    snorm = ztest;
                }
            }

            while ((m + 2 > 0) && (qq < 75)) {
                qp1 = m;
                int exitg1;
                do {
                    exitg1 = 0;
                    q = qp1 + 1;
                    if (qp1 + 1 == 0) {
                        exitg1 = 1;
                    }
                    else {
                        nrm = std::abs(e[qp1]);
                        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[qp1]) + std::abs
                        (b_s[qp1 + 1]))) || (nrm <= 1.0020841800044864E-292) || ((qq >
                            20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                            e[qp1] = 0.0;
                            exitg1 = 1;
                        }
                        else {
                            qp1--;
                        }
                    }
                } while (exitg1 == 0);

                if (qp1 + 1 == m + 1) {
                    kase = 4;
                }
                else {
                    boolean_T exitg2;
                    qjj = m + 2;
                    kase = m + 2;
                    exitg2 = false;
                    while ((!exitg2) && (kase >= qp1 + 1)) {
                        qjj = kase;
                        if (kase == qp1 + 1) {
                            exitg2 = true;
                        }
                        else {
                            nrm = 0.0;
                            if (kase < m + 2) {
                                nrm = std::abs(e[kase - 1]);
                            }

                            if (kase > qp1 + 2) {
                                nrm += std::abs(e[kase - 2]);
                            }

                            ztest = std::abs(b_s[kase - 1]);
                            if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                                1.0020841800044864E-292)) {
                                b_s[kase - 1] = 0.0;
                                exitg2 = true;
                            }
                            else {
                                kase--;
                            }
                        }
                    }

                    if (qjj == qp1 + 1) {
                        kase = 3;
                    }
                    else if (qjj == m + 2) {
                        kase = 1;
                    }
                    else {
                        kase = 2;
                        q = qjj;
                    }
                }

                switch (kase) {
                case 1:
                    ztest = e[m];
                    e[m] = 0.0;
                    kase = m + 1;
                    for (qp1 = kase; qp1 >= q + 1; qp1--) {
                        blas::xrotg(&b_s[qp1 - 1], &ztest, &sm, &sqds);
                        if (qp1 > q + 1) {
                            ztest = -sqds * e[0];
                            e[0] *= sm;
                        }

                        blas::xrot(V, 3 * (qp1 - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
                    }
                    break;

                case 2:
                    ztest = e[q - 1];
                    e[q - 1] = 0.0;
                    for (qp1 = q + 1; qp1 <= m + 2; qp1++) {
                        blas::xrotg(&b_s[qp1 - 1], &ztest, &sm, &sqds);
                        rt = e[qp1 - 1];
                        ztest = -sqds * rt;
                        e[qp1 - 1] = rt * sm;
                        blas::xrot(U, 3 * (qp1 - 1) + 1, 3 * (q - 1) + 1, sm, sqds);
                    }
                    break;

                case 3:
                {
                    double scale;
                    kase = m + 1;
                    nrm = b_s[m + 1];
                    scale = std::abs(nrm);
                    ztest = std::abs(b_s[m]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(e[m]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(b_s[q]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    ztest = std::abs(e[q]);
                    if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
                        scale = ztest;
                    }

                    sm = nrm / scale;
                    nrm = b_s[m] / scale;
                    ztest = e[m] / scale;
                    sqds = b_s[q] / scale;
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
                    nrm = sqds * (e[q] / scale);
                    for (qp1 = q + 1; qp1 <= kase; qp1++) {
                        blas::xrotg(&ztest, &nrm, &sm, &sqds);
                        if (qp1 > q + 1) {
                            e[0] = ztest;
                        }

                        nrm = e[qp1 - 1];
                        rt = b_s[qp1 - 1];
                        e[qp1 - 1] = sm * nrm - sqds * rt;
                        ztest = sqds * b_s[qp1];
                        b_s[qp1] *= sm;
                        blas::xrot(V, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
                        b_s[qp1 - 1] = sm * rt + sqds * nrm;
                        blas::xrotg(&b_s[qp1 - 1], &ztest, &sm, &sqds);
                        ztest = sm * e[qp1 - 1] + sqds * b_s[qp1];
                        b_s[qp1] = -sqds * e[qp1 - 1] + sm * b_s[qp1];
                        nrm = sqds * e[qp1];
                        e[qp1] *= sm;
                        blas::xrot(U, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
                    }

                    e[m] = ztest;
                    qq++;
                }
                break;

                default:
                    if (b_s[q] < 0.0) {
                        b_s[q] = -b_s[q];
                        qjj = 3 * q;
                        kase = qjj + 3;
                        for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                            V[qp1 - 1] = -V[qp1 - 1];
                        }
                    }

                    qp1 = q + 1;
                    while ((q + 1 < 3) && (b_s[q] < b_s[qp1])) {
                        rt = b_s[q];
                        b_s[q] = b_s[qp1];
                        b_s[qp1] = rt;
                        blas::xswap(V, 3 * q + 1, 3 * (q + 1) + 1);
                        blas::xswap(U, 3 * q + 1, 3 * (q + 1) + 1);
                        q = qp1;
                        qp1++;
                    }

                    qq = 0;
                    m--;
                    break;
                }
            }

            s[0] = b_s[0];
            s[1] = b_s[1];
            s[2] = b_s[2];
        }

    }
           
}

static double rt_atan2d_snf(double u0, double u1)
{
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
        y = rtNaN;
    }
    else if (rtIsInf(u0) && rtIsInf(u1)) {
        int b_u0;
        int b_u1;
        if (u0 > 0.0) {
            b_u0 = 1;
        }
        else {
            b_u0 = -1;
        }

        if (u1 > 0.0) {
            b_u1 = 1;
        }
        else {
            b_u1 = -1;
        }

        y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
    }
    else if (u1 == 0.0) {
        if (u0 > 0.0) {
            y = RT_PI / 2.0;
        }
        else if (u0 < 0.0) {
            y = -(RT_PI / 2.0);
        }
        else {
            y = 0.0;
        }
    }
    else {
        y = std::atan2(u0, u1);
    }

    return y;
}

void logR(const double R[9], double w[3])
{
    double V[9];
    double b_I[9];
    double a;
    boolean_T p;
    std::memset(&b_I[0], 0, 9U * sizeof(double));
    b_I[0] = 1.0;
    b_I[4] = 1.0;
    b_I[8] = 1.0;
    p = true;
    for (int k{ 0 }; k < 9; k++) {
        a = R[k] - b_I[k];
        b_I[k] = a;
        if ((!p) || (std::isinf(a) || std::isnan(a))) {
            p = false;
        }
    }
    if (p) {
        double U[9];
        coder::internal::svd(b_I, U, w, V);
    }
    else {
        for (int k{ 0 }; k < 9; k++) {
            V[k] = rtNaN;
        }
    }
    a = rt_atan2d_snf(((R[5] - R[7]) * V[6] + (R[6] - R[2]) * V[7]) +
        (R[1] - R[3]) * V[8],
        ((R[0] + R[4]) + R[8]) - 1.0);
    w[0] = a * V[6];
    w[1] = a * V[7];
    w[2] = a * V[8];
}

//
// File trailer for logR.cpp
//
// [EOF]
//
