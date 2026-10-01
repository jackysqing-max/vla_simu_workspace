//
// File: force_calibration.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 30-Apr-2022 11:19:44
//

// Include Files
#include "force_calibration.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

static FILE * eml_openfiles[20];
static boolean_T isInitialized_force_calibration = false;
static boolean_T eml_autoflush[20];
static void filedata_init()
{
  FILE * a;
  a = NULL;
  for (int i = 0; i < 20; i++) {
    eml_autoflush[i] = false;
    eml_openfiles[i] = a;
  }
}

static void force_calibration_initialize()
{
  filedata_init();
  isInitialized_force_calibration = true;
}

static void force_calibration_terminate()
{
  // (no terminate code required)
  isInitialized_force_calibration = false;
}

static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}
// Variable Definitions



// Function Definitions
//
// Arguments    : void
// Return Type  : signed char
//
namespace coder
{
    static double b_std(const ::coder::array<double, 2U>& x)
    {
        array<double, 1U> absdiff;
        double xbar;
        double y;
        int kend;
        kend = x.size(1);
        if (x.size(1) == 0) {
            y = rtNaN;
        }
        else if (x.size(1) == 1) {
            if ((!rtIsInf(x[0])) && (!rtIsNaN(x[0]))) {
                y = 0.0;
            }
            else {
                y = rtNaN;
            }
        }
        else {
            int k;
            xbar = x[0];
            for (k = 2; k <= kend; k++) {
                xbar += x[k - 1];
            }

            xbar /= static_cast<double>(x.size(1));
            absdiff.set_size(x.size(1));
            for (k = 0; k < kend; k++) {
                absdiff[k] = std::abs(x[k] - xbar);
            }

            y = 0.0;
            xbar = 3.3121686421112381E-170;
            kend = x.size(1);
            for (k = 0; k < kend; k++) {
                if (absdiff[k] > xbar) {
                    double t;
                    t = xbar / absdiff[k];
                    y = y * t * t + 1.0;
                    xbar = absdiff[k];
                }
                else {
                    double t;
                    t = absdiff[k] / xbar;
                    y += t * t;
                }
            }

            y = xbar * std::sqrt(y);
            y /= std::sqrt(static_cast<double>(x.size(1)) - 1.0);
        }

        return y;
    }
    namespace internal
    {
        namespace blas
        {
            static double xnrm2(int n, const ::coder::array<double, 2U>& x, int ix0)
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
        }
    }
}

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
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
// Arguments    : const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 1U> &B
//                double Y[3]
// Return Type  : void
//
namespace coder
{
    namespace internal
    {
        static void qrsolve(const ::coder::array<double, 2U>& A, const ::coder::array<
            double, 1U>& B, double Y[3])
        {
            array<double, 2U> b_A;
            array<double, 1U> b_B;
            double tau_data[3];
            double vn1[3];
            double vn2[3];
            double work[3];
            double s;
            double smax;
            double temp2;
            int b_i;
            int i;
            int iy;
            int jA;
            int m;
            int minmana;
            int minmn;
            int pvt;
            int rankR;
            signed char jpvt[3];
            boolean_T guard1 = false;
            b_A.set_size(A.size(0), 3);
            minmana = A.size(0) * A.size(1);
            for (i = 0; i < minmana; i++) {
                b_A[i] = A[i];
            }

            m = A.size(0);
            minmana = A.size(0);
            if (minmana >= 3) {
                minmana = 3;
            }

            if (0 <= minmana - 1) {
                std::memset(&tau_data[0], 0, minmana * sizeof(double));
            }

            guard1 = false;
            if (A.size(0) == 0) {
                guard1 = true;
            }
            else {
                minmana = A.size(0);
                if (minmana >= 3) {
                    minmana = 3;
                }

                if (minmana < 1) {
                    guard1 = true;
                }
                else {
                    int ma;
                    ma = A.size(0);
                    minmn = A.size(0);
                    if (minmn >= 3) {
                        minmn = 3;
                    }

                    jpvt[0] = 1;
                    work[0] = 0.0;
                    smax = blas::xnrm2(A.size(0), A, 1);
                    vn1[0] = smax;
                    vn2[0] = smax;
                    jpvt[1] = 2;
                    work[1] = 0.0;
                    smax = blas::xnrm2(A.size(0), A, A.size(0) + 1);
                    vn1[1] = smax;
                    vn2[1] = smax;
                    jpvt[2] = 3;
                    work[2] = 0.0;
                    smax = blas::xnrm2(A.size(0), A, (A.size(0) << 1) + 1);
                    vn1[2] = smax;
                    vn2[2] = smax;
                    for (b_i = 0; b_i < minmn; b_i++) {
                        int ii;
                        int ip1;
                        int ix;
                        int mmi;
                        ip1 = b_i + 2;
                        iy = b_i * ma;
                        ii = iy + b_i;
                        mmi = m - b_i;
                        minmana = 3 - b_i;
                        pvt = 0;
                        if (3 - b_i > 1) {
                            ix = b_i;
                            smax = std::abs(vn1[b_i]);
                            for (rankR = 2; rankR <= minmana; rankR++) {
                                ix++;
                                s = std::abs(vn1[ix]);
                                if (s > smax) {
                                    pvt = rankR - 1;
                                    smax = s;
                                }
                            }
                        }

                        pvt += b_i;
                        if (pvt + 1 != b_i + 1) {
                            ix = pvt * ma;
                            for (rankR = 0; rankR < m; rankR++) {
                                smax = b_A[ix];
                                b_A[ix] = b_A[iy];
                                b_A[iy] = smax;
                                ix++;
                                iy++;
                            }

                            minmana = jpvt[pvt];
                            jpvt[pvt] = jpvt[b_i];
                            jpvt[b_i] = static_cast<signed char>(minmana);
                            vn1[pvt] = vn1[b_i];
                            vn2[pvt] = vn2[b_i];
                        }

                        if (b_i + 1 < m) {
                            temp2 = b_A[ii];
                            minmana = ii + 2;
                            tau_data[b_i] = 0.0;
                            if (mmi > 0) {
                                smax = blas::xnrm2(mmi - 1, b_A, ii + 2);
                                if (smax != 0.0) {
                                    s = rt_hypotd_snf(b_A[ii], smax);
                                    if (b_A[ii] >= 0.0) {
                                        s = -s;
                                    }

                                    if (std::abs(s) < 1.0020841800044864E-292) {
                                        pvt = -1;
                                        i = ii + mmi;
                                        do {
                                            pvt++;
                                            for (rankR = minmana; rankR <= i; rankR++) {
                                                b_A[rankR - 1] = 9.9792015476736E+291 * b_A[rankR - 1];
                                            }

                                            s *= 9.9792015476736E+291;
                                            temp2 *= 9.9792015476736E+291;
                                        } while (!(std::abs(s) >= 1.0020841800044864E-292));

                                        s = rt_hypotd_snf(temp2, blas::xnrm2(mmi - 1, b_A, ii + 2));
                                        if (temp2 >= 0.0) {
                                            s = -s;
                                        }

                                        tau_data[b_i] = (s - temp2) / s;
                                        smax = 1.0 / (temp2 - s);
                                        for (rankR = minmana; rankR <= i; rankR++) {
                                            b_A[rankR - 1] = smax * b_A[rankR - 1];
                                        }

                                        for (rankR = 0; rankR <= pvt; rankR++) {
                                            s *= 1.0020841800044864E-292;
                                        }

                                        temp2 = s;
                                    }
                                    else {
                                        tau_data[b_i] = (s - b_A[ii]) / s;
                                        smax = 1.0 / (b_A[ii] - s);
                                        i = ii + mmi;
                                        for (rankR = minmana; rankR <= i; rankR++) {
                                            b_A[rankR - 1] = smax * b_A[rankR - 1];
                                        }

                                        temp2 = s;
                                    }
                                }
                            }

                            b_A[ii] = temp2;
                        }
                        else {
                            tau_data[b_i] = 0.0;
                        }

                        if (b_i + 1 < 3) {
                            int lastc;
                            int lastv;
                            temp2 = b_A[ii];
                            b_A[ii] = 1.0;
                            jA = (ii + ma) + 1;
                            if (tau_data[b_i] != 0.0) {
                                boolean_T exitg2;
                                lastv = mmi - 1;
                                minmana = (ii + mmi) - 1;
                                while ((lastv + 1 > 0) && (b_A[minmana] == 0.0)) {
                                    lastv--;
                                    minmana--;
                                }

                                lastc = 1 - b_i;
                                exitg2 = false;
                                while ((!exitg2) && (lastc + 1 > 0)) {
                                    int exitg1;
                                    minmana = jA + lastc * ma;
                                    pvt = minmana;
                                    do {
                                        exitg1 = 0;
                                        if (pvt <= minmana + lastv) {
                                            if (b_A[pvt - 1] != 0.0) {
                                                exitg1 = 1;
                                            }
                                            else {
                                                pvt++;
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
                                lastv = -1;
                                lastc = -1;
                            }

                            if (lastv + 1 > 0) {
                                if (lastc + 1 != 0) {
                                    if (0 <= lastc) {
                                        std::memset(&work[0], 0, (lastc + 1) * sizeof(double));
                                    }

                                    iy = 0;
                                    i = jA + ma * lastc;
                                    for (rankR = jA; ma < 0 ? rankR >= i : rankR <= i; rankR += ma)
                                    {
                                        ix = ii;
                                        smax = 0.0;
                                        minmana = rankR + lastv;
                                        for (pvt = rankR; pvt <= minmana; pvt++) {
                                            smax += b_A[pvt - 1] * b_A[ix];
                                            ix++;
                                        }

                                        work[iy] += smax;
                                        iy++;
                                    }
                                }

                                if (!(-tau_data[b_i] == 0.0)) {
                                    minmana = 0;
                                    for (iy = 0; iy <= lastc; iy++) {
                                        if (work[minmana] != 0.0) {
                                            smax = work[minmana] * -tau_data[b_i];
                                            ix = ii;
                                            i = lastv + jA;
                                            for (rankR = jA; rankR <= i; rankR++) {
                                                b_A[rankR - 1] = b_A[rankR - 1] + b_A[ix] * smax;
                                                ix++;
                                            }
                                        }

                                        minmana++;
                                        jA += ma;
                                    }
                                }
                            }

                            b_A[ii] = temp2;
                        }

                        for (iy = ip1; iy < 4; iy++) {
                            minmana = b_i + (iy - 1) * ma;
                            smax = vn1[iy - 1];
                            if (smax != 0.0) {
                                s = std::abs(b_A[minmana]) / smax;
                                s = 1.0 - s * s;
                                if (s < 0.0) {
                                    s = 0.0;
                                }

                                temp2 = smax / vn2[iy - 1];
                                temp2 = s * (temp2 * temp2);
                                if (temp2 <= 1.4901161193847656E-8) {
                                    if (b_i + 1 < m) {
                                        smax = blas::xnrm2(mmi - 1, b_A, minmana + 2);
                                        vn1[iy - 1] = smax;
                                        vn2[iy - 1] = smax;
                                    }
                                    else {
                                        vn1[iy - 1] = 0.0;
                                        vn2[iy - 1] = 0.0;
                                    }
                                }
                                else {
                                    vn1[iy - 1] = smax * std::sqrt(s);
                                }
                            }
                        }
                    }
                }
            }

            if (guard1) {
                jpvt[0] = 1;
                jpvt[1] = 2;
                jpvt[2] = 3;
            }

            rankR = 0;
            if (b_A.size(0) < 3) {
                minmn = b_A.size(0);
                minmana = 3;
            }
            else {
                minmn = 3;
                minmana = b_A.size(0);
            }

            if (minmn > 0) {
                smax = 2.2204460492503131E-15 * static_cast<double>(minmana);
                if (1.4901161193847656E-8 < smax) {
                    smax = 1.4901161193847656E-8;
                }

                smax *= std::abs(b_A[0]);
                while ((rankR < minmn) && (!(std::abs(b_A[rankR + b_A.size(0) * rankR]) <=
                    smax))) {
                    rankR++;
                }
            }

            b_B.set_size(B.size(0));
            minmana = B.size(0);
            for (i = 0; i < minmana; i++) {
                b_B[i] = B[i];
            }

            Y[0] = 0.0;
            Y[1] = 0.0;
            Y[2] = 0.0;
            m = b_A.size(0);
            minmana = b_A.size(0);
            if (minmana >= 3) {
                minmana = 3;
            }

            for (iy = 0; iy < minmana; iy++) {
                if (tau_data[iy] != 0.0) {
                    smax = b_B[iy];
                    i = iy + 2;
                    for (b_i = i; b_i <= m; b_i++) {
                        smax += b_A[(b_i + b_A.size(0) * iy) - 1] * b_B[b_i - 1];
                    }

                    smax *= tau_data[iy];
                    if (smax != 0.0) {
                        b_B[iy] = b_B[iy] - smax;
                        i = iy + 2;
                        for (b_i = i; b_i <= m; b_i++) {
                            b_B[b_i - 1] = b_B[b_i - 1] - b_A[(b_i + b_A.size(0) * iy) - 1] *
                                smax;
                        }
                    }
                }
            }

            for (b_i = 0; b_i < rankR; b_i++) {
                Y[jpvt[b_i] - 1] = b_B[b_i];
            }

            for (iy = rankR; iy >= 1; iy--) {
                minmana = jpvt[iy - 1] - 1;
                Y[minmana] /= b_A[(iy + b_A.size(0) * (iy - 1)) - 1];
                for (b_i = 0; b_i <= iy - 2; b_i++) {
                    pvt = jpvt[b_i] - 1;
                    Y[pvt] -= Y[minmana] * b_A[b_i + b_A.size(0) * (iy - 1)];
                }
            }
        }

        static void b_nullAssignment(::coder::array<double, 2U>& x, const ::coder::array<
            boolean_T, 2U>& idx)
        {
            int k;
            int k0;
            int nxin;
            int nxout;
            nxin = x.size(1);
            nxout = 0;
            k0 = idx.size(1);
            for (k = 0; k < k0; k++) {
                nxout += idx[k];
            }

            nxout = x.size(1) - nxout;
            k0 = -1;
            for (k = 0; k < nxin; k++) {
                if ((k + 1 > idx.size(1)) || (!idx[k])) {
                    k0++;
                    x[k0] = x[k];
                }
            }

            if (1 > nxout) {
                nxout = 0;
            }

            x.set_size(x.size(0), nxout);
        }

        //
        // Arguments    : ::coder::array<double, 2U> &x
        //                const ::coder::array<int, 2U> &idx
        // Return Type  : void
        //
        static void c_nullAssignment(::coder::array<double, 2U>& x, const ::coder::array<
            int, 2U>& idx)
        {
            array<double, 2U> b_x;
            array<boolean_T, 2U> b;
            int i;
            int nrows;
            int nrowx;
            nrowx = x.size(0);
            if (idx.size(1) == 1) {
                int b_i;
                nrows = x.size(0) - 1;
                i = idx[0];
                for (b_i = i; b_i <= nrows; b_i++) {
                    x[b_i - 1] = x[b_i];
                }

                for (b_i = i; b_i <= nrows; b_i++) {
                    x[(b_i + x.size(0)) - 1] = x[b_i + x.size(0)];
                }
            }
            else {
                int b_i;
                int k;
                b.set_size(1, x.size(0));
                nrows = x.size(0);
                for (i = 0; i < nrows; i++) {
                    b[i] = false;
                }

                i = idx.size(1);
                for (k = 0; k < i; k++) {
                    b[idx[k] - 1] = true;
                }

                nrows = 0;
                i = b.size(1);
                for (k = 0; k < i; k++) {
                    nrows += b[k];
                }

                nrows = x.size(0) - nrows;
                b_i = 0;
                for (k = 0; k < nrowx; k++) {
                    if ((k + 1 > b.size(1)) || (!b[k])) {
                        x[b_i] = x[k];
                        x[b_i + x.size(0)] = x[k + x.size(0)];
                        b_i++;
                    }
                }
            }

            if (1 > nrows) {
                nrows = 0;
            }

            b_x.set_size(nrows, 2);
            for (i = 0; i < nrows; i++) {
                b_x[i] = x[i];
            }

            for (i = 0; i < nrows; i++) {
                b_x[i + b_x.size(0)] = x[i + x.size(0)];
            }

            x.set_size(b_x.size(0), 2);
            nrows = b_x.size(0);
            for (i = 0; i < nrows; i++) {
                x[i] = b_x[i];
            }

            for (i = 0; i < nrows; i++) {
                x[i + x.size(0)] = b_x[i + b_x.size(0)];
            }
        }

        //
        // Arguments    : ::coder::array<double, 2U> &x
        //                const ::coder::array<boolean_T, 2U> &idx
        // Return Type  : void
        //
        static void d_nullAssignment(::coder::array<double, 2U>& x, const ::coder::array<
            boolean_T, 2U>& idx)
        {
            array<double, 2U> b_x;
            int i;
            int k;
            int nrows;
            int nrowx;
            nrowx = x.size(0);
            nrows = 0;
            i = idx.size(1);
            for (k = 0; k < i; k++) {
                nrows += idx[k];
            }

            nrows = x.size(0) - nrows;
            i = 0;
            for (k = 0; k < nrowx; k++) {
                if ((k + 1 > idx.size(1)) || (!idx[k])) {
                    x[i] = x[k];
                    x[i + x.size(0)] = x[k + x.size(0)];
                    i++;
                }
            }

            if (1 > nrows) {
                nrows = 0;
            }

            b_x.set_size(nrows, 2);
            for (i = 0; i < nrows; i++) {
                b_x[i] = x[i];
            }

            for (i = 0; i < nrows; i++) {
                b_x[i + b_x.size(0)] = x[i + x.size(0)];
            }

            x.set_size(b_x.size(0), 2);
            nrows = b_x.size(0);
            for (i = 0; i < nrows; i++) {
                x[i] = b_x[i];
            }

            for (i = 0; i < nrows; i++) {
                x[i + x.size(0)] = b_x[i + b_x.size(0)];
            }
        }

        //
        // Arguments    : ::coder::array<double, 2U> &x
        //                const ::coder::array<int, 2U> &idx
        // Return Type  : void
        //
        static void nullAssignment(::coder::array<double, 2U>& x, const ::coder::array<int,
            2U>& idx)
        {
            array<boolean_T, 2U> b;
            int k;
            int k0;
            int nxin;
            int nxout;
            nxin = x.size(1);
            b.set_size(1, x.size(1));
            nxout = x.size(1);
            for (k0 = 0; k0 < nxout; k0++) {
                b[k0] = false;
            }

            k0 = idx.size(1);
            for (k = 0; k < k0; k++) {
                b[idx[k] - 1] = true;
            }

            nxout = 0;
            k0 = b.size(1);
            for (k = 0; k < k0; k++) {
                nxout += b[k];
            }

            nxout = x.size(1) - nxout;
            k0 = -1;
            for (k = 0; k < nxin; k++) {
                if ((k + 1 > b.size(1)) || (!b[k])) {
                    k0++;
                    x[k0] = x[k];
                }
            }

            if (1 > nxout) {
                nxout = 0;
            }

            x.set_size(x.size(0), nxout);
        }
    }

  static void mean(const ::coder::array<double, 2U> &x, double y[6])
  {
    int vlen;
    vlen = x.size(1);
    if (x.size(1) == 0) {
      for (vlen = 0; vlen < 6; vlen++) {
        y[vlen] = 0.0;
      }
    } else {
      int j;
      for (j = 0; j < 6; j++) {
        y[j] = x[j];
      }

      for (int k = 2; k <= vlen; k++) {
        int xoffset;
        xoffset = (k - 1) * 6;
        for (j = 0; j < 6; j++) {
          y[j] += x[xoffset + j];
        }
      }
    }

    for (vlen = 0; vlen < 6; vlen++) {
      y[vlen] /= static_cast<double>(x.size(1));
    }
  }

  static void b_fscanf(double fileID, ::coder::array<double, 2U> &A)
  {
    FILE * filestar;
    array<double, 1U> bigA;
    double tmpOut_f1;
    int actualSize;
    int i;
    int numRead;
    int numWrittenTotal;
    signed char fileid;
    boolean_T exitg1;
    tmpOut_f1 = 0.0;

    // If formatSpec contains a combination of numeric and character specifiers, then fscanf converts each character to its numeric equivalent. This conversion occurs even when the format explicitly skips all numeric values (for example, formatSpec is '%*d %s'). 
    bigA.set_size(0);
    numRead = 1;
    numWrittenTotal = 0;
    fileid = static_cast<signed char>(rt_roundd_snf(fileID));
    if ((fileid < 0) || (fileID != fileid)) {
      fileid = -1;
    }

    if (fileid >= 3) {
      filestar = eml_openfiles[fileid - 3];
    } else if (fileid == 0) {
      filestar = stdin;
    } else if (fileid == 1) {
      filestar = stdout;
    } else if (fileid == 2) {
      filestar = stderr;
    } else {
      filestar = NULL;
    }

    exitg1 = false;
    while ((!exitg1) && (numRead > 0)) {
      boolean_T incompleteRead;
      actualSize = -1;
      numRead = fscanf(filestar, "%lf%n", &tmpOut_f1, &actualSize);
      if (actualSize != -1) {
        numRead++;
      }

      incompleteRead = (numRead == 0);
      if (numRead > 0) {
        i = bigA.size(0);
        bigA.set_size((bigA.size(0) + 1));
        bigA[i] = tmpOut_f1;
        numWrittenTotal++;
        incompleteRead = ((2 > numRead) || incompleteRead);
      }

      if (incompleteRead) {
        exitg1 = true;
      }
    }

    if (numWrittenTotal > 0) {
      if (numWrittenTotal > 6) {
        i = numWrittenTotal - numWrittenTotal / 6 * 6;
        if (i == 0) {
          if (numWrittenTotal > bigA.size(0)) {
            i = bigA.size(0);
          } else {
            i = numWrittenTotal;
          }

          if (1 > i) {
            i = 0;
          }

          A.set_size(6, (i / 6));
          actualSize = 6 * (i / 6);
          for (i = 0; i < actualSize; i++) {
            A[i] = bigA[i];
          }
        } else {
          actualSize = static_cast<int>(static_cast<double>(numWrittenTotal) /
            6.0);
          if (i > 0) {
            actualSize++;
          }

          A.set_size(6, actualSize);
          actualSize *= 6;
          for (i = 0; i < actualSize; i++) {
            A[i] = 0.0;
          }

          for (i = 0; i < numWrittenTotal; i++) {
            A[i] = bigA[i];
          }
        }
      } else {
        A.set_size(numWrittenTotal, 1);
        for (i = 0; i < numWrittenTotal; i++) {
          A[i] = bigA[i];
        }
      }
    } else {
      A.set_size(0, 1);
    }
  }

  static signed char filedata()
  {
    int k;
    signed char f;
    boolean_T exitg1;
    f = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 20)) {
      if (eml_openfiles[k] == NULL) {
        f = static_cast<signed char>(k + 1);
        exitg1 = true;
      } else {
        k++;
      }
    }

    return f;
  }

  //
  // Arguments    : double fid
  // Return Type  : int
  //
  static int cfclose(double fid)
  {
    FILE * filestar;
    int st;
    signed char b_fileid;
    signed char fileid;
    st = -1;
    fileid = static_cast<signed char>(rt_roundd_snf(fid));
    if ((fileid < 0) || (fid != fileid)) {
      fileid = -1;
    }

    b_fileid = fileid;
    if (fileid < 0) {
      b_fileid = -1;
    }

    if (b_fileid >= 3) {
      filestar = eml_openfiles[b_fileid - 3];
    } else if (b_fileid == 0) {
      filestar = stdin;
    } else if (b_fileid == 1) {
      filestar = stdout;
    } else if (b_fileid == 2) {
      filestar = stderr;
    } else {
      filestar = NULL;
    }

    if ((filestar != NULL) && (fileid >= 3)) {
      int cst;
      cst = fclose(filestar);
      if (cst == 0) {
        st = 0;
        eml_openfiles[fileid - 3] = NULL;
        eml_autoflush[fileid - 3] = true;
      }
    }

    return st;
  }

  //
  // Arguments    : const ::coder::array<char, 2U> &cfilename
  //                const char *cpermission
  // Return Type  : signed char
  //
  static signed char cfopen(const ::coder::array<char, 2U> &cfilename, const char
                     *cpermission)
  {
    array<char, 2U> ccfilename;
    signed char fileid;
    signed char j;
    fileid = -1;
    j = filedata();
    if (j >= 1) {
      FILE * filestar;
      int i;
      int loop_ub;
      ccfilename.set_size(1, (cfilename.size(1) + 1));
      loop_ub = cfilename.size(1);
      for (i = 0; i < loop_ub; i++) {
        ccfilename[i] = cfilename[i];
      }

      ccfilename[cfilename.size(1)] = '\x00';
      filestar = fopen(&ccfilename[0], cpermission);
      if (filestar != NULL) {
        eml_openfiles[j - 1] = filestar;
        eml_autoflush[j - 1] = true;
        i = j + 2;
        if (j + 2 > 127) {
          i = 127;
        }

        fileid = static_cast<signed char>(i);
      }
    }

    return fileid;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
}



namespace coder
{
  static void mldivide(const ::coder::array<double, 2U> &A, const ::coder::array<double,
                1U> &B, double Y[3])
  {
    double A_data[9];
    if ((A.size(0) == 0) || (B.size(0) == 0)) {
      Y[0] = 0.0;
      Y[1] = 0.0;
      Y[2] = 0.0;
    } else if (A.size(0) == 3) {
      double a21;
      double maxval;
      int r1;
      int r2;
      int r3;
      r1 = A.size(0) * A.size(1);
      for (r2 = 0; r2 < r1; r2++) {
        A_data[r2] = A[r2];
      }

      r1 = 0;
      r2 = 1;
      r3 = 2;
      maxval = std::abs(A[0]);
      a21 = std::abs(A[1]);
      if (a21 > maxval) {
        maxval = a21;
        r1 = 1;
        r2 = 0;
      }

      if (std::abs(A[2]) > maxval) {
        r1 = 2;
        r2 = 1;
        r3 = 0;
      }

      A_data[r2] = A[r2] / A[r1];
      A_data[r3] /= A_data[r1];
      A_data[r2 + 3] -= A_data[r2] * A_data[r1 + 3];
      A_data[r3 + 3] -= A_data[r3] * A_data[r1 + 3];
      A_data[r2 + 6] -= A_data[r2] * A_data[r1 + 6];
      A_data[r3 + 6] -= A_data[r3] * A_data[r1 + 6];
      if (std::abs(A_data[r3 + 3]) > std::abs(A_data[r2 + 3])) {
        int rtemp;
        rtemp = r2;
        r2 = r3;
        r3 = rtemp;
      }

      A_data[r3 + 3] /= A_data[r2 + 3];
      A_data[r3 + 6] -= A_data[r3 + 3] * A_data[r2 + 6];
      Y[1] = B[r2] - B[r1] * A_data[r2];
      Y[2] = (B[r3] - B[r1] * A_data[r3]) - Y[1] * A_data[r3 + 3];
      Y[2] /= A_data[r3 + 6];
      Y[0] = B[r1] - Y[2] * A_data[r1 + 6];
      Y[1] -= Y[2] * A_data[r2 + 6];
      Y[1] /= A_data[r2 + 3];
      Y[0] -= Y[1] * A_data[r1 + 3];
      Y[0] /= A_data[r1];
    } else {
      internal::qrsolve(A, B, Y);
    }
  }

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
    } else {
      t = absxk / 3.3121686421112381E-170;
      y = t * t;
    }

    absxk = std::abs(x[1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    absxk = std::abs(x[2]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    return scale * std::sqrt(y);
  }
}
// Function Definitions
//
// Arguments    : const coder::array<char, 2U> &forceFile
//                const coder::array<char, 2U> &poseFile
//                double *mass
//                double r[3]
//                double offset[6]
// Return Type  : void
//
void force_calibration(const coder::array<char, 2U> &forceFile, const coder::
  array<char, 2U> &poseFile, double *mass, double r[3], double offset[6])
{
  coder::array<double, 2U> A;
  coder::array<double, 2U> M;
  coder::array<double, 2U> force;
  coder::array<double, 2U> outliers;
  coder::array<double, 2U> pose;
  coder::array<double, 1U> b;
  coder::array<int, 2U> b_r;
  coder::array<boolean_T, 2U> ind;
  double Ri[9];
  double b_so_w_hat[9];
  double so_w_hat[9];
  double w_hat[3];
  double y[3];
  double absxk;
  double b_i;
  double b_j;
  double cnt;
  double m;
  double scale;
  double t;
  double theta;
  int aoffset;
  int i;
  int i1;
  int input_sizes_idx_1;
  int k;
  int sizes_idx_1;
  signed char b_I[9];
  signed char fileid;
  boolean_T empty_non_axis_sizes;
  if (!isInitialized_force_calibration) {
    force_calibration_initialize();
  }

  fileid = coder::cfopen(forceFile, "rb");

  //  fx fy fz mx my mz
  coder::b_fscanf(static_cast<double>(fileid), force);
  coder::cfclose(static_cast<double>(fileid));
  fileid = coder::cfopen(poseFile, "rb");

  //  x y z rx ry rz
  coder::b_fscanf(static_cast<double>(fileid), pose);
  coder::cfclose(static_cast<double>(fileid));
  A.set_size(pose.size(1), pose.size(0));
  aoffset = pose.size(0);
  for (i = 0; i < aoffset; i++) {
    input_sizes_idx_1 = pose.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      A[i1 + A.size(0) * i] = pose[i + pose.size(0) * i1];
    }
  }

  M.set_size(force.size(1), force.size(0));
  aoffset = force.size(0);
  for (i = 0; i < aoffset; i++) {
    input_sizes_idx_1 = force.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      M[i1 + M.size(0) * i] = force[i + force.size(0) * i1];
    }
  }

  if ((A.size(0) != 0) && (A.size(1) != 0)) {
    aoffset = A.size(0);
  } else if ((M.size(0) != 0) && (M.size(1) != 0)) {
    aoffset = M.size(0);
  } else {
    aoffset = A.size(0);
    if (aoffset <= 0) {
      aoffset = 0;
    }

    if (M.size(0) > aoffset) {
      aoffset = M.size(0);
    }
  }

  empty_non_axis_sizes = (aoffset == 0);
  if (empty_non_axis_sizes || ((A.size(0) != 0) && (A.size(1) != 0))) {
    input_sizes_idx_1 = A.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }

  if (empty_non_axis_sizes || ((M.size(0) != 0) && (M.size(1) != 0))) {
    sizes_idx_1 = M.size(1);
  } else {
    sizes_idx_1 = 0;
  }

  pose.set_size(aoffset, (input_sizes_idx_1 + sizes_idx_1));
  for (i = 0; i < input_sizes_idx_1; i++) {
    for (i1 = 0; i1 < aoffset; i1++) {
      pose[i1 + pose.size(0) * i] = A[i1 + aoffset * i];
    }
  }

  for (i = 0; i < sizes_idx_1; i++) {
    for (i1 = 0; i1 < aoffset; i1++) {
      pose[i1 + pose.size(0) * (i + input_sizes_idx_1)] = M[i1 + aoffset * i];
    }
  }

  m = static_cast<double>(pose.size(0)) * (static_cast<double>(pose.size(0)) -
    1.0) / 2.0;
  M.set_size(1, (static_cast<int>(m)));
  aoffset = static_cast<int>(m);
  for (i = 0; i < aoffset; i++) {
    M[i] = 0.0;
  }

  cnt = 1.0;
  outliers.set_size((static_cast<int>(m)), 2);
  aoffset = static_cast<int>(m) << 1;
  for (i = 0; i < aoffset; i++) {
    outliers[i] = 0.0;
  }

  i = pose.size(0);
  for (sizes_idx_1 = 0; sizes_idx_1 <= i - 2; sizes_idx_1++) {
    i1 = pose.size(0) - sizes_idx_1;
    for (int j = 0; j <= i1 - 2; j++) {
      b_j = (static_cast<unsigned int>(sizes_idx_1) + j) + 2U;
      w_hat[0] = pose[sizes_idx_1 + pose.size(0) * 3];
      w_hat[1] = pose[sizes_idx_1 + pose.size(0) * 4];
      w_hat[2] = pose[sizes_idx_1 + pose.size(0) * 5];
      theta = coder::b_norm(w_hat);
      if (theta <= 2.2204460492503131E-16) {
        std::memset(&Ri[0], 0, 9U * sizeof(double));
        Ri[0] = 1.0;
        Ri[4] = 1.0;
        Ri[8] = 1.0;
      } else {
        w_hat[0] /= theta;
        w_hat[1] /= theta;
        w_hat[2] /= theta;

        //  w - > [w]
        //  x: vector or m by 3 matrix
        //  X: 3 by 3 by m matrix
        so_w_hat[0] = 0.0;
        so_w_hat[3] = -w_hat[2];
        so_w_hat[6] = w_hat[1];
        so_w_hat[1] = w_hat[2];
        so_w_hat[4] = 0.0;
        so_w_hat[7] = -w_hat[0];
        so_w_hat[2] = -w_hat[1];
        so_w_hat[5] = w_hat[0];
        so_w_hat[8] = 0.0;
        scale = std::sin(theta);
        theta = 1.0 - std::cos(theta);
        for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 9; input_sizes_idx_1++)
        {
          b_I[input_sizes_idx_1] = 0;
        }

        for (k = 0; k < 3; k++) {
          b_I[k + 3 * k] = 1;
          for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 3; input_sizes_idx_1++)
          {
            Ri[k + 3 * input_sizes_idx_1] = (so_w_hat[k] * so_w_hat[3 *
              input_sizes_idx_1] + so_w_hat[k + 3] * so_w_hat[3 *
              input_sizes_idx_1 + 1]) + so_w_hat[k + 6] * so_w_hat[3 *
              input_sizes_idx_1 + 2];
          }
        }

        for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 9; input_sizes_idx_1++)
        {
          Ri[input_sizes_idx_1] = (static_cast<double>(b_I[input_sizes_idx_1]) +
            scale * so_w_hat[input_sizes_idx_1]) + theta * Ri[input_sizes_idx_1];
        }
      }

      w_hat[0] = pose[(static_cast<int>(b_j) + pose.size(0) * 3) - 1];
      w_hat[1] = pose[(static_cast<int>(b_j) + pose.size(0) * 4) - 1];
      w_hat[2] = pose[(static_cast<int>(b_j) + pose.size(0) * 5) - 1];
      theta = coder::b_norm(w_hat);
      if (theta <= 2.2204460492503131E-16) {
        std::memset(&so_w_hat[0], 0, 9U * sizeof(double));
        so_w_hat[0] = 1.0;
        so_w_hat[4] = 1.0;
        so_w_hat[8] = 1.0;
      } else {
        w_hat[0] /= theta;
        w_hat[1] /= theta;
        w_hat[2] /= theta;

        //  w - > [w]
        //  x: vector or m by 3 matrix
        //  X: 3 by 3 by m matrix
        so_w_hat[0] = 0.0;
        so_w_hat[3] = -w_hat[2];
        so_w_hat[6] = w_hat[1];
        so_w_hat[1] = w_hat[2];
        so_w_hat[4] = 0.0;
        so_w_hat[7] = -w_hat[0];
        so_w_hat[2] = -w_hat[1];
        so_w_hat[5] = w_hat[0];
        so_w_hat[8] = 0.0;
        scale = std::sin(theta);
        theta = 1.0 - std::cos(theta);
        for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 9; input_sizes_idx_1++)
        {
          b_I[input_sizes_idx_1] = 0;
        }

        for (k = 0; k < 3; k++) {
          b_I[k + 3 * k] = 1;
          for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 3; input_sizes_idx_1++)
          {
            b_so_w_hat[k + 3 * input_sizes_idx_1] = (so_w_hat[k] * so_w_hat[3 *
              input_sizes_idx_1] + so_w_hat[k + 3] * so_w_hat[3 *
              input_sizes_idx_1 + 1]) + so_w_hat[k + 6] * so_w_hat[3 *
              input_sizes_idx_1 + 2];
          }
        }

        for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 9; input_sizes_idx_1++)
        {
          so_w_hat[input_sizes_idx_1] = (static_cast<double>
            (b_I[input_sizes_idx_1]) + scale * so_w_hat[input_sizes_idx_1]) +
            theta * b_so_w_hat[input_sizes_idx_1];
        }
      }

      for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 9; input_sizes_idx_1++) {
        Ri[input_sizes_idx_1] -= so_w_hat[input_sizes_idx_1];
      }

      theta = 0.0;
      scale = 3.3121686421112381E-170;
      for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 3; input_sizes_idx_1++) {
        aoffset = input_sizes_idx_1 * 3;
        absxk = std::abs((Ri[aoffset] * 0.0 + Ri[aoffset + 1] * 0.0) +
                         -Ri[aoffset + 2]);
        if (absxk > scale) {
          t = scale / absxk;
          theta = theta * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          theta += t * t;
        }
      }

      theta = scale * std::sqrt(theta);
      if (!(theta < 0.2)) {
        outliers[static_cast<int>(cnt) - 1] = static_cast<double>(sizes_idx_1) +
          1.0;
        outliers[(static_cast<int>(cnt) + outliers.size(0)) - 1] = b_j;
        w_hat[0] = pose[sizes_idx_1 + pose.size(0) * 6] - pose[(static_cast<int>
          (b_j) + pose.size(0) * 6) - 1];
        w_hat[1] = pose[sizes_idx_1 + pose.size(0) * 7] - pose[(static_cast<int>
          (b_j) + pose.size(0) * 7) - 1];
        w_hat[2] = pose[sizes_idx_1 + pose.size(0) * 8] - pose[(static_cast<int>
          (b_j) + pose.size(0) * 8) - 1];
        M[static_cast<int>(cnt) - 1] = coder::b_norm(w_hat) / theta;
        cnt++;
      }
    }
  }

  b_r.set_size(1, (static_cast<int>(static_cast<double>(outliers.size(0)) - cnt)
                   + 1));
  aoffset = static_cast<int>(static_cast<double>(outliers.size(0)) - cnt);
  for (i = 0; i <= aoffset; i++) {
    b_r[i] = static_cast<int>(cnt + static_cast<double>(i));
  }

  coder::internal::c_nullAssignment(outliers, b_r);
  b_r.set_size(1, (static_cast<int>(static_cast<double>(M.size(1)) - cnt) + 1));
  aoffset = static_cast<int>(static_cast<double>(M.size(1)) - cnt);
  for (i = 0; i <= aoffset; i++) {
    b_r[i] = static_cast<int>(cnt + static_cast<double>(i));
  }

  coder::internal::nullAssignment(M, b_r);

  //  plot(M);
  aoffset = M.size(1);
  if (M.size(1) == 0) {
    theta = 0.0;
  } else {
    theta = M[0];
    for (k = 2; k <= aoffset; k++) {
      theta += M[k - 1];
    }
  }

  *mass = theta / static_cast<double>(M.size(1));
  force.set_size(1, M.size(1));
  aoffset = M.size(0) * M.size(1);
  for (i = 0; i < aoffset; i++) {
    force[i] = M[i] - *mass;
  }

  aoffset = force.size(1);
  A.set_size(1, force.size(1));
  for (k = 0; k < aoffset; k++) {
    A[k] = std::abs(force[k]);
  }

  ind.set_size(1, A.size(1));
  absxk = 3.0 * coder::b_std(M);
  aoffset = A.size(0) * A.size(1);
  for (i = 0; i < aoffset; i++) {
    ind[i] = (A[i] > absxk);
  }

  if (ind.size(1) != 0) {
    coder::internal::d_nullAssignment(outliers, ind);
    coder::internal::b_nullAssignment(M, ind);
    m = outliers.size(0);
  }

  force.set_size(6, (static_cast<int>(m)));
  aoffset = 6 * static_cast<int>(m);
  for (i = 0; i < aoffset; i++) {
    force[i] = 0.0;
  }

  aoffset = static_cast<int>(3.0 * m);
  A.set_size(aoffset, 3);
  input_sizes_idx_1 = aoffset * 3;
  for (i = 0; i < input_sizes_idx_1; i++) {
    A[i] = 0.0;
  }

  b.set_size(aoffset);
  for (i = 0; i < aoffset; i++) {
    b[i] = 0.0;
  }

  aoffset = M.size(1);
  if (M.size(1) == 0) {
    theta = 0.0;
  } else {
    theta = M[0];
    for (k = 2; k <= aoffset; k++) {
      theta += M[k - 1];
    }
  }

  *mass = theta / static_cast<double>(M.size(1));
  cnt = 1.0;
  i = static_cast<int>(m);
  for (k = 0; k < i; k++) {
    b_i = outliers[k];
    b_j = outliers[k + outliers.size(0)];

    //  t = t(:);
    theta = pose[(static_cast<int>(b_i) + pose.size(0) * 3) - 1];
    w_hat[0] = theta;
    scale = pose[(static_cast<int>(b_i) + pose.size(0) * 4) - 1];
    w_hat[1] = scale;
    absxk = pose[(static_cast<int>(b_i) + pose.size(0) * 5) - 1];
    w_hat[2] = absxk;
    t = coder::b_norm(w_hat);
    if (0.0 == t) {
      std::memset(&Ri[0], 0, 9U * sizeof(double));
      Ri[0] = 1.0;
      Ri[4] = 1.0;
      Ri[8] = 1.0;
    } else {
      so_w_hat[0] = 0.0;
      so_w_hat[3] = -absxk;
      so_w_hat[6] = scale;
      so_w_hat[1] = absxk;
      so_w_hat[4] = 0.0;
      so_w_hat[7] = -theta;
      so_w_hat[2] = -scale;
      so_w_hat[5] = pose[(static_cast<int>(b_i) + pose.size(0) * 3) - 1];
      so_w_hat[8] = 0.0;
      scale = std::sin(t) / t;
      theta = (1.0 - std::cos(t)) / (t * t);
      for (i1 = 0; i1 < 9; i1++) {
        b_I[i1] = 0;
      }

      for (aoffset = 0; aoffset < 3; aoffset++) {
        b_I[aoffset + 3 * aoffset] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          Ri[aoffset + 3 * i1] = (so_w_hat[aoffset] * so_w_hat[3 * i1] +
            so_w_hat[aoffset + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[aoffset + 6]
            * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        Ri[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) + theta *
          Ri[i1];
      }
    }

    //  t = t(:);
    theta = pose[(static_cast<int>(b_j) + pose.size(0) * 3) - 1];
    w_hat[0] = theta;
    scale = pose[(static_cast<int>(b_j) + pose.size(0) * 4) - 1];
    w_hat[1] = scale;
    absxk = pose[(static_cast<int>(b_j) + pose.size(0) * 5) - 1];
    w_hat[2] = absxk;
    t = coder::b_norm(w_hat);
    if (0.0 == t) {
      std::memset(&so_w_hat[0], 0, 9U * sizeof(double));
      so_w_hat[0] = 1.0;
      so_w_hat[4] = 1.0;
      so_w_hat[8] = 1.0;
    } else {
      so_w_hat[0] = 0.0;
      so_w_hat[3] = -absxk;
      so_w_hat[6] = scale;
      so_w_hat[1] = absxk;
      so_w_hat[4] = 0.0;
      so_w_hat[7] = -theta;
      so_w_hat[2] = -scale;
      so_w_hat[5] = theta;
      so_w_hat[8] = 0.0;
      scale = std::sin(t) / t;
      theta = (1.0 - std::cos(t)) / (t * t);
      for (i1 = 0; i1 < 9; i1++) {
        b_I[i1] = 0;
      }

      for (aoffset = 0; aoffset < 3; aoffset++) {
        b_I[aoffset + 3 * aoffset] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          b_so_w_hat[aoffset + 3 * i1] = (so_w_hat[aoffset] * so_w_hat[3 * i1] +
            so_w_hat[aoffset + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[aoffset + 6]
            * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        so_w_hat[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) +
          theta * b_so_w_hat[i1];
      }
    }

    absxk = 3.0 * cnt;
    b[static_cast<int>(absxk + -2.0) - 1] = -(pose[(static_cast<int>(b_i) +
      pose.size(0) * 9) - 1] - pose[(static_cast<int>(b_j) + pose.size(0) * 9) -
      1]);
    b[static_cast<int>(absxk + -1.0) - 1] = -(pose[(static_cast<int>(b_i) +
      pose.size(0) * 10) - 1] - pose[(static_cast<int>(b_j) + pose.size(0) * 10)
      - 1]);
    b[static_cast<int>(absxk) - 1] = -(pose[(static_cast<int>(b_i) + pose.size(0)
      * 11) - 1] - pose[(static_cast<int>(b_j) + pose.size(0) * 11) - 1]);
    for (i1 = 0; i1 < 9; i1++) {
      Ri[i1] -= so_w_hat[i1];
    }

    for (sizes_idx_1 = 0; sizes_idx_1 < 3; sizes_idx_1++) {
      aoffset = sizes_idx_1 * 3;
      y[sizes_idx_1] = (Ri[aoffset] * 0.0 + Ri[aoffset + 1] * 0.0) + -Ri[aoffset
        + 2];
    }

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    A[static_cast<int>(absxk + -2.0) - 1] = 0.0 * *mass;
    A[(static_cast<int>(absxk + -2.0) + A.size(0)) - 1] = -y[2] * *mass;
    A[(static_cast<int>(absxk + -2.0) + A.size(0) * 2) - 1] = y[1] * *mass;
    A[static_cast<int>(absxk + -1.0) - 1] = y[2] * *mass;
    A[(static_cast<int>(absxk + -1.0) + A.size(0)) - 1] = 0.0 * *mass;
    A[(static_cast<int>(absxk + -1.0) + A.size(0) * 2) - 1] = -y[0] * *mass;
    A[static_cast<int>(absxk) - 1] = -y[1] * *mass;
    A[(static_cast<int>(absxk) + A.size(0)) - 1] = y[0] * *mass;
    A[(static_cast<int>(absxk) + A.size(0) * 2) - 1] = 0.0 * *mass;
    cnt++;
  }

  //  for i = 1 : n - 1
  //      for j = i + 1 : n
  //          Ri = RotationByAxisAngleRep(data(i,4:6));
  //          Rj = RotationByAxisAngleRep(data(j,4:6));
  //          Mi = data(i,10:12);
  //          Mj = data(j,10:12);
  //          b(3 * cnt - 2 : 3 * cnt) = -(Mi - Mj)';
  //          A(3 * cnt - 2 : 3 * cnt, :) = so_w((Ri - Rj)' * [0,0,-1]') * mass; 
  //          cnt = cnt + 1;
  //      end
  //  end
  coder::mldivide(A, b, r);
  cnt = 1.0;
  i = static_cast<int>(m);
  for (k = 0; k < i; k++) {
    b_i = outliers[k];
    b_j = outliers[k + outliers.size(0)];

    //  t = t(:);
    theta = pose[(static_cast<int>(b_i) + pose.size(0) * 3) - 1];
    w_hat[0] = theta;
    scale = pose[(static_cast<int>(b_i) + pose.size(0) * 4) - 1];
    w_hat[1] = scale;
    absxk = pose[(static_cast<int>(b_i) + pose.size(0) * 5) - 1];
    w_hat[2] = absxk;
    t = coder::b_norm(w_hat);
    if (0.0 == t) {
      std::memset(&Ri[0], 0, 9U * sizeof(double));
      Ri[0] = 1.0;
      Ri[4] = 1.0;
      Ri[8] = 1.0;
    } else {
      so_w_hat[0] = 0.0;
      so_w_hat[3] = -absxk;
      so_w_hat[6] = scale;
      so_w_hat[1] = absxk;
      so_w_hat[4] = 0.0;
      so_w_hat[7] = -theta;
      so_w_hat[2] = -scale;
      so_w_hat[5] = theta;
      so_w_hat[8] = 0.0;
      scale = std::sin(t) / t;
      theta = (1.0 - std::cos(t)) / (t * t);
      for (i1 = 0; i1 < 9; i1++) {
        b_I[i1] = 0;
      }

      for (aoffset = 0; aoffset < 3; aoffset++) {
        b_I[aoffset + 3 * aoffset] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          Ri[aoffset + 3 * i1] = (so_w_hat[aoffset] * so_w_hat[3 * i1] +
            so_w_hat[aoffset + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[aoffset + 6]
            * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        Ri[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) + theta *
          Ri[i1];
      }
    }

    //  t = t(:);
    theta = pose[(static_cast<int>(b_j) + pose.size(0) * 3) - 1];
    w_hat[0] = theta;
    scale = pose[(static_cast<int>(b_j) + pose.size(0) * 4) - 1];
    w_hat[1] = scale;
    absxk = pose[(static_cast<int>(b_j) + pose.size(0) * 5) - 1];
    w_hat[2] = absxk;
    t = coder::b_norm(w_hat);
    if (0.0 == t) {
      std::memset(&so_w_hat[0], 0, 9U * sizeof(double));
      so_w_hat[0] = 1.0;
      so_w_hat[4] = 1.0;
      so_w_hat[8] = 1.0;
    } else {
      so_w_hat[0] = 0.0;
      so_w_hat[3] = -pose[(static_cast<int>(b_j) + pose.size(0) * 5) - 1];
      so_w_hat[6] = scale;
      so_w_hat[1] = absxk;
      so_w_hat[4] = 0.0;
      so_w_hat[7] = -pose[(static_cast<int>(b_j) + pose.size(0) * 3) - 1];
      so_w_hat[2] = -pose[(static_cast<int>(b_j) + pose.size(0) * 4) - 1];
      so_w_hat[5] = theta;
      so_w_hat[8] = 0.0;
      scale = std::sin(t) / t;
      theta = (1.0 - std::cos(t)) / (t * t);
      for (i1 = 0; i1 < 9; i1++) {
        b_I[i1] = 0;
      }

      for (aoffset = 0; aoffset < 3; aoffset++) {
        b_I[aoffset + 3 * aoffset] = 1;
        for (i1 = 0; i1 < 3; i1++) {
          b_so_w_hat[aoffset + 3 * i1] = (so_w_hat[aoffset] * so_w_hat[3 * i1] +
            so_w_hat[aoffset + 3] * so_w_hat[3 * i1 + 1]) + so_w_hat[aoffset + 6]
            * so_w_hat[3 * i1 + 2];
        }
      }

      for (i1 = 0; i1 < 9; i1++) {
        so_w_hat[i1] = (static_cast<double>(b_I[i1]) + scale * so_w_hat[i1]) +
          theta * b_so_w_hat[i1];
      }
    }

    for (i1 = 0; i1 < 9; i1++) {
      Ri[i1] += so_w_hat[i1];
    }

    for (sizes_idx_1 = 0; sizes_idx_1 < 3; sizes_idx_1++) {
      absxk = Ri[3 * sizes_idx_1];
      theta = Ri[3 * sizes_idx_1 + 1];
      scale = Ri[3 * sizes_idx_1 + 2];
      force[sizes_idx_1 + 6 * (static_cast<int>(cnt) - 1)] = ((pose[(
        static_cast<int>(b_i) + pose.size(0) * (sizes_idx_1 + 6)) - 1] + pose[(
        static_cast<int>(b_j) + pose.size(0) * (sizes_idx_1 + 6)) - 1]) -
        ((*mass * absxk * 0.0 + *mass * theta * 0.0) + -(*mass * scale))) / 2.0;
      y[sizes_idx_1] = (absxk * 0.0 + theta * 0.0) + -scale;
    }

    force[6 * (static_cast<int>(cnt) - 1) + 3] = ((pose[(static_cast<int>(b_i) +
      pose.size(0) * 9) - 1] + pose[(static_cast<int>(b_j) + pose.size(0) * 9) -
      1]) - *mass * (r[1] * y[2] - r[2] * y[1])) / 2.0;
    force[6 * (static_cast<int>(cnt) - 1) + 4] = ((pose[(static_cast<int>(b_i) +
      pose.size(0) * 10) - 1] + pose[(static_cast<int>(b_j) + pose.size(0) * 10)
      - 1]) - *mass * (r[2] * y[0] - r[0] * y[2])) / 2.0;
    force[6 * (static_cast<int>(cnt) - 1) + 5] = ((pose[(static_cast<int>(b_i) +
      pose.size(0) * 11) - 1] + pose[(static_cast<int>(b_j) + pose.size(0) * 11)
      - 1]) - *mass * (r[0] * y[1] - r[1] * y[0])) / 2.0;
    cnt++;
  }

  //  for i = 1 : n - 1
  //      for j = i + 1 : n
  //          Ri = RotationByAxisAngleRep(data(i,4:6));
  //          Rj = RotationByAxisAngleRep(data(j,4:6));
  //          Fi = data(i,7:9);
  //          Fj = data(j,7:9);
  //          Mi = data(i,10:12);
  //          Mj = data(j,10:12);
  //          offsets(1:3, cnt) = (Fi' + Fj' - mass * (Ri + Rj)' * [0, 0, -1]') / 2; 
  //          offsets(4:6, cnt) = (Mi' + Mj' - mass * cross(r, (Ri + Rj)' * [0, 0, -1]')) / 2; 
  //          cnt = cnt + 1;
  //      end
  //  end
  //  figure;
  //  plot(offsets');
  coder::mean(force, offset);

  //  fid = fopen('sensor_calib.txt', 'w');
  //  fprintf(fid, '%f ', [mass, r', offset']);
  //  fclose(fid);
}

//
// File trailer for force_calibration.cpp
//
// [EOF]
//
