//
// File: svd.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 25-Mar-2022 14:44:47
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 2U> &U
//                ::coder::array<double, 1U> &s
//                ::coder::array<double, 2U> &V
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      static void xaxpy(int n, double a, const ::coder::array<double, 2U> &x, int ix0, ::coder::array<double, 1U> &y, int iy0)
      {
        if ((n >= 1) && (!(a == 0.0)))
        {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++)
          {
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
      static void xaxpy(int n, double a, const ::coder::array<double, 1U> &x, int ix0, ::coder::array<double, 2U> &y, int iy0)
      {
        if ((n >= 1) && (!(a == 0.0)))
        {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++)
          {
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
      //                ::coder::array<double, 2U> &y
      //                int iy0
      // Return Type  : void
      //
      static void xaxpy(int n, double a, int ix0, ::coder::array<double, 2U> &y, int iy0)
      {
        if ((n >= 1) && (!(a == 0.0)))
        {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++)
          {
            y[iy] = y[iy] + a * y[ix];
            ix++;
            iy++;
          }
        }
      }
      static double xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1)
        {
          if (n == 1)
          {
            y = std::abs(x[ix0 - 1]);
          }
          else
          {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++)
            {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale)
              {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              }
              else
              {
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
      //                const ::coder::array<double, 1U> &x
      //                int ix0
      // Return Type  : double
      //
      static double xnrm2(int n, const ::coder::array<double, 1U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1)
        {
          if (n == 1)
          {
            y = std::abs(x[ix0 - 1]);
          }
          else
          {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++)
            {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale)
              {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              }
              else
              {
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
      static void xrotg(double *a, double *b, double *c, double *s)
      {
        double absa;
        double absb;
        double roe;
        double scale;
        roe = *b;
        absa = std::abs(*a);
        absb = std::abs(*b);
        if (absa > absb)
        {
          roe = *a;
        }

        scale = absa + absb;
        if (scale == 0.0)
        {
          *s = 0.0;
          *c = 1.0;
          *a = 0.0;
          *b = 0.0;
        }
        else
        {
          double ads;
          double bds;
          ads = absa / scale;
          bds = absb / scale;
          scale *= std::sqrt(ads * ads + bds * bds);
          if (roe < 0.0)
          {
            scale = -scale;
          }

          *c = *a / scale;
          *s = *b / scale;
          if (absa > absb)
          {
            *b = *s;
          }
          else if (*c != 0.0)
          {
            *b = 1.0 / *c;
          }
          else
          {
            *b = 1.0;
          }

          *a = scale;
        }
      }

    }

    void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U,
             ::coder::array<double, 1U> &s, ::coder::array<double, 2U> &V)
    {
      array<double, 2U> Vf;
      array<double, 2U> b_A;
      array<double, 1U> b_s;
      array<double, 1U> e;
      array<double, 1U> work;
      double nrm;
      double r;
      double scale;
      double sm;
      int i;
      int k;
      int minnp;
      int n;
      int ns;
      int p;
      int qjj;
      b_A.set_size(A.size(0), A.size(1));
      ns = A.size(0) * A.size(1);
      for (i = 0; i < ns; i++)
      {
        b_A[i] = A[i];
      }

      n = A.size(0);
      p = A.size(1);
      qjj = A.size(0) + 1;
      ns = A.size(1);
      if (qjj < ns)
      {
        ns = qjj;
      }

      qjj = A.size(0);
      minnp = A.size(1);
      if (qjj < minnp)
      {
        minnp = qjj;
      }

      b_s.set_size(ns);
      for (i = 0; i < ns; i++)
      {
        b_s[i] = 0.0;
      }

      e.set_size(A.size(1));
      ns = A.size(1);
      for (i = 0; i < ns; i++)
      {
        e[i] = 0.0;
      }

      work.set_size(A.size(0));
      ns = A.size(0);
      for (i = 0; i < ns; i++)
      {
        work[i] = 0.0;
      }

      U.set_size(A.size(0), minnp);
      ns = A.size(0) * minnp;
      for (i = 0; i < ns; i++)
      {
        U[i] = 0.0;
      }

      Vf.set_size(A.size(1), A.size(1));
      ns = A.size(1) * A.size(1);
      for (i = 0; i < ns; i++)
      {
        Vf[i] = 0.0;
      }

      if ((A.size(0) == 0) || (A.size(1) == 0))
      {
        int ii;
        qjj = A.size(0);
        if (qjj >= minnp)
        {
          qjj = minnp;
        }

        for (ii = 0; ii < qjj; ii++)
        {
          U[ii + U.size(0) * ii] = 1.0;
        }

        i = A.size(1);
        for (ii = 0; ii < i; ii++)
        {
          Vf[ii + Vf.size(0) * ii] = 1.0;
        }
      }
      else
      {
        double snorm;
        int ii;
        int ix;
        int iy;
        int jj;
        int m;
        int nct;
        int nctp1;
        int nmq;
        int nrt;
        int q;
        int qp1;
        int qq;
        if (A.size(1) > 2)
        {
          qjj = A.size(1) - 2;
        }
        else
        {
          qjj = 0;
        }

        nrt = A.size(0);
        if (qjj < nrt)
        {
          nrt = qjj;
        }

        if (A.size(0) > 1)
        {
          qjj = A.size(0) - 1;
        }
        else
        {
          qjj = 0;
        }

        nct = A.size(1);
        if (qjj < nct)
        {
          nct = qjj;
        }

        nctp1 = nct + 1;
        if (nct > nrt)
        {
          i = nct;
        }
        else
        {
          i = nrt;
        }

        for (q = 0; q < i; q++)
        {
          boolean_T apply_transform;
          qp1 = q + 2;
          qq = (q + n * q) + 1;
          nmq = (n - q) - 1;
          apply_transform = false;
          if (q + 1 <= nct)
          {
            nrm = blas::xnrm2(nmq + 1, b_A, qq);
            if (nrm > 0.0)
            {
              apply_transform = true;
              if (b_A[qq - 1] < 0.0)
              {
                r = -nrm;
                b_s[q] = -nrm;
              }
              else
              {
                r = nrm;
                b_s[q] = nrm;
              }

              if (std::abs(r) >= 1.0020841800044864E-292)
              {
                nrm = 1.0 / r;
                ns = qq + nmq;
                for (k = qq; k <= ns; k++)
                {
                  b_A[k - 1] = nrm * b_A[k - 1];
                }
              }
              else
              {
                ns = qq + nmq;
                for (k = qq; k <= ns; k++)
                {
                  b_A[k - 1] = b_A[k - 1] / b_s[q];
                }
              }

              b_A[qq - 1] = b_A[qq - 1] + 1.0;
              b_s[q] = -b_s[q];
            }
            else
            {
              b_s[q] = 0.0;
            }
          }

          for (jj = qp1; jj <= p; jj++)
          {
            qjj = q + n * (jj - 1);
            if (apply_transform)
            {
              nrm = 0.0;
              if (nmq + 1 >= 1)
              {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= nmq; k++)
                {
                  nrm += b_A[ix - 1] * b_A[iy];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / b_A[q + b_A.size(0) * q]);
              blas::xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
            }

            e[jj - 1] = b_A[qjj];
          }

          if (q + 1 <= nct)
          {
            for (ii = q + 1; ii <= n; ii++)
            {
              U[(ii + U.size(0) * q) - 1] = b_A[(ii + b_A.size(0) * q) - 1];
            }
          }

          if (q + 1 <= nrt)
          {
            qq = p - q;
            nrm = blas::xnrm2(qq - 1, e, q + 2);
            if (nrm == 0.0)
            {
              e[q] = 0.0;
            }
            else
            {
              if (e[q + 1] < 0.0)
              {
                e[q] = -nrm;
              }
              else
              {
                e[q] = nrm;
              }

              nrm = e[q];
              if (std::abs(e[q]) >= 1.0020841800044864E-292)
              {
                nrm = 1.0 / e[q];
                ns = q + qq;
                for (k = qp1; k <= ns; k++)
                {
                  e[k - 1] = nrm * e[k - 1];
                }
              }
              else
              {
                ns = q + qq;
                for (k = qp1; k <= ns; k++)
                {
                  e[k - 1] = e[k - 1] / nrm;
                }
              }

              e[q + 1] = e[q + 1] + 1.0;
              e[q] = -e[q];
              if (q + 2 <= n)
              {
                for (ii = qp1; ii <= n; ii++)
                {
                  work[ii - 1] = 0.0;
                }

                for (jj = qp1; jj <= p; jj++)
                {
                  blas::xaxpy(nmq, e[jj - 1], b_A, (q + n * (jj - 1)) + 2, work,
                              q + 2);
                }

                for (jj = qp1; jj <= p; jj++)
                {
                  blas::xaxpy(nmq, -e[jj - 1] / e[q + 1], work, q + 2, b_A, (q + n * (jj - 1)) + 2);
                }
              }
            }

            for (ii = qp1; ii <= p; ii++)
            {
              Vf[(ii + Vf.size(0) * q) - 1] = e[ii - 1];
            }
          }
        }

        if (A.size(1) < A.size(0) + 1)
        {
          m = A.size(1) - 1;
        }
        else
        {
          m = A.size(0);
        }

        if (nct < A.size(1))
        {
          b_s[nct] = b_A[nct + b_A.size(0) * nct];
        }

        if (A.size(0) < m + 1)
        {
          b_s[m] = 0.0;
        }

        if (nrt + 1 < m + 1)
        {
          e[nrt] = b_A[nrt + b_A.size(0) * m];
        }

        e[m] = 0.0;
        if (nct + 1 <= minnp)
        {
          for (jj = nctp1; jj <= minnp; jj++)
          {
            for (ii = 0; ii < n; ii++)
            {
              U[ii + U.size(0) * (jj - 1)] = 0.0;
            }

            U[(jj + U.size(0) * (jj - 1)) - 1] = 1.0;
          }
        }

        for (q = nct; q >= 1; q--)
        {
          qp1 = q + 1;
          ns = n - q;
          qq = (q + n * (q - 1)) - 1;
          if (b_s[q - 1] != 0.0)
          {
            for (jj = qp1; jj <= minnp; jj++)
            {
              qjj = q + n * (jj - 1);
              nrm = 0.0;
              if (ns + 1 >= 1)
              {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= ns; k++)
                {
                  nrm += U[ix] * U[iy - 1];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / U[qq]);
              blas::xaxpy(ns + 1, nrm, qq + 1, U, qjj);
            }

            for (ii = q; ii <= n; ii++)
            {
              U[(ii + U.size(0) * (q - 1)) - 1] = -U[(ii + U.size(0) * (q - 1)) - 1];
            }

            U[qq] = U[qq] + 1.0;
            for (ii = 0; ii <= q - 2; ii++)
            {
              U[ii + U.size(0) * (q - 1)] = 0.0;
            }
          }
          else
          {
            for (ii = 0; ii < n; ii++)
            {
              U[ii + U.size(0) * (q - 1)] = 0.0;
            }

            U[qq] = 1.0;
          }
        }

        for (q = p; q >= 1; q--)
        {
          if ((q <= nrt) && (e[q - 1] != 0.0))
          {
            qp1 = q + 1;
            qq = p - q;
            ns = q + p * (q - 1);
            for (jj = qp1; jj <= p; jj++)
            {
              qjj = q + p * (jj - 1);
              nrm = 0.0;
              if (qq >= 1)
              {
                ix = ns;
                iy = qjj;
                for (k = 0; k < qq; k++)
                {
                  nrm += Vf[ix] * Vf[iy];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / Vf[ns]);
              blas::xaxpy(qq, nrm, ns + 1, Vf, qjj + 1);
            }
          }

          for (ii = 0; ii < p; ii++)
          {
            Vf[ii + Vf.size(0) * (q - 1)] = 0.0;
          }

          Vf[(q + Vf.size(0) * (q - 1)) - 1] = 1.0;
        }

        nmq = m;
        qq = 0;
        snorm = 0.0;
        for (q = 0; q <= m; q++)
        {
          if (b_s[q] != 0.0)
          {
            nrm = std::abs(b_s[q]);
            r = b_s[q] / nrm;
            b_s[q] = nrm;
            if (q + 1 < m + 1)
            {
              e[q] = e[q] / r;
            }

            if (q + 1 <= n)
            {
              ns = n * q;
              i = ns + n;
              for (k = ns + 1; k <= i; k++)
              {
                U[k - 1] = r * U[k - 1];
              }
            }
          }

          if ((q + 1 < m + 1) && (e[q] != 0.0))
          {
            nrm = std::abs(e[q]);
            r = nrm / e[q];
            e[q] = nrm;
            b_s[q + 1] = b_s[q + 1] * r;
            ns = p * (q + 1);
            i = ns + p;
            for (k = ns + 1; k <= i; k++)
            {
              Vf[k - 1] = r * Vf[k - 1];
            }
          }

          nrm = std::abs(b_s[q]);
          r = std::abs(e[q]);
          if ((nrm > r) || rtIsNaN(r))
          {
            r = nrm;
          }

          if ((!(snorm > r)) && (!rtIsNaN(r)))
          {
            snorm = r;
          }
        }

        while ((m + 1 > 0) && (qq < 75))
        {
          boolean_T exitg1;
          ii = m;
          exitg1 = false;
          while (!(exitg1 || (ii == 0)))
          {
            nrm = std::abs(e[ii - 1]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[ii - 1]) + std::
                                                                              abs(b_s[ii]))) ||
                (nrm <= 1.0020841800044864E-292) || ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm)))
            {
              e[ii - 1] = 0.0;
              exitg1 = true;
            }
            else
            {
              ii--;
            }
          }

          if (ii == m)
          {
            ns = 4;
          }
          else
          {
            qjj = m + 1;
            ns = m + 1;
            exitg1 = false;
            while ((!exitg1) && (ns >= ii))
            {
              qjj = ns;
              if (ns == ii)
              {
                exitg1 = true;
              }
              else
              {
                nrm = 0.0;
                if (ns < m + 1)
                {
                  nrm = std::abs(e[ns - 1]);
                }

                if (ns > ii + 1)
                {
                  nrm += std::abs(e[ns - 2]);
                }

                r = std::abs(b_s[ns - 1]);
                if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
                                                            1.0020841800044864E-292))
                {
                  b_s[ns - 1] = 0.0;
                  exitg1 = true;
                }
                else
                {
                  ns--;
                }
              }
            }

            if (qjj == ii)
            {
              ns = 3;
            }
            else if (qjj == m + 1)
            {
              ns = 1;
            }
            else
            {
              ns = 2;
              ii = qjj;
            }
          }

          switch (ns)
          {
          case 1:
          {
            r = e[m - 1];
            e[m - 1] = 0.0;
            for (k = m; k >= ii + 1; k--)
            {
              blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
              if (k > ii + 1)
              {
                double b;
                b = e[k - 2];
                r = -scale * b;
                e[k - 2] = b * sm;
              }

              if (p >= 1)
              {
                ix = p * (k - 1);
                iy = p * m;
                for (ns = 0; ns < p; ns++)
                {
                  double temp;
                  temp = sm * Vf[ix] + scale * Vf[iy];
                  Vf[iy] = sm * Vf[iy] - scale * Vf[ix];
                  Vf[ix] = temp;
                  iy++;
                  ix++;
                }
              }
            }
          }
          break;

          case 2:
          {
            r = e[ii - 1];
            e[ii - 1] = 0.0;
            for (k = ii + 1; k <= m + 1; k++)
            {
              double b;
              blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
              b = e[k - 1];
              r = -scale * b;
              e[k - 1] = b * sm;
              if (n >= 1)
              {
                ix = n * (k - 1);
                iy = n * (ii - 1);
                for (ns = 0; ns < n; ns++)
                {
                  double temp;
                  temp = sm * U[ix] + scale * U[iy];
                  U[iy] = sm * U[iy] - scale * U[ix];
                  U[ix] = temp;
                  iy++;
                  ix++;
                }
              }
            }
          }
          break;

          case 3:
          {
            double b;
            double temp;
            scale = std::abs(b_s[m]);
            nrm = b_s[m - 1];
            r = std::abs(nrm);
            if ((!(scale > r)) && (!rtIsNaN(r)))
            {
              scale = r;
            }

            b = e[m - 1];
            r = std::abs(b);
            if ((!(scale > r)) && (!rtIsNaN(r)))
            {
              scale = r;
            }

            r = std::abs(b_s[ii]);
            if ((!(scale > r)) && (!rtIsNaN(r)))
            {
              scale = r;
            }

            r = std::abs(e[ii]);
            if ((!(scale > r)) && (!rtIsNaN(r)))
            {
              scale = r;
            }

            sm = b_s[m] / scale;
            nrm /= scale;
            r = b / scale;
            temp = b_s[ii] / scale;
            b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
            nrm = sm * r;
            nrm *= nrm;
            if ((b != 0.0) || (nrm != 0.0))
            {
              r = std::sqrt(b * b + nrm);
              if (b < 0.0)
              {
                r = -r;
              }

              r = nrm / (b + r);
            }
            else
            {
              r = 0.0;
            }

            r += (temp + sm) * (temp - sm);
            nrm = temp * (e[ii] / scale);
            for (k = ii + 1; k <= m; k++)
            {
              blas::xrotg(&r, &nrm, &sm, &scale);
              if (k > ii + 1)
              {
                e[k - 2] = r;
              }

              b = e[k - 1];
              nrm = b_s[k - 1];
              e[k - 1] = sm * b - scale * nrm;
              r = scale * b_s[k];
              b_s[k] = b_s[k] * sm;
              if (p >= 1)
              {
                ix = p * (k - 1);
                iy = p * k;
                for (ns = 0; ns < p; ns++)
                {
                  temp = sm * Vf[ix] + scale * Vf[iy];
                  Vf[iy] = sm * Vf[iy] - scale * Vf[ix];
                  Vf[ix] = temp;
                  iy++;
                  ix++;
                }
              }

              b_s[k - 1] = sm * nrm + scale * b;
              blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
              r = sm * e[k - 1] + scale * b_s[k];
              b_s[k] = -scale * e[k - 1] + sm * b_s[k];
              nrm = scale * e[k];
              e[k] = e[k] * sm;
              if ((k < n) && (n >= 1))
              {
                ix = n * (k - 1);
                iy = n * k;
                for (ns = 0; ns < n; ns++)
                {
                  temp = sm * U[ix] + scale * U[iy];
                  U[iy] = sm * U[iy] - scale * U[ix];
                  U[ix] = temp;
                  iy++;
                  ix++;
                }
              }
            }

            e[m - 1] = r;
            qq++;
          }
          break;

          default:
          {
            if (b_s[ii] < 0.0)
            {
              b_s[ii] = -b_s[ii];
              ns = p * ii;
              i = ns + p;
              for (k = ns + 1; k <= i; k++)
              {
                Vf[k - 1] = -Vf[k - 1];
              }
            }

            qp1 = ii + 1;
            while ((ii + 1 < nmq + 1) && (b_s[ii] < b_s[qp1]))
            {
              double temp;
              nrm = b_s[ii];
              b_s[ii] = b_s[qp1];
              b_s[qp1] = nrm;
              if (ii + 1 < p)
              {
                ix = p * ii;
                iy = p * (ii + 1);
                for (k = 0; k < p; k++)
                {
                  temp = Vf[ix];
                  Vf[ix] = Vf[iy];
                  Vf[iy] = temp;
                  ix++;
                  iy++;
                }
              }

              if (ii + 1 < n)
              {
                ix = n * ii;
                iy = n * (ii + 1);
                for (k = 0; k < n; k++)
                {
                  temp = U[ix];
                  U[ix] = U[iy];
                  U[iy] = temp;
                  ix++;
                  iy++;
                }
              }

              ii = qp1;
              qp1++;
            }

            qq = 0;
            m--;
          }
          break;
          }
        }
      }

      s.set_size(minnp);
      V.set_size(A.size(1), minnp);
      for (k = 0; k < minnp; k++)
      {
        s[k] = b_s[k];
        for (ns = 0; ns < p; ns++)
        {
          V[ns + V.size(0) * k] = Vf[ns + Vf.size(0) * k];
        }
      }
    }
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
