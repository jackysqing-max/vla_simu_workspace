//
// File: pseudo.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 25-Mar-2022 14:44:47
//

// Include Files
#include "pseudo.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <svd.h>
#include <cmath>
namespace coder
{
    void eml_pinv(const ::coder::array<double, 2U>& A, ::coder::array<double, 2U>
        & X)
    {
        array<double, 2U> U;
        array<double, 2U> V;
        array<double, 1U> s;
        double absx;
        int ar;
        int br;
        int cr;
        int i;
        int ib;
        int m;
        int n;
        int nx;
        m = A.size(0);
        n = A.size(1);
        X.set_size(A.size(1), A.size(0));
        nx = A.size(1) * A.size(0);
        for (i = 0; i < nx; i++) {
            X[i] = 0.0;
        }

        if ((A.size(0) != 0) && (A.size(1) != 0)) {
            boolean_T p;
            nx = A.size(0) * A.size(1);
            p = true;
            for (ar = 0; ar < nx; ar++) {
                if ((!p) || (rtIsInf(A[ar]) || rtIsNaN(A[ar]))) {
                    p = false;
                }
            }

            if (!p) {
                X.set_size(A.size(1), A.size(0));
                nx = A.size(1) * A.size(0);
                for (i = 0; i < nx; i++) {
                    X[i] = rtNaN;
                }
            }
            else {
                int r;
                internal::svd(A, U, s, V);
                absx = std::abs(s[0]);
                if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
                    if (absx <= 2.2250738585072014E-308) {
                        absx = 4.94065645841247E-324;
                    }
                    else {
                        frexp(absx, &br);
                        absx = std::ldexp(1.0, br - 53);
                    }
                }
                else {
                    absx = rtNaN;
                }

                absx *= static_cast<double>(A.size(0));
                r = -1;
                ar = 0;
                while ((ar <= n - 1) && (s[ar] > absx)) {
                    r++;
                    ar++;
                }

                if (r + 1 > 0) {
                    int i1;
                    int ic;
                    nx = 1;
                    for (br = 0; br <= r; br++) {
                        absx = 1.0 / s[br];
                        i = nx + n;
                        i1 = i - 1;
                        for (ar = nx; ar <= i1; ar++) {
                            V[ar - 1] = absx * V[ar - 1];
                        }

                        nx = i;
                    }

                    nx = A.size(1) * (A.size(0) - 1);
                    for (cr = 0; n < 0 ? cr >= nx : cr <= nx; cr += n) {
                        i = cr + 1;
                        i1 = cr + n;
                        for (ic = i; ic <= i1; ic++) {
                            X[ic - 1] = 0.0;
                        }
                    }

                    br = 0;
                    for (cr = 0; n < 0 ? cr >= nx : cr <= nx; cr += n) {
                        ar = -1;
                        br++;
                        i = br + m * r;
                        for (ib = br; m < 0 ? ib >= i : ib <= i; ib += m) {
                            int i2;
                            int ia;
                            ia = ar;
                            i1 = cr + 1;
                            i2 = cr + n;
                            for (ic = i1; ic <= i2; ic++) {
                                ia++;
                                X[ic - 1] = X[ic - 1] + U[ib - 1] * V[ia];
                            }

                            ar += n;
                        }
                    }
                }
            }
        }
    }
}

// Function Definitions
//
// Arguments    : const coder::array<double, 2U> &M
//                coder::array<double, 2U> &invp
// Return Type  : void
//
void pseudo(const coder::array<double, 2U> &M, coder::array<double, 2U> &invp)
{
  coder::array<double, 2U> b_M;
  coder::array<double, 2U> r;
  if (M.size(0) < M.size(1)) {
    int b_loop_ub;
    int i;
    int i1;
    int loop_ub;
    b_M.set_size(M.size(1), M.size(0));
    loop_ub = M.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = M.size(1);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        b_M[i1 + b_M.size(0) * i] = M[i + M.size(0) * i1];
      }
    }

    coder::eml_pinv(b_M, r);
    invp.set_size(r.size(1), r.size(0));
    loop_ub = r.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r.size(1);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        invp[i1 + invp.size(0) * i] = r[i + r.size(0) * i1];
      }
    }
  } else {
    coder::eml_pinv(M, invp);
  }
}

//
// File trailer for pseudo.cpp
//
// [EOF]
//
