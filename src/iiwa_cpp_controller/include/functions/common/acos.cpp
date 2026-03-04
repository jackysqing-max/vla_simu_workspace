//
// File: acos.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 05-Apr-2022 20:02:20
//

// Include Files
#include "acos.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>


static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : creal_T *x
// Return Type  : void
//
namespace coder
{
  void b_acos(creal_T *x)
  {
    creal_T u;
    creal_T v;
    double ci;
    if ((x->im == 0.0) && (!(std::abs(x->re) > 1.0))) {
      x->re = std::acos(x->re);
      x->im = 0.0;
    } else {
      double absre;
      boolean_T xneg;
      v.re = x->re + 1.0;
      v.im = x->im;
      internal::scalar::b_sqrt(&v);
      u.re = 1.0 - x->re;
      u.im = 0.0 - x->im;
      internal::scalar::b_sqrt(&u);
      if ((-v.im == 0.0) && (u.im == 0.0)) {
        ci = 0.0;
      } else {
        double t3;
        double t4;
        t3 = v.re * u.im;
        t4 = -v.im * u.re;
        ci = t3 + t4;
        if ((rtIsInf(ci) || rtIsNaN(ci)) && (!rtIsNaN(v.re)) && (!rtIsNaN(-v.im))
            && (!rtIsNaN(u.re)) && (!rtIsNaN(u.im))) {
          double absim;
          double b_absre;
          double sai;
          double sar;
          double sbi;
          double sbr;
          absre = std::abs(v.re);
          absim = std::abs(-v.im);
          if (absre > absim) {
            if (v.re < 0.0) {
              sar = -1.0;
            } else {
              sar = 1.0;
            }

            sai = -v.im / absre;
          } else if (absim > absre) {
            sar = v.re / absim;
            if (-v.im < 0.0) {
              sai = -1.0;
            } else {
              sai = 1.0;
            }

            absre = absim;
          } else {
            if (v.re < 0.0) {
              sar = -1.0;
            } else {
              sar = 1.0;
            }

            if (-v.im < 0.0) {
              sai = -1.0;
            } else {
              sai = 1.0;
            }
          }

          b_absre = std::abs(u.re);
          absim = std::abs(u.im);
          if (b_absre > absim) {
            if (u.re < 0.0) {
              sbr = -1.0;
            } else {
              sbr = 1.0;
            }

            sbi = u.im / b_absre;
          } else if (absim > b_absre) {
            sbr = u.re / absim;
            if (u.im < 0.0) {
              sbi = -1.0;
            } else {
              sbi = 1.0;
            }

            b_absre = absim;
          } else {
            if (u.re < 0.0) {
              sbr = -1.0;
            } else {
              sbr = 1.0;
            }

            if (u.im < 0.0) {
              sbi = -1.0;
            } else {
              sbi = 1.0;
            }
          }

          if ((!rtIsInf(absre)) && (!rtIsNaN(absre)) && ((!rtIsInf(b_absre)) &&
               (!rtIsNaN(b_absre)))) {
            xneg = true;
          } else {
            xneg = false;
          }

          if (rtIsNaN(ci) || (rtIsInf(ci) && xneg)) {
            ci = sar * sbi + sai * sbr;
            if (ci != 0.0) {
              ci = ci * absre * b_absre;
            } else {
              if ((rtIsInf(absre) && ((u.re == 0.0) || (u.im == 0.0))) ||
                  (rtIsInf(b_absre) && ((v.re == 0.0) || (-v.im == 0.0)))) {
                if (rtIsNaN(t3)) {
                  t3 = 0.0;
                }

                if (rtIsNaN(t4)) {
                  t4 = 0.0;
                }

                ci = t3 + t4;
              }
            }
          }
        }
      }

      xneg = (ci < 0.0);
      if (xneg) {
        ci = -ci;
      }

      if (ci >= 2.68435456E+8) {
        ci = std::log(ci) + 0.69314718055994529;
      } else if (ci > 2.0) {
        ci = std::log(2.0 * ci + 1.0 / (std::sqrt(ci * ci + 1.0) + ci));
      } else {
        absre = ci * ci;
        ci += absre / (std::sqrt(absre + 1.0) + 1.0);
        absre = std::abs(ci);
        if ((absre > 4.503599627370496E+15) || (rtIsInf(ci) || rtIsNaN(ci))) {
          ci++;
          ci = std::log(ci);
        } else {
          if (!(absre < 2.2204460492503131E-16)) {
            ci = std::log(ci + 1.0) * (ci / ((ci + 1.0) - 1.0));
          }
        }
      }

      if (xneg) {
        ci = -ci;
      }

      x->re = 2.0 * rt_atan2d_snf(u.re, v.re);
      x->im = ci;
    }
  }
}

//
// File trailer for acos.cpp
//
// [EOF]
//
