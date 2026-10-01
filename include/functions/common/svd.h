//
// File: svd.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 25-Mar-2022 14:44:47
//
#ifndef SVD_H
#define SVD_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U,
             ::coder::array<double, 1U> &s, ::coder::array<double, 2U> &V);
  }
}

#endif

//
// File trailer for svd.h
//
// [EOF]
//
