//
// File: derivative_jacobian_matrix.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 17-Apr-2022 12:11:48
//
#ifndef DERIVATIVE_JACOBIAN_MATRIX_H
#define DERIVATIVE_JACOBIAN_MATRIX_H

// Include Files
#include "robot.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations, NO TCP
extern void derivative_jacobian_matrix(const Robot *robot, const coder::
  array<double, 2U> &q, const coder::array<double, 2U> &qd, coder::array<double,
  2U> &dJb, coder::array<double, 2U> &Jb, double dT[16], double T[16]);

#endif

//
// File trailer for derivative_jacobian_matrix.h
//
// [EOF]
//
