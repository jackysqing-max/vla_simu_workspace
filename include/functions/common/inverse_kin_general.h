//
// File: inverse_kin_general.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 22:44:18
//
#ifndef INVERSE_KIN_GENERAL_H
#define INVERSE_KIN_GENERAL_H

// Include Files
#include "robot.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void inverse_kin_general(const Robot *robot, const double Td[16],
  const coder::array<double, 2U> &ref, const double tol[2], coder::array<double,
  2U> &angles, double *flag);

#endif

//
// File trailer for inverse_kin_general.h
//
// [EOF]
//
