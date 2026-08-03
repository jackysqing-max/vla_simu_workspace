//
// File: force_calibration.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 30-Apr-2022 11:19:44
//
#ifndef FORCE_CALIBRATION_H
#define FORCE_CALIBRATION_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void force_calibration(const coder::array<char, 2U> &forceFile, const
  coder::array<char, 2U> &poseFile, double *mass, double r[3], double offset[6]);

#endif

//
// File trailer for force_calibration.h
//
// [EOF]
//
