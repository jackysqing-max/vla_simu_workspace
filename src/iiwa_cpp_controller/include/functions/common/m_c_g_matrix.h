//
// File: m_c_g_matrix.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 09-Mar-2024 19:04:26
//

#ifndef M_C_G_MATRIX_H
#define M_C_G_MATRIX_H

// Include Files
#include "robot.h"
#include "coder_array.h"

// Function Declarations
extern void
m_c_g_matrix(const Robot *robot, const coder::array<double, 1U> &q,
             const coder::array<double, 1U> &qd, coder::array<double, 2U> &Mq,
             coder::array<double, 2U> &C, coder::array<double, 1U> &g,
             coder::array<double, 2U> &Jb, coder::array<double, 2U> &dJb,
             coder::array<double, 2U> &dMq, double dTcp[16], double Tcp[16]);

#endif
//
// File trailer for m_c_g_matrix.h
//
// [EOF]
//
