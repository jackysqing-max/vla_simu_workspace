//
// File: UR5e.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 12-Feb-2022 22:09:22
/*
The kinematics of UR5e including forward and inverse kinematics
UR5e的正反解，从MATLAB中导出的代码
 */

// Include Files
#include "UR5e.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <exception>
#include <stdexcept>
Robot UR5e(const char *calibration_file)
{
  Robot bot;
  bot.A.set_size(6, 6);
  bot.dof = 6;
  bot.inertia.set_size(3, 3, 6);
  bot.com.set_size(6, 3);
  bot.M.set_size(4, 4, 6);
  bot.mass.set_size(6);
  Robot *robot = &bot;

  double a[6] = {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0};
  double d[6] = {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996};
  double alpha[6] = {1.570796327, 0, 0, 1.570796327, -1.570796327, 0};
  double v;
  std::vector<double> delta;
  std::string laji;
  try
  {
    std::ifstream fin(calibration_file);
    if (!fin.is_open())
      throw std::invalid_argument("file does not exist!");
    while (!fin.eof())
    {
      if (fin >> v)
        delta.push_back(v);
      else
      {
        fin.clear();
        fin >> laji;
      }
      if (delta.size() == 24)
        break;
    }
    if (delta.size() != 24)
      throw std::runtime_error("data format wrong!");
  }
  catch (std::exception &e)
  {
    std::cerr << "reading calibraiton file failed: " << e.what() << std::endl;
    throw e;
  }
  double *delta_theta = &delta[0];
  double *delta_a = &delta[0] + 6;
  double *delta_d = &delta[0] + 12;
  double *delta_alpha = &delta[0] + 18;
  const double dh_table[24] =
      {alpha[5] + delta_alpha[5], alpha[0] + delta_alpha[0], alpha[1] + delta_alpha[1], alpha[2] + delta_alpha[2], alpha[3] + delta_alpha[3], alpha[4] + delta_alpha[4],
       a[5] + delta_a[5], a[0] + delta_a[0], a[1] + delta_a[1], a[2] + delta_a[2], a[3] + delta_a[3], a[4] + delta_a[4],
       d[0] + delta_d[0], d[1] + delta_d[1], d[2] + delta_d[2], d[3] + delta_d[3], d[4] + delta_d[4], d[5] + delta_d[5],
       delta_theta[0], delta_theta[1], delta_theta[2], delta_theta[3], delta_theta[4], delta_theta[5]};
  /*static const double dh_table[24] = {0.0, 1.5706381365466275,
    0.0011379901824535516, 0.00732065515331057, 1.5699013066072129,
    -1.570215141982787, 0.0, 0.00018476643006531352, -0.35384590500124719,
    -0.38137801722259496, -7.7844561272379507E-5, 3.663908567097521E-5,
    0.16275539960981536, -207.3621341312587, 220.00956641856965,
    -12.513940703626719, 0.099744483485578433, 0.099650356951636945,
    -1.3056566983832285E-8, -0.58815911518087249, 0.826314252559053,
    -0.23816520072369335, -1.0736218860607494E-6, -2.9306916079357981E-7 };*/

  static const double dv[6] = {3.761, 8.058, 2.846, 1.37, 1.3, 0.365};

  static const signed char iv[6] = {0, 0, 1, 0, 0, 0};

  int b_i;
  int i;
  signed char A[36];
  robot->dof = 6.0;
  for (i = 0; i < 6; i++)
  {
    double cpha;
    double ct;
    double dh_row_idx_0;
    double dh_row_idx_2;
    double spha;
    double st;

    //  dh_row: alpha, a, d, theta
    b_i = i << 4;
    robot->M[b_i + 8] = 0.0;
    dh_row_idx_0 = dh_table[i];
    robot->M[b_i + 3] = 0.0;
    robot->M[b_i + 7] = 0.0;
    dh_row_idx_2 = dh_table[i + 12];
    robot->M[b_i + 11] = 0.0;
    spha = dh_table[i + 18];
    robot->M[b_i + 15] = 1.0;
    ct = std::cos(spha);
    st = std::sin(spha);
    cpha = std::cos(dh_row_idx_0);
    spha = std::sin(dh_row_idx_0);
    robot->M[b_i] = ct;
    robot->M[b_i + 4] = -st;
    robot->M[b_i + 12] = dh_table[i + 6];
    robot->M[b_i + 1] = st * cpha;
    robot->M[b_i + 5] = ct * cpha;
    robot->M[b_i + 9] = -spha;
    robot->M[b_i + 13] = -dh_row_idx_2 * spha;
    robot->M[b_i + 2] = st * spha;
    robot->M[b_i + 6] = ct * spha;
    robot->M[b_i + 10] = cpha;
    robot->M[b_i + 14] = dh_row_idx_2 * cpha;
    for (b_i = 0; b_i < 6; b_i++)
    {
      A[i + 6 * b_i] = iv[b_i];
    }
  }

  for (b_i = 0; b_i < 36; b_i++)
  {
    robot->A[b_i] = A[b_i];
  }

  std::memset(&robot->ME[0], 0, 16U * sizeof(double));
  robot->ME[0] = 1.0;
  robot->ME[5] = 1.0;
  robot->ME[10] = 1.0;
  robot->ME[15] = 1.0;
  for (i = 0; i < 6; i++)
  {
    robot->mass[i] = dv[i];
  }

  std::memset(&robot->inertia[0], 0, 54U * sizeof(double));
  std::memset(&robot->com[0], 0, 18U * sizeof(double));
  std::memset(&robot->TCP[0], 0, 16U * sizeof(double));
  robot->TCP[0] = 1.0;
  robot->TCP[5] = 1.0;
  robot->TCP[10] = 1.0;
  robot->TCP[15] = 1.0;
  robot->gravity[0] = 0.0;
  robot->gravity[1] = 0.0;
  robot->gravity[2] = -9.8;
  return bot;
}

//
// File trailer for UR5e.cpp
//
// [EOF]
//
