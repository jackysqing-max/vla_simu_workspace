#ifndef REGRESSOR_MATRIX_H
#define REGRESSOR_MATRIX_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include "utility.h"
#include "robot.h"
#include "m_c_g_utility.h"

void regressor_matrix(const Robot* robot, const Eigen::Vector7d q, const Eigen::Vector7d dq, 
const Eigen::Vector7d a, const Eigen::Vector7d v, const Eigen::Vector7d r, Eigen::MatrixYd &Y, Eigen::MatrixYTrd &YTr);

Eigen::MatrixXd A_matrix(const Eigen::Vector6d &V);

Eigen::MatrixYTrd get_dynamics(const Robot* robot);

#endif