#ifndef CAL_J_H
#define CAL_J_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include "utility.h"
#include "robot.h"
#include "m_c_g_utility.h"

//long pole ee
void cal_J(const Robot* robot, const Eigen::Vector7d &q, const Eigen::Vector7d &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F,
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J1, Eigen::MatrixXd &dJ1, Eigen::MatrixXd &J2, Eigen::MatrixXd &dJ2, Eigen::Vector3d &p1, Eigen::Vector3d &p2);

#endif