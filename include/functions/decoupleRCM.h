#ifndef DECOUPLERCM_H
#define DECOUPLERCM_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include "utility.h"
#include "robot.h"
#include "m_c_g_utility.h"

Eigen::Matrix6x8d J_external(const Eigen::Matrix6x7d &J1, const Eigen::Matrix6x7d &J2, const double &lamda, const Eigen::Vector3d p1, const Eigen::Vector3d p2);

Eigen::Vector3d P_trocar(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double &lamda);

Eigen::Vector6d RCM_error(const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Tcpd, const Eigen::Vector3d &P_trd, const Eigen::Vector3d &P_tr);



Eigen::Vector8d generateRCM_dq(const Eigen::Matrix6x8d &J_ext, const Eigen::Matrix3d &Kp2, 
const Eigen::Matrix3d &Kptr, const Eigen::Vector8d &w, const Eigen::Vector6d &e);

Eigen::Vector8d generateRCM_tau(const Eigen::Matrix6x8d &J_ext, const Eigen::Matrix3d &Kptr, 
const Eigen::Matrix3d &Bptr, const Eigen::Vector8d &w, const Eigen::Vector6d &e, const Eigen::Vector6d &dp);

Eigen::MatrixXd J1_sharp_null_J2(const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2);

Eigen::MatrixXd cal_J_lamda(const Eigen::Vector3d &p_rcm, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2, const Eigen::Vector7d &dq);

Eigen::MatrixXd cal_dJ_lamda(const Eigen::Vector3d &p_rcm, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2, const Eigen::MatrixXd &dJ1, const Eigen::MatrixXd &dJ2, const Eigen::Vector7d &dq);

void rcm_Jacobian(const Robot* robot, const Eigen::Vector7d &q, const Eigen::Vector7d &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F, const Eigen::Vector3d &rcm, 
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J, Eigen::MatrixXd &dJ, Eigen::Vector3d &x);

void rcm_motion_Jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F, const Eigen::Vector3d &rcm, 
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J, Eigen::MatrixXd &dJ, Eigen::VectorXd &x);

#endif