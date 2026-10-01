#ifndef M_C_G_UTILITY_H
#define M_C_G_UTILITY_H

#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include "coder_array.h"
#include "robot.h"
using namespace Eigen;
const int n = 7;
typedef Eigen::Vector<double, 6> Vector6d;
typedef Eigen::Vector<double, n> Vectornd;
typedef Eigen::Vector<double, 21> Vector21d;
typedef Eigen::Matrix<double, n, n> Matrixnd;
typedef Eigen::Matrix<double, 6, n> Matrix6xnd;
typedef Eigen::Matrix<double, n, 6> Matrixnx6d;
typedef Eigen::Matrix<double, n, 3> Matrixnx3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3 * n> Matrix3x3nd;
typedef Eigen::Matrix<double, 4, 4 * n> Matrix4x4nd;
typedef Eigen::Matrix<double, 6, 6 * n> Matrix6x6nd;
typedef Eigen::Matrix<double, 6, n* n> Matrix6xnnd;
typedef Eigen::Matrix<double, n, n* n> Matrixnxnnd;
typedef Eigen::Matrix<double, n, n* n> Matrixnxnnd;
typedef Eigen::Matrix<double, 6 * n, n* n> Matrix6xnnnd;
typedef Eigen::Matrix<double, 1, 6> RVector6d;
typedef Eigen::Matrix<double, 1, n> RVectornd;
typedef Eigen::Matrix<double, 1, 3> RVector3d;

 
//struct Robot
//{
//	double dof;
//	coder::array<double, 1U> mass;
//	coder::array<double, 3U> inertia;
//	coder::array<double, 2U> A;
//	coder::array<double, 3U> M;
//	double ME[16];
//	coder::array<double, 2U> com;
//	double gravity[3];
//	double TCP[16];
//};

// extern void 
// mass_matrix(const Robot* robot, Vectornd q, Matrixnd& Mq, Matrix6xnd& J);
Eigen::Matrix3d exp_r_E(const Eigen::Vector3d &r);

void
exp_w_E(const RVector3d v, Matrix3d& R);

void
normalize_twist_E(const RVector6d v, RVectornd& sa);

void
exp_twist_E(const RVector6d v, Matrix4d& tform);

void
tform_inv_E(const Matrix4d T, Matrix4d& invT);

void
se_twist_E(const Vector6d v, Matrix4d& Tv);

void
adjoint_T_E(const Matrix4d tform, Matrix6d& Adt);

void
derivative_adjoint_T_E(const Matrix4d T, const Matrix4d dT,
	Matrix6d& dAdT, Matrix6d& AdT);

void
m_c_g_matrix_E(const Robot* robot, const Vectornd q, const Vectornd qd,
	Matrixnd& Mq, Matrixnd& C,
	Vectornd& g, Matrix6xnd& Jb,
	Matrix6xnd& dJb, Matrixnd& dMq,
	Matrix4d& dTcp, Matrix4d& Tcp);
#endif
// #endif#pragma once
