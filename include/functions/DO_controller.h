#ifndef DO_CONTROLLER_H
#define DO_CONTROLLER_H

#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
//#include "m_c_g_matrix.h"
#include "m_c_g_utility.h"
//#include "logR.h"
using namespace Eigen;
typedef Eigen::Vector<double, 3*n> Vector3nd;

struct Desired_pos {
	Matrix4d Xd;
	Vector6d vel;
	Vector6d acc;
};

extern void logR(const Matrix3d R, Vector3d& w);

extern void A_x_inv(const Matrix6xnd J, const Matrixnd M, Matrix6d& invAx);

extern void A_x_x(const Matrix6xnd J, const Matrixnd M, Matrix6xnd x, Matrix6xnd& Axx);

extern void pinv_JT_x(const Matrix6xnd J, const Matrixnd M, Matrixnd x, Matrix6xnd& pinvJtx);

extern void pinv_J(const Matrix6xnd J, const Matrixnd M, Matrixnx6d& pinvJ);

extern void Mu_x(const Matrix6xnd J, const Matrixnd M, Matrix6xnd dJ, Matrixnd C, Matrix6d& mux);

extern void pinv_J_x(const Matrix6xnd J, const Matrixnd M, Vector6d x, Vectornd& pinvJX);

extern void null_proj(const Matrix6xnd J, const Matrixnd M, Vectornd x, Vectornd& Nx);

extern void DO_controller(const Robot* robot, const Desired_pos *pos, const Matrix6d Kx, const Matrix6d Bx,
	const Matrixnd Kn, const Matrixnd Bn, const double kesai, Vectornd ddq, const Matrixnd Y, const Vector3nd y, Vectornd& tao, Vectornd& td);



#endif