//
// File: read_dynamics_file_types.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//
#ifndef ROBOT_H
#define ROBOT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include "utility.h"
// Type Definitions
struct Robot
{
	double dof;
	coder::array<double, 1U> mass;
	coder::array<double, 3U> inertia;
	coder::array<double, 2U> A;
	coder::array<double, 3U> M;
	double ME[16];
	coder::array<double, 2U> com;
	double gravity[3];
	double TCP[16];
};

// load robot structure in Json file
Robot loadRobot(const char* filename);

void jacobian_matrix(const Robot* robot, const std::vector<double>& q, Eigen::MatrixXd& J, Eigen::Matrix4d& T);

void m_c_g_matrix(const Robot* robot, const std::vector<double>& q,
	const std::vector<double>& dq, Eigen::MatrixXd& M, 
	Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::MatrixXd& J, Eigen::MatrixXd& dJ,
	Eigen::MatrixXd& dM, Eigen::Matrix4d& dT, Eigen::Matrix4d& T);


void derivative_jacobian_matrix(const Robot* robot, const std::vector<double>& q, const std::vector<double>& dq,
	                            Eigen::MatrixXd& dJ, Eigen::MatrixXd& J, Eigen::Matrix4d& dT, Eigen::Matrix4d& T);


void admittance_control(const Robot* robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Td, const Eigen::Vector6d& Vd,
	const Eigen::Matrix3d& Mp, const Eigen::Matrix3d& Bp, const Eigen::Matrix3d& Kp,
	const Eigen::Matrix3d& Mr, const Eigen::Matrix3d& Br, const Eigen::Matrix3d& Kr,
	const std::vector<double>& q, const std::vector<double>& qd, const Eigen::Vector6d& F, double dt,
	Eigen::Vector3d& re, Eigen::Vector3d& pe, Eigen::Vector3d& red, Eigen::Vector3d& ped, bool flag = false, double* Tcmd = nullptr);

void admittance_error_cal(const Robot* robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Td, const Eigen::Vector6d& Vd,
	const std::vector<double>& q, const std::vector<double>& qd, Eigen::Vector3d& re, Eigen::Vector3d& pe, Eigen::Vector3d& red, Eigen::Vector3d& ped, bool flag);

void getTaskSpaceMotion(const Robot& robot, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd, double T[16], double V[6], double dV[6]);
void gravityAndInertComp(const Robot& robot, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd, double _T[16], double _Tcb[16], double _Tcs[16], float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double>& pose, const double _G[36]);
Eigen::Vector6d gravityAndInertiaCompensation(const Robot& robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Tsensor, const std::vector<double>& q, const std::vector<double>& qd,
	const std::vector<double>& qdd, const float* rawForce, float mass, const float offset[6], const float cog[3], const Eigen::Matrix3d& mI, double scale = 1.0);




template <class T, int m, int n>
coder::array<T, 1>& coder_array_1d_wrapper(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 1> array;
	array.set(M.data(), m*n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper1(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper2(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}
template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper3(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}
template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper4(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper5(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper6(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

template <class T, int m, int n>
coder::array<T, 2>& coder_array_wrapper7(Eigen::Matrix<T, m, n>& M)
{
	static coder::array<T, 2> array;
	array.set(M.data(), m, n);
	return array;
}

#endif

//
// File trailer for read_dynamics_file_types.h
//
// [EOF]
//
