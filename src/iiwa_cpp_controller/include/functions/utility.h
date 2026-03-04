#ifndef UTILITY_H
#define UTILITY_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <chrono>

namespace Eigen
{
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Vector<double, 6> Vector6d;
	typedef Eigen::Matrix<double, 6, 7> Matrix6x7d;
	typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
	typedef Eigen::Matrix<double, 7, 7> Matrix7d;
	typedef Eigen::Vector<double, 7> Vector7d;
	typedef Eigen::Matrix<double, 6, 8> Matrix6x8d;
	typedef Eigen::Matrix<double, 8, 6> Matrix8x6d;
	typedef Eigen::Matrix<double, 3, 8> Matrix3x8d;
	typedef Eigen::Matrix<double, 8, 3> Matrix8x3d;
	typedef Eigen::Matrix<double, 8, 8> Matrix8d;
    typedef Eigen::Vector<double, 8> Vector8d;
	typedef Eigen::Matrix<double, 2, 7> Matrix2x7d;
	typedef Eigen::Matrix<double, 7, 70> MatrixYd;
	typedef Eigen::Matrix<double, 70, 1> MatrixYTrd;
	typedef Eigen::Matrix<double, 3, 10> Matrix3x10d;
	typedef Eigen::Matrix<double, 6, 10> Matrix6x10d;
	typedef Eigen::Matrix<double, 70, 70> Matrix70d;
}


int isTriggered(int signal);
void getExternalForce(float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose);
Eigen::Matrix4d pose2T(const std::vector<double> &pose);
std::vector<double> T2pose(const Eigen::Matrix4d &T);
Eigen::Matrix3d so_w(const Eigen::Vector3d &w);
Eigen::Matrix4d se_twist(const Eigen::Vector6d& V);
Eigen::Vector7d normalize_twist(const Eigen::Vector6d& V);
Eigen::Matrix3d exp_r(const Eigen::Vector3d &r);
Eigen::Vector3d logR(const Eigen::Matrix3d& R);
Eigen::Matrix4d exp_twist(const Eigen::Vector6d& twist);
Eigen::Vector6d logT(const Eigen::Matrix4d& T);
Eigen::Matrix4d invertT(const Eigen::Matrix4d &T);
Eigen::Matrix6d adjoint_T(const Eigen::Matrix4d &T);
Eigen::Matrix6d adjoint_V(const Eigen::Vector6d &V);

Eigen::MatrixXd pInv(const Eigen::MatrixXd& matrix, double tol = 1e-4);
Eigen::Vector3d cond_Matrix(const Eigen::MatrixXd& A);

Eigen::MatrixXd J_sharp(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M);// X x 6
Eigen::MatrixXd d_J_sharp(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& dM);
Eigen::MatrixXd d_J_sharp_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& X);
Eigen::MatrixXd J_sharp_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& X);// X x 
Eigen::MatrixXd J_sharp_T_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& X);
Eigen::MatrixXd A_x(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M);
Eigen::MatrixXd A_x_inv(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M);
Eigen::MatrixXd A_x_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd &X);
Eigen::VectorXd null_proj(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::VectorXd& v);
Eigen::MatrixXd null_z(const Eigen::MatrixXd& J);// n x r
Eigen::MatrixXd d_null_z(const Eigen::MatrixXd& J, const Eigen::MatrixXd& dJ);
Eigen::MatrixXd z_sharp(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M);// r x n
Eigen::MatrixXd d_z_sharp(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM);
Eigen::MatrixXd Mu_x_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& C, const Eigen::MatrixXd& X);
Eigen::MatrixXd Mu_x(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& C);
Eigen::MatrixXd A_v(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M);
Eigen::MatrixXd Mu_v(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& C);
Eigen::MatrixXd Mu_xv(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& C);
Eigen::MatrixXd Mu_vx(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& C);
Eigen::MatrixXd Mu_vx_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM,  const Eigen::MatrixXd& C, const Eigen::MatrixXd& X);

void cal_motion_error(const Eigen::Matrix4d& T, const Eigen::Matrix4d& T_d,
	const Eigen::Vector3d& v, const Eigen::Vector3d& w,
	const Eigen::Vector3d& v_d, const Eigen::Vector3d& w_d,
	const Eigen::Vector3d& a_d, const Eigen::Vector3d& alpha_d,
	Eigen::Vector6d& xe, Eigen::Vector6d& dxe, Eigen::Vector6d& ddx_d);

void bodyTwist2SpatialTwist(const double T[16], const double Vb[6], const double dVb[16], double Vs[6], double dVs[6]);
void spatialTwist2BodyTwist(const double T[16], const double Vs[6], double const dVs[16], double Vb[6], double dVb[6]);
void swapOrder(double* buf, int n);

Eigen::Vector6d twist_estimate(const Eigen::Matrix4d &Td, const Eigen::Matrix4d &Td_pre, double dt);

void preciseSleep(double seconds);

void waitPeriod(const std::chrono::steady_clock::time_point& t_cycle_start, double dt);

bool setRealtimePriority(int priority);

#endif