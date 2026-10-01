#include <iostream>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <Eigen/Dense>
#include "DO_controller.h"
//#include "A_x_inv.h"//ok
//#include "Mu_x.h"
//#include "pinv_J_x.h"
//#include "null_proj.h"
//#include "pinv_J_x.h"
using namespace Eigen;

void A_x_inv(const Matrix6xnd J, const Matrixnd M, Matrix6d& invAx) {
	invAx = J * (M.inverse() * J.transpose());
}

void A_x_x(const Matrix6xnd J, const Matrixnd M, Matrix6xnd x, Matrix6xnd& Axx) {
	Axx = (J * (M.inverse() * J.transpose())).inverse() * x;
}

void pinv_JT_x(const Matrix6xnd J, const Matrixnd M, Matrixnd x, Matrix6xnd& pinvJtx) {
	Matrixnd Mt = M.transpose();
	Matrixnx6d Jt = J.transpose();
	Matrixnd invMt = Mt.inverse();
	pinvJtx = (J * (invMt * Jt)).inverse() * J * (invMt * x);
	//pinvJtx = ((J * (M.transpose().inverse() * ((J.transpose()))).inverse())) * J * (M.transpose().inverse() * x);
}

void pinv_J(const Matrix6xnd J, const Matrixnd M, Matrixnx6d& pinvJ) {
	Matrixnx6d tem = M.inverse() * (J.transpose());
	pinvJ = tem * ((J * tem).inverse());
}

void Mu_x(const Matrix6xnd J, const Matrixnd M, Matrix6xnd dJ, Matrixnd C, Matrix6d& mux) {
	Matrix6xnd pinvJTx;
	pinv_JT_x(J, M, C, pinvJTx);
	Matrix6xnd Axx;
	A_x_x(J, M, dJ, Axx);
	Matrixnx6d pinvJ;
	pinv_J(J, M, pinvJ);
	mux = (pinvJTx - Axx) * pinvJ;
}

void pinv_J_x(const Matrix6xnd J, const Matrixnd M, Vector6d x, Vectornd& pinvJX) {
	pinvJX = (M.inverse() * (J.transpose())) * ((J * (M.inverse() * (J.transpose()))).inverse() * x);
}

void null_proj(const Matrix6xnd J, const Matrixnd M, Vectornd x, Vectornd& Nx) {
	Vectornd pinvJx;
	pinv_J_x(J, M, J * x, pinvJx);
	Nx = x - pinvJx;
}

void logR(const Matrix3d R, Vector3d& w) {
	// double M_PI;
	double eps = 1e-7;
	double tr = R.trace();
	double theta = std::real(std::acos((tr - 1) / 2));
	w = Vector3d::Zero();

	if (std::abs(theta - M_PI) < eps) {
		w = M_PI / std::sqrt(2 * (1 + R(0, 0))) * (R.col(0).transpose() + Vector3d(1, 0, 0).transpose());

		if (w.hasNaN()) {
			JacobiSVD<Eigen::Matrix3d> svd(R - Eigen::Matrix3d::Identity(), Eigen::ComputeFullV);
			Matrix3d V = svd.matrixV();
			w = M_PI * V.col(2);
		}
	}
	else if (theta > eps) {
		Vector3d v;
		v << (R(2, 1) - R(1, 2)) / (2 * std::sin(theta)), (R(0, 2) - R(2, 0)) / (2 * std::sin(theta)),
			(R(1, 0) - R(0, 1)) / (2 * std::sin(theta));
		double normv = v.norm();

		if (normv != 0) {
			v /= normv;
		}
		w = theta * v;
	}
}

void DO_controller(const Robot* robot, const Desired_pos* pos, const Matrix6d Kx, const Matrix6d Bx,
const Matrixnd Kn,const Matrixnd Bn, const double kesai, Vectornd ddq, const Matrixnd Y, const Vector3nd y, Vectornd &tao, Vectornd &td)
{
	double n = robot->dof;
	Vectornd q = y.head(n);
	//Vectornd qd = y.segment(n, 2*n-1);
	Vectornd qd;
	qd << y(7), y(8), y(9), y(10), y(11), y(12), y(13);
	Matrixnd Mq;
	Matrixnd C;
	Vectornd g; 
	Matrix6xnd Jb;
	Matrix6xnd dJb;
	Matrixnd dMq;
	Matrix4d dX;
	Matrix4d X;
	m_c_g_matrix_E(robot, q, qd, Mq, C, g, Jb, dJb, dMq, dX, X);
	Matrix3d R = X.block(0, 0, 3, 3);
	Vector3d p = X.block(0, 3, 3, 1);
	Vector6d Vb = Jb * qd;
	Vector3d wb = Vb.head(3);
	Vector3d v = R * Vb.tail(3);
	Matrix6d Jh;
	Jh << Matrix3d::Identity(), Matrix3d::Zero(),
		           Matrix3d::Zero(), R;
	Matrix6d dJh;
	dJh << Matrix3d::Zero(), Matrix3d::Zero(),
		Matrix3d::Zero(), dX.block(0, 0, 3, 3);
	dJb = dJh * Jb + Jh * dJb;
	Jb = Jh * Jb;
	td = y.tail(7);
	Matrix4d Xd = pos->Xd;

	//std::cout << "X: " << X << std::endl << std::endl;
	//std::cout << "Xd: " << Xd << std::endl;

	Vector6d vel = pos->vel;
	Vector6d acc = pos->acc;
	Matrix3d Rd = Xd.block(0, 0, 3, 3);
	Vector3d pd = Xd.block(0, 3, 3, 1);
	Vector3d wd = R.transpose() * (vel.head(3));
	Vector3d vd = vel.tail(3);
	Vector3d alphad = R.transpose() * (acc.head(3));
	Vector3d ad = acc.tail(3);
	//acquire initial position
	Vectornd q0;
	static int flag = 0;
	if (flag == 0)
	{
		q0 = y.head(n);
		flag = 1;
	}

	//std::cout << q0 << std::endl;

	Vector6d Vd;
	Vd << wd(0), wd(1), wd(2), vd(0), vd(1), vd(2);
	Vector6d dVd;
	Vector3d aww;
	aww = alphad - wb.cross(wd);

	//std::cout << aww << std::endl;

	dVd << aww(0), aww(1), aww(2), ad(0), ad(1), ad(2);

	//std::cout << dVd << std::endl;

	Vector6d xe;
	Vector6d dxe;
	Vector3d we;
	logR(R.transpose() * Rd, we);
	xe << we(0), we(1), we(2), pd(0) - p(0), pd(1) - p(1), pd(2) - p(2);
	//xe << 0.1, 0.1, 0.1, 0.001, 0.001, 0.001;
	//std::cout << xe << std::endl;

	Vector6d temp;
	temp << wb(0), wb(1), wb(2), v(0), v(1), v(2);
	dxe = Vd - temp;
	//dxe << 0.01, 0.01, 0.01, 0.0001, 0.0001, 0.0001;
	//std::cout << dxe << std::endl;

	Vector6d s;
	s = dxe + Kx * xe;
	//s = dxe + 25 * xe;
	
	//std::cout << s << std::endl << std::endl;
	Vector6d ax1;
	Matrix6d invAx;
	A_x_inv(Jb, Mq, invAx);
	//std::cout << Jb << std::endl << std::endl;
	//std::cout << Mq << std::endl << std::endl;
	//std::cout << invAx << std::endl << std::endl;
	Matrix6d mux;
	Mu_x(Jb, Mq, dJb, C, mux);
	//std::cout << mux << std::endl << std::endl;
	
	/*control law(26)*/
	//ax1 = dVd + Bx*dxe + Kx*xe;
	/*control law(28)*/
	//ax1 = dVd + invAx * (mux + Bx) * dxe + Kx * xe;
	/*control law(29)*/
	//ax1 = -25 * qd + invAx * ((mux + Kx) * s);
	//ax1 = dVd +  Kx * dxe + invAx * ((mux + Bx) * s);
	
	//std::cout << ax1 << std::endl << std::endl;
	Vectornd a1;
	pinv_J_x(Jb, Mq, ax1 - dJb * qd, a1);
	//std::cout << a1 << std::endl << std::endl;
	Vectornd qe;
	qe = q0 - q;
	//for pos hold
	Vectornd qed;
	qed = -qd;
	Vectornd a2;
	null_proj(Jb, Mq, Mq.inverse() * (Bn * qed + Kn * qe), a2);
	//std::cout << a2 << std::endl;
	Vectornd P;
	P = Y * qd;
	Vectornd tao_d;
	tao_d = td + P;
	Matrix6d ttemp = Jb*(Mq.inverse()*Jb.transpose());
 	tao_d = Jb.transpose() * (ttemp.inverse() * (Jb * (Mq.inverse() * tao_d)));
	//tao_d = Jb.transpose() * (Jb * (Mq.inverse() * (Jb.transpose())).inverse() * (Jb * (Mq.inverse() * tao_d)));
	//tao = Mq * (a1 + a2) + C * qd + g - tao_d;
	tao = Mq * (a1 + a2) - tao_d;
	//td = Y * ((Mq.inverse()) * (C * qd + g - tao - P - td));
	td = Y * ((Mq.inverse()) * (C * qd - tao - P - td));
}