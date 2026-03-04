#include "utility.h"
#include <string>
#include <iostream>
#include <thread>
#include <limits>
#include "cal_J.h"

void cal_J(const Robot* robot, const Eigen::Vector7d &q, const Eigen::Vector7d &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F,
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J1, Eigen::MatrixXd &dJ1, Eigen::MatrixXd &J2, Eigen::MatrixXd &dJ2, Eigen::Vector3d &p1, Eigen::Vector3d &p2)
{
    Eigen::Matrix4d T1, T2, invT1, invT2;
    Eigen::Matrix3d R, dR;
    Eigen::Vector3d t, v, u, dv, du;
    Eigen::MatrixXd J1_all, dJ1_all, J2_all, dJ2_all;
    Eigen::MatrixXd J_lambda, dJ_lambda;
    double lambda, vn2, vn4;
    Eigen::MatrixXd term1, term2, dterm1, dterm2; 
    
    T1 << Eigen::Matrix3d::Identity(), p1_F,
          0,0,0,1;
    T2 << Eigen::Matrix3d::Identity(), p2_F,
          0,0,0,1;
    R = T.block<3,3>(0,0);
    dR = dT.block<3,3>(0,0); 
    t = T.block<3,1>(0,3);
    tform_inv_E(T1, invT1);
    tform_inv_E(T2, invT2);
    J1_all = adjoint_T(invT1) *Jb;
    dJ1_all = adjoint_T(invT1) *dJb;
    J2_all = adjoint_T(invT2) *Jb;
    dJ2_all = adjoint_T(invT2) *dJb;
    Matrix6d Th = Matrix6d::Identity(), dTh = Eigen::Matrix6d::Zero();
    Th.block<3, 3>(3, 3) = R;
    Th.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    dTh.block<3, 3>(3, 3) = dR;
    J1 = Th*J1_all;
    dJ1 = dTh*J1_all + Th*dJ1_all;
    J2 = Th*J2_all;
    dJ2 = dTh*J2_all + Th*dJ2_all;
    p1 = R*p1_F + t;
    p2 = R*p2_F + t;
}