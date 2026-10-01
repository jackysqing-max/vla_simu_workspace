#include "utility.h"
#include <string>
#include <iostream>
#include <thread>
#include <limits>
#include "decoupleRCM.h"

Eigen::Matrix6x8d J_external(const Eigen::Matrix6x7d &J1, const Eigen::Matrix6x7d &J2, const double &lamda, const Eigen::Vector3d p1, const Eigen::Vector3d p2)
{
    Eigen::Matrix6x8d J_ext = Eigen::Matrix6x8d::Zero();
    J_ext << J2.bottomRows(3), Eigen::Vector3d::Zero(),
             J1.bottomRows(3) + lamda*(J2.bottomRows(3) - J1.bottomRows(3)), p2 - p1;
    return J_ext;
}

Eigen::Vector3d P_trocar(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double &lamda)
{
    Eigen::Vector3d P_tr = Eigen::Vector3d::Zero();
    P_tr = p1 + lamda*(p2 - p1);
    return P_tr;
}

Eigen::Vector6d RCM_error(const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Tcpd, const Eigen::Vector3d &P_trd, const Eigen::Vector3d &P_tr)
{
    Eigen::Vector3d p2d = Tcpd.block<3,1>(0,3);
    Eigen::Vector3d p2 = Tcp.block<3,1>(0,3);
    Eigen::Vector6d rcm_error = Eigen::Vector6d::Zero();
    rcm_error.head<3>() = p2d - p2;
    rcm_error.tail<3>() = P_trd - P_tr;
    return rcm_error;
}



Eigen::Vector8d generateRCM_dq(const Eigen::Matrix6x8d &J_ext, const Eigen::Matrix3d &Kp2, 
const Eigen::Matrix3d &Kptr, const Eigen::Vector8d &w, const Eigen::Vector6d &e)
{
    Eigen::Vector8d dq_dlamda = Eigen::Vector8d::Zero();
    // Eigen::Matrix8x6d J_ext_pinv = pInv(J_ext);
    Eigen::Matrix8x6d J_ext_pinv = J_ext.transpose()*((J_ext*J_ext.transpose()).inverse());
    // std::cout << J_ext_pinv*J_ext << "\n\n";
    // std::cout << J_ext*J_ext_pinv << "\n\n";
    Eigen::Vector8d Nw = (Eigen::Matrix8d::Identity() - J_ext_pinv*J_ext)*w;
    Eigen::Matrix6d K = Eigen::Matrix6d::Zero();
    K << Kp2, Eigen::Matrix3d::Zero(),
    Eigen::Matrix3d::Zero(), Kptr;

    dq_dlamda = J_ext_pinv*K*e + Nw;

    return dq_dlamda;
}

Eigen::Vector8d generateRCM_tau(const Eigen::Matrix6x8d &J_ext, const Eigen::Matrix3d &Kptr, 
const Eigen::Matrix3d &Bptr, const Eigen::Vector8d &w, const Eigen::Vector6d &e, const Eigen::Vector6d &dp)
{
    Eigen::Vector8d tau_dlamda = Eigen::Vector8d::Zero();
    Eigen::Matrix8x6d J_ext_pinv = J_ext.transpose()*((J_ext*J_ext.transpose()).inverse());
    Eigen::Vector8d Nw = (Eigen::Matrix8d::Identity() - J_ext_pinv*J_ext)*w;

    Eigen::Matrix6d Kp = Eigen::Matrix6d::Zero(), Bp = Eigen::Matrix6d::Zero();
    Kp.bottomRightCorner(3,3) = Kptr;
    Bp.bottomRightCorner(3,3) = Bptr;

    tau_dlamda = J_ext_pinv*(Kp*e - Bp*dp) + Nw;

    return tau_dlamda;
}

Eigen::MatrixXd J1_sharp_null_J2(const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2)
{
    Eigen::MatrixXd pinv_J1 = (J1.transpose() * J1).inverse() * J1.transpose();
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(J1.rows(), J1.rows()) - J2*(J2.transpose() * J2).inverse() * J2.transpose();
    Eigen::MatrixXd J1_sharp = pinv_J1 * P;
    return J1_sharp;
}

Eigen::MatrixXd cal_J_lamda(const Eigen::Vector3d &p_rcm, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2, const Eigen::Vector7d &dq)
{
    Eigen::MatrixXd J;
    J = (-(p2 - p1).transpose()/(p2 - p1).norm())*J1 + 
        ((p_rcm - p1).transpose()/(p2 - p1).norm())*(J2 - J1) + 
        (((p_rcm - p1).transpose()*(p2 - p1)*(p2 - p1).transpose()*(J2 - J1)*dq) / ((p2 - p1).norm()*(p2 - p1).norm()*(p2 - p1).norm()))*(J2 - J1);

    return J;
}

Eigen::MatrixXd cal_dJ_lamda(const Eigen::Vector3d &p_rcm, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, 
const Eigen::MatrixXd &J1, const Eigen::MatrixXd &J2, const Eigen::MatrixXd &dJ1, const Eigen::MatrixXd &dJ2, const Eigen::Vector7d &dq)
{
    Eigen::MatrixXd dJ, dJ_1, dJ_2, dJ_3, part_1, part_2, part_3;
    Eigen::Vector3d v = p2 - p1, u = p_rcm - p1, dv = J2.bottomRows(3)*dq - J1.bottomRows(3)*dq, du = -J1.bottomRows(3)*dq;

    part_1 = -(dv.transpose()*v.norm() - v.transpose()*(v*dv.transpose()/v.norm())) / (v.norm()*v.norm());

    dJ_1 = part_1*J1 + (-v.transpose()/v.norm())*dJ1;
    
    part_2 = (du.transpose()*v.norm() - u.transpose()*((v*dv.transpose())/v.norm())) / (v.norm()*v.norm());
    
    dJ_2 = part_2*(J2 - J1) + (u.transpose()/v.norm())*(dJ2 - dJ1);
    
    part_3 = ((du.transpose()*v*v.transpose() + u.transpose()*dv*v.transpose() + u.transpose()*v*dv.transpose())*(v.norm()*v.norm()*v.norm())
     - u.transpose()*v*v.transpose()*3*(v.norm()*v.norm())*((v*dv.transpose())/v.norm())) / (v.norm()*v.norm()*v.norm()*v.norm()*v.norm()*v.norm());
    
    dJ_3 = part_3*(J2 - J1) + ((u.transpose()*v*v.transpose()) / (v.norm()*v.norm()*v.norm()))*(dJ2 - dJ1);
    
    dJ = dJ_1 + dJ_2 + dJ_3;
    
    return dJ;
}

void rcm_Jacobian(const Robot* robot, const Eigen::Vector7d &q, const Eigen::Vector7d &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F, const Eigen::Vector3d &rcm, 
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J, Eigen::MatrixXd &dJ, Eigen::Vector3d &x)
{
    Eigen::Matrix4d T1, T2, invT1, invT2;
    Eigen::Matrix3d R, dR;
    Eigen::Vector3d t, p1, p2, v, u, dv, du;
    Eigen::MatrixXd J1_all, dJ1_all, J2_all, dJ2_all;
    Eigen::MatrixXd J1, J2, dJ1, dJ2, J_lambda, dJ_lambda;
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
    J1 = R*J1_all.bottomRows(3);
    dJ1 = dR*J1_all.bottomRows(3) + R*dJ1_all.bottomRows(3);
    J2 = R*J2_all.bottomRows(3);
    dJ2 = dR*J2_all.bottomRows(3) + R*dJ2_all.bottomRows(3);
    p1 = R*p1_F + t;
    p2 = R*p2_F + t;
    v = p2 - p1;
    u = rcm - p1;
    dv = (J2 - J1)*dq;
    du = -J1*dq;
    vn2 = v.dot(v);
    vn4 = vn2*vn2;
    lambda = u.dot(v) / vn2;
    x = p1 + lambda*(p2 - p1);
    term1 = 2*u.dot(v)*v.transpose() - (u + v).transpose() / vn2;
    term2 = u.transpose() / vn2 - 2*u.dot(v) * v.transpose();
    
    // std::cout << dJ1 << "\n\n";
    // std::cout << dJ2 << "\n\n";

    J_lambda = term1*J1 + term2*J2;
    dterm1 = 2*(du.dot(v) + u.dot(dv)) * v.transpose() + 2*u.dot(v)*dv.transpose() - ((du + dv).transpose()*vn2 - 2*v.dot(dv)*(u + v).transpose()) / vn4;
    dterm2 = (du.transpose() * vn2 - 2*v.dot(dv)*u.transpose()) / vn4 - (2*(du.dot(v) + u.dot(dv))*v.transpose() + 2*u.dot(v)*dv.transpose());
    dJ_lambda = dterm1*J1 + term1*dJ1 + dterm2*J2 + term2*dJ2;
    
    J = J1 + lambda*(J2 - J1) + v*J_lambda;
    double dlambda = (J_lambda*dq)(0,0);
    dJ = dJ1 + dlambda*(J2 - J1) + lambda*(dJ2 - dJ1) + dv*J_lambda + v*dJ_lambda;
    /*MatrixXd a,b,c,d;
    a = dJ1 + J_lambda*dq*(J2 - J1);
    b = lambda*(dJ2 - dJ1);
    c = dv*J_lambda;
    d = v*dJ_lambda;*/
    // dJ = a+b+c+d;

}

void rcm_motion_Jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, 
const Eigen::Vector3d &p1_F, const Eigen::Vector3d &p2_F, const Eigen::Vector3d &rcm, 
const Eigen::MatrixXd &Jb, const Eigen::MatrixXd &dJb, const Eigen::Matrix4d &T, const Eigen::Matrix4d &dT,
Eigen::MatrixXd &J, Eigen::MatrixXd &dJ, Eigen::VectorXd &x)
{
    Eigen::Matrix4d T1, T2, invT1, invT2;
    Eigen::Matrix3d R, dR;
    Eigen::Vector3d t, p1, p2, v, u, dv, du, trocar;
    Eigen::MatrixXd J1_all, dJ1_all, J2_all, dJ2_all;
    Eigen::MatrixXd J1, J2, dJ1, dJ2, J_lambda, dJ_lambda, J_rcm, dJ_rcm;
    double lambda, vn2, vn4;
    int n = q.size();
    Eigen::MatrixXd term1, term2, dterm1, dterm2; 
    
    T1 << Eigen::Matrix3d::Identity(), p1_F,
          0,0,0,1;
    T2 << Eigen::Matrix3d::Identity(), p2_F,
          0,0,0,1;
    R = T.block<3,3>(0,0);
    dR = dT.block<3,3>(0,0); 
    t = T.block<3,1>(0,3);
    invT1 = invertT(T1);
    invT2 = invertT(T2);
    J1_all = adjoint_T(invT1) *Jb;
    dJ1_all = adjoint_T(invT1) *dJb;
    J2_all = adjoint_T(invT2) *Jb;
    dJ2_all = adjoint_T(invT2) *dJb;
    J1 = R*J1_all.bottomRows(3);
    dJ1 = dR*J1_all.bottomRows(3) + R*dJ1_all.bottomRows(3);
    J2 = R*J2_all.bottomRows(3);
    dJ2 = dR*J2_all.bottomRows(3) + R*dJ2_all.bottomRows(3);
    p1 = R*p1_F + t;
    p2 = R*p2_F + t;
    v = p2 - p1;
    u = rcm - p1;
    dv = (J2 - J1)*dq;
    du = -J1*dq;
    vn2 = v.dot(v);
    vn4 = vn2*vn2;
    lambda = u.dot(v) / vn2;
    trocar = p1 + lambda*(p2 - p1);
    term1 = 2*u.dot(v)*v.transpose() - (u + v).transpose() / vn2;
    term2 = u.transpose() / vn2 - 2*u.dot(v) * v.transpose();
    
    // std::cout << dJ1 << "\n\n";
    // std::cout << dJ2 << "\n\n";

    J_lambda = term1*J1 + term2*J2;
    dterm1 = 2*(du.dot(v) + u.dot(dv)) * v.transpose() + 2*u.dot(v)*dv.transpose() - ((du + dv).transpose()*vn2 - 2*v.dot(dv)*(u + v).transpose()) / vn4;
    dterm2 = (du.transpose() * vn2 - 2*v.dot(dv)*u.transpose()) / vn4 - (2*(du.dot(v) + u.dot(dv))*v.transpose() + 2*u.dot(v)*dv.transpose());
    dJ_lambda = dterm1*J1 + term1*dJ1 + dterm2*J2 + term2*dJ2;
    
    J_rcm = J1 + lambda*(J2 - J1) + v*J_lambda;
    double dlambda = (J_lambda*dq)(0,0);
    dJ_rcm = dJ1 + dlambda*(J2 - J1) + lambda*(dJ2 - dJ1) + dv*J_lambda + v*dJ_lambda;
    J.resize(6, n);
    dJ.resize(6, n);
    x.resize(6);
    J << J_rcm, J2;
    dJ << dJ_rcm, dJ2;
    x << trocar, p2;

}