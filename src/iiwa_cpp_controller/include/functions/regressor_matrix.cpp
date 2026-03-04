#include "utility.h"
#include <string>
#include <iostream>
#include "regressor_matrix.h"


void regressor_matrix(const Robot* robot, const Eigen::Vector7d q, const Eigen::Vector7d dq, 
const Eigen::Vector7d a, const Eigen::Vector7d v, const Eigen::Vector7d r, Eigen::MatrixYd &Y, Eigen::MatrixYTrd &YTr)
{
    double n = robot->dof;
    coder::array<double, 2U> A_t = robot->A;
    coder::array<double, 3U> M_t = robot->M;
    Matrixnx6d A;
    Matrix4x4nd M;
    M = Matrix4x4nd::Zero();
    Matrix6xnnd J = Matrix6xnnd::Zero(), dJ = Matrix6xnnd::Zero();
    for (int i = 0; i < n; i++)
    {
        M.block(0, 4 * i, 4, 4) <<
            M_t[16 * i + 0], M_t[16 * i + 4], M_t[16 * i + 8], M_t[16 * i + 12],
            M_t[16 * i + 1], M_t[16 * i + 5], M_t[16 * i + 9], M_t[16 * i + 13],
            M_t[16 * i + 2], M_t[16 * i + 6], M_t[16 * i + 10], M_t[16 * i + 14],
            M_t[16 * i + 3], M_t[16 * i + 7], M_t[16 * i + 11], M_t[16 * i + 15];
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            A.block(i, j, 1, 1) << A_t[i + j * n];
        }

    }

    // std::cout << A << "\n\n";
    
    Vector6d gravity(0,0,0,robot->gravity[0], robot->gravity[1], -robot->gravity[2]); 

    for (int i = 0; i < n; ++i)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity(), dT = Eigen::Matrix4d::Zero();
        for (int j = i; j >= 0; --j)
        {
            Eigen::Matrix6d dAdT = Eigen::Matrix6d::Zero(), AdT = Eigen::Matrix6d::Zero();
            derivative_adjoint_T_E(T, dT, dAdT, AdT);
            J.block<6,1>(0,7*i + j) = AdT * A.row(j).transpose();
            dJ.block<6,1>(0,7*i + j) = dAdT * A.row(j).transpose();
            Matrix4d M_j, invM_j;
            tform_inv_E(M.block<4,4>(0,j*4), invM_j);
            Matrix4d tform = exp_twist(-A.row(j) * q(j)) * invM_j;
            dT = (dT + T*se_twist(-A.row(j)) * dq(j)) * tform;
            T = T*tform;

        }
        Matrix6x7d Jk = J.block<6,7>(0,i*7), dJk = dJ.block<6,7>(0,i*7);
        Matrix6d Adk = adjoint_T(T);
        Vector6d Vk = Jk*dq;
        Matrix6d adk = adjoint_V(Vk);
        Vector6d rk = Adk*gravity;
        Vector6d alpha = Jk*a + adk*Jk*v + dJk*v + rk;
        Y.block<7, 10>(0,i*10) = Jk.transpose() * (A_matrix(alpha) - adk.transpose() * A_matrix(Jk*v));
        YTr.block<10,1>(i*10,0) = Y.block<7, 10>(0,i*10).transpose() * r;   

    }
}

Eigen::MatrixXd A_matrix(const Eigen::Vector6d &V)
{
    //  
    Eigen::Vector3d w,v;
    Eigen::Matrix3x10d Aw,Av;
    Eigen::Matrix6x10d A;
    Eigen::Matrix3d w_diag = Eigen::Matrix3d::Zero(), w_mat = Eigen::Matrix3d::Zero();
    w = V.head(3);
    v = V.tail(3);
    w_diag.diagonal() << w(0), w(1), w(2);
    w_mat << w(1), w(2), 0, 
            w(0), 0 ,w(2), 
            0, w(0), w(1);
    Aw << Eigen::MatrixXd::Zero(3,1), -so_w(v), w_diag, w_mat;
    Av << v, so_w(w), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
    A.topRows(3) = Aw; 
    A.bottomRows(3) = Av;
    // std::cout << Aw << "\n\n";
    // std::cout << Av << "\n\n";
    return A;
}

Eigen::MatrixYTrd get_dynamics(const Robot* robot)
{
    int n = robot->dof;
    std::vector<double> mass = robot->mass;
    coder::array<double, 2U> com = robot->com;
    coder::array<double, 3U> inertia_t = robot->inertia;
    // Eigen::MatrixXd comMat;
    // comMat.resize(7,3);
    Matrix3x3nd inertia; 
    Eigen::MatrixYTrd param;
    
    for (int i = 0; i < n; i++)
    {
        inertia.block(0, 3 * i, 3, 3) <<
            inertia_t[0 + 9 * i], inertia_t[1 + 9 * i], inertia_t[2 + 9 * i],
            inertia_t[3 + 9 * i], inertia_t[4 + 9 * i], inertia_t[5 + 9 * i],
            inertia_t[6 + 9 * i], inertia_t[7 + 9 * i], inertia_t[8 + 9 * i];
    }
    for (int i = 0; i < n; i++)
    {
        param.segment(10*i, 10) << mass[i], mass[i]*com[i], mass[i]*com[i + n*1], mass[i]*com[i*3 + n*2],
                                   inertia(0,3*i), inertia(1,3*i + 1), inertia(2,3*i + 2),
                                   inertia(0,3*i + 1), inertia(0,3*i + 2), inertia(1,3*i + 2);  
    }

    return param;   
}