#include <iostream>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include "coder_array.h"
#include <Eigen/Dense>
#include "m_c_g_utility.h"
using namespace Eigen;

Eigen::Matrix3d exp_r_E(const Eigen::Vector3d &r)
{
    if (r.norm() > 1e-10)
    {
        Eigen::AngleAxisd angax(r.norm(), r.normalized());
        return angax.toRotationMatrix();
    }
    return Eigen::Matrix3d::Identity();
}

void exp_w_E(const RVector3d w, Matrix3d& R)
{
    R = Matrix3d::Zero();
    double theta;
    theta = w.norm();
    if (theta <= 0)
    {
        R = Matrix3d::Identity();
    }
    else
    {
        RVector3d w_hat;
        w_hat = w / theta;
        Matrix3d so_w_hat;
        so_w_hat << 0, -w_hat(2), w_hat(1),
            w_hat(2), 0, -w_hat(0),
            -w_hat(1), w_hat(0), 0;
        Matrix3d so_w_hat2;
        so_w_hat2 = so_w_hat * so_w_hat;
        R = Matrix3d::Identity() + sin(theta) * so_w_hat + (1 - cos(theta)) * so_w_hat2;
    }

}

void normalize_twist_E(const RVector6d v, RVectornd& sa)
{
    double eps = 2e-16;
    int n = 7;
    sa = RVectornd::Zero();

    for (int i = 0; i < 1; i++)
    {
        if (v.norm() <= eps) {
            sa(n - 2) = 1;
            continue;
        }
        // std::cout<<"here"<<std::endl;
        double theta = v.head(3).norm();
        if (theta <= eps) {
            sa(n - 1) = v.tail(3).norm();
            Vector3d sa456;
            // std::cout<<sa<<std::endl;
            sa456 = v.tail(3) / sa(n - 1);
            // std::cout<<v<<std::endl;
            // std::cout<<"here"<<std::endl;
            Vector3d sa123;
            sa123 = sa.head(3);
            sa << sa123(0), sa123(1), sa123(2), sa456(0), sa456(1), sa456(2), sa(n - 1);
            // std::cout<<"here"<<std::endl;
        }
        else
        {
            sa(n - 1) = theta;
            sa.head(6) = v / theta;
            sa << sa.head(6), sa(n - 1);
        }
        // std::cout<<"here"<<std::endl;
    }


}

void exp_twist_E(const RVector6d v, Matrix4d& tform)
{
    // tform = Matrix4d::Zero();
    // tform(3,3) = 1;
    tform << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1;
    RVectornd sa;
    // std::cout<<"here"<<std::endl;
    normalize_twist_E(v, sa);
    RVector6d s;
    s = sa.head(6);
    double theta = sa(6);
    Vector3d t = s.head(3);
    Matrix3d W;
    W << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    RVector3d v_w;
    v_w = v.head(3);
    Matrix3d R;
    exp_w_E(v_w, R);
    tform.block(0, 0, 3, 3) = R;
    tform.block(0, 3, 3, 1) = (Matrix3d::Identity() * theta +
        (1 - cos(theta)) * W + (theta - sin(theta)) * W * W) * s.tail(3).transpose();
    // std::cout << W <<", W" << std::endl;
    // std::cout << theta <<", theta" << std::endl;
    // std::cout << s << std::endl;

    // tform.block(0, 0, 3, 3) = 
    // std::cout<<"here"<<std::endl;
}

void tform_inv_E(const Matrix4d T, Matrix4d& invT)
{
    invT = Matrix4d::Zero();
    // std::cout<<"here"<<std::endl;
    invT << T.block(0, 0, 3, 3).transpose(), -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1),
        0, 0, 0, 1;
    // std::cout<<"here"<<std::endl;
}

void se_twist_E(const Vector6d v, Matrix4d& Tv)
{
    Matrix3d sk_v;
    sk_v << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    Tv << sk_v, v.tail(3),
        0, 0, 0, 0;

}

void adjoint_T_E(const Matrix4d tform, Matrix6d& Adt)
{
    Adt = Matrix6d::Zero();
    Adt.block(0, 0, 3, 3) = tform.block(0, 0, 3, 3);
    Adt.block(3, 3, 3, 3) = tform.block(0, 0, 3, 3);
    Matrix3d sk_t;
    Vector3d t;
    t = tform.block(0, 3, 3, 1);
    sk_t << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    Adt.block(3, 0, 3, 3) = sk_t * tform.block(0, 0, 3, 3);
}

void derivative_adjoint_T_E(const Matrix4d T, const Matrix4d dT,
    Matrix6d& dAdT, Matrix6d& AdT)
{
    // std::cout<<"here"<<std::endl;
    Matrix3d R = T.block(0, 0, 3, 3);
    // std::cout<<"here"<<std::endl;
    // Vector3d t = T.block(0,2,3,3).transpose();
    Vector3d t;
    t << T(0, 3), T(1, 3), T(2, 3);
    // std::cout<<"here"<<std::endl;
    Matrix3d dR = dT.block(0, 0, 3, 3);
    // Vector3d dt = dT.block(0,2,3,3).transpose();
    Vector3d dt;
    dt << dT(0, 3), dT(1, 3), dT(2, 3);
    Matrix3d sk_t;
    Matrix3d sk_dt;
    // std::cout<<"here"<<std::endl;
    sk_t << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;

    sk_dt << 0, -dt(2), dt(1),
        dt(2), 0, -dt(0),
        -dt(1), dt(0), 0;
    // std::cout<<"here"<<std::endl;
    dAdT << dR, Matrix3d::Zero(),
        sk_dt* R + sk_t * dR, dR;
    // std::cout<<"here"<<std::endl;
    AdT << R, Matrix3d::Zero(),
        //     std::cout<<"here"<<std::endl;
        sk_t* R, R;
}

void m_c_g_matrix_E(const Robot* robot, const Vectornd q, const Vectornd qd,
    Matrixnd& Mq, Matrixnd& C,
    Vectornd& g, Matrix6xnd& Jb,
    Matrix6xnd& dJb, Matrixnd& dMq,
    Matrix4d& dTcp, Matrix4d& Tcp)
{
    double n = robot->dof;
    //coder::array<double, 1U> mass_t;
    coder::array<double, 3U> inertia_t = robot->inertia;
    //coder::array<double, 3U> inertia_t;
    coder::array<double, 2U> A_t = robot->A;
    coder::array<double, 3U> M_t = robot->M;
    Vectornd mass; //= robot->mass;
    Matrix3x3nd inertia; //= robot->inertia;
    Matrixnx6d A; //= robot->A;
    Matrix4x4nd M; //= robot->M;
    //Matrix3d inertia;
    //inertia.Zero();
    for (int i = 0; i < n; i++)
    {
        inertia.block(0, 3 * i, 3, 3) <<
            inertia_t[0 + 9 * i], inertia_t[1 + 9 * i], inertia_t[2 + 9 * i],
            inertia_t[3 + 9 * i], inertia_t[4 + 9 * i], inertia_t[5 + 9 * i],
            inertia_t[6 + 9 * i], inertia_t[7 + 9 * i], inertia_t[8 + 9 * i];
    }

    //std::cout << inertia << std::endl << std::endl;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            //A.block(i, j, 1, 1) << A_t.data()[i, j];
            A.block(i, j, 1, 1) << A_t[i + j * n];
        }

    }

    //std::cout << M_t.data()[0,0,1] << std::endl;
    // M.Zero();
    M = Matrix4x4nd::Zero();
    for (int i = 0; i < n; i++)
    {
        M.block(0, 4 * i, 4, 4) <<
            M_t[16 * i + 0], M_t[16 * i + 4], M_t[16 * i + 8], M_t[16 * i + 12],
            M_t[16 * i + 1], M_t[16 * i + 5], M_t[16 * i + 9], M_t[16 * i + 13],
            M_t[16 * i + 2], M_t[16 * i + 6], M_t[16 * i + 10], M_t[16 * i + 14],
            M_t[16 * i + 3], M_t[16 * i + 7], M_t[16 * i + 11], M_t[16 * i + 15];
    }
    Matrix4d ME;
    mass << robot->mass[0], robot->mass[1], robot->mass[2], robot->mass[3], robot->mass[4], robot->mass[5], robot->mass[6];


    //std::cout << A << std::endl << std::endl;

    //ME << robot->ME[0], robot->ME[1], robot->ME[2], robot->ME[3],
    //    robot->ME[4], robot->ME[5], robot->ME[6], robot->ME[7],
    //    robot->ME[8], robot->ME[9], robot->ME[10], robot->ME[11],
    //    robot->ME[12], robot->ME[13], robot->ME[14], robot->ME[15];//11 and 14 is inverted

    ME << robot->ME[0], robot->ME[4], robot->ME[8], robot->ME[12],
        robot->ME[1], robot->ME[5], robot->ME[9], robot->ME[13],
        robot->ME[2], robot->ME[6], robot->ME[10], robot->ME[14],
        robot->ME[3], robot->ME[7], robot->ME[11], robot->ME[15];

    //std::cout << ME << std::endl;

    Vector3d gravity;
    gravity << robot->gravity[0], robot->gravity[1], robot->gravity[2];
    Matrix6xnnd J = Matrix6xnnd::Zero();
    Matrix6xnnd dJ = Matrix6xnnd::Zero();
    // Matrixnd Mqq;
    // Matrixnd dMqq;
    Matrixnxnnd pdMq = Matrixnxnnd::Zero();
    Matrixnd Cq;
    Matrix6x6nd G = Matrix6x6nd::Zero();
    Matrixnd P = Matrixnd::Zero();
    //Vectornd gq;
    Matrix6xnd Jbb;
    Matrix6xnd dJbb;
    Matrix6xnnnd pdJ = Matrix6xnnnd::Zero();
    Mq = Matrixnd::Zero();
    C = Matrixnd::Zero();
    g = Vectornd::Zero();
    Jb = Matrix6xnd::Zero();
    dJb = Matrix6xnd::Zero();
    dMq = Matrixnd::Zero();
    dTcp = Matrix4d::Zero();
    Tcp = Matrix4d::Zero();

    static double ii = 0;

    for (int i = n; i >= 1; i--)
    {
        G.block(0, 6 * (i - 1), 3, 3) = inertia.block(0, 3 * (i - 1), 3, 3);
        G.block(3, 6 * i - 3, 3, 3) = mass(i - 1) * Matrix3d::Identity();
        Matrix4d T = Matrix4d::Identity();
        Matrix4d dT = Matrix4d::Zero();
        Matrix4x4nd pdT = Matrix4x4nd::Zero();
        Matrix6xnd J_i = J.block(0, n * (i - 1), 6, 7);
        Matrix6xnd dJ_i = J.block(0, n * (i - 1), 6, 7);
        Vector3d drc;

        Matrix6d G_i = G.block(0, 6 * (i - 1), 6, 6);
        for (int j = i; j >= 1; j--)
        {
            Matrix6d dAdT;
            Matrix6d AdT;
            Matrix4d M_j;
            M_j = M.block(0, 4 * (j - 1), 4, 4);
            derivative_adjoint_T_E(T, dT, dAdT, AdT);
            J_i.block(0, j - 1, 6, 1) = AdT * A.row(j - 1).transpose();
            dJ_i.block(0, j - 1, 6, 1) = dAdT * A.row(j - 1).transpose();

            Vector6d v;
            Matrix4d invM_j;
            tform_inv_E(M_j, invM_j);
            v = -A.row(j - 1) * q(j - 1);
            Matrix4d tform;
            exp_twist_E(v, tform);
            tform = tform * invM_j;

            Matrix4d Tv;
            Matrix4d pdtform;
            se_twist_E(-A.row(j - 1), Tv);
            pdtform = Tv * tform;
            for (int k = 1; k <= i; k++)
            {
                Matrix6d pAdT;
                Matrix6d pdAdT;
                derivative_adjoint_T_E(T, pdT.block(0, 4 * (k - 1), 4, 4), pdAdT, pAdT);
                pdJ.block(6 * (k - 1), n * (i - 1) + j - 1, 6, 1) = pdAdT * A.row(j - 1).transpose();
                pdT.block(0, 4 * (k - 1), 4, 4) = pdT.block(0, 4 * (k - 1), 4, 4) * tform;
                // if (k == j)
                // {
                //     pdT.block(0, 4 * (k - 1), 4, 4) = pdT.block(0, 4 * (k - 1), 4, 4) + T * pdtform;
                // }
            }
            dT = (dT + T * Tv * qd(j - 1)) * tform;
            T = T * tform;
        }
        Mq = Mq + J_i.transpose() * G_i * J_i;
        dMq = dMq + dJ_i.transpose() * G_i * J_i + J_i.transpose() * G_i * dJ_i;
        /* if (i == 1 && ii == 0)
              {
                  std::cout << Mq << std::endl;
                  ii = 1;
              }*/
        for (int k = 1; k <= i; k++)
        {
            drc = -pdT.block(0, 4 * (k - 1), 3, 3).transpose() * T.block(0, 3, 3, 1)
                - T.block(0, 0, 3, 3).transpose() * pdT.block(0, 4 * k - 1, 3, 1);
            P(i - 1, k - 1) = -mass(i - 1) * gravity.dot(drc);
            /*if (k == 1 && ii == 0)
                {/;/../
                    std::cout << drc << std::endl;
                    ii = 1;
                }*/
            J.block(0, n * (i - 1), 6, 7) = J_i;
            dJ.block(0, n * (i - 1), 6, 7) = dJ_i;
        }
        if (i == n)
        {
            Matrix4d invT;
            tform_inv_E(T, invT);
            Tcp = invT * ME;
            dTcp = -invT * dT * invT * ME;
            Jb = J.block(0, n * (n - 1), 6, 7);
            dJb = dJ.block(0, n * (n - 1), 6, 7);
        }
        /*   if (i == 1 && ii == 0)
           {
               std::cout << drc << std::endl << std::endl;

               std::cout << T << std::endl << std::endl;

               std::cout << pdT << std::endl;
               ii = 1;
           }*/
    }

    for (int i = 1; i <= n; i++)
    {
        for (int j = 1; j <= n; j++)
        {
            pdMq.block(0, n * (i - 1), 7, 7) = pdMq.block(0, n * (i - 1), 7, 7) +
                pdJ.block(6 * (j - 1), 7 * (i - 1), 6, 7).transpose() * G.block(0, 6 * (j - 1), 6, 6) * J.block(0, n * (j - 1), 6, 7) +
                J.block(0, n * (j - 1), 6, 7).transpose() * G.block(0, 6 * (j - 1), 6, 6) * pdJ.block(6 * (j - 1), 7 * (i - 1), 6, 7);

            /*if (i == 7 && ii == 0)
            {
                std::cout << g << std::endl << std::endl;
                ii = 1;
            }*/

            g(i - 1) = g(i - 1) + P(j - 1, i - 1);
        }
    }
    //std::cout << g << std::endl << std::endl;

     //std::cout << P << std::endl;
    //std::cout << pdMq << std::endl;
    Matrix4d Tb;
    tform_inv_E(ME, Tb);
    Matrix6d adTb;
    adjoint_T_E(Tb, adTb);
    Jb = adTb * Jb;
    dJb = adTb * dJb;
    for (int k = 1; k <= n; k++)
    {
        for (int j = 1; j <= n; j++)
        {
            for (int i = 1; i <= n; i++)
            {
                int ii = j - 1 + n * (i - 1);
                int jj = n * (j - 1) + i - 1;
                int kk = n * (k - 1) + j - 1;
                // C(k-1,j-1) = C(k-1,j-1) + 0.5*(pdMq(k-1, n*(i-1)+j-1) + pdMq(k-1, n*(j-1)+i-1) - pdMq(i-1, n*(k-1)+j-1))*qd(i-1);
                C(k - 1, j - 1) = C(k - 1, j - 1) + 0.5 * qd(i - 1) * (pdMq(k - 1, ii) + pdMq(k - 1, jj) + pdMq(k - 1, kk));
            }

        }

    }

}
