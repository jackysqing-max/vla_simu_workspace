#include "robot.h"
#include <json/json.h>
#include <fstream>
#include <iostream>
#include <exception>
#include <Eigen/Dense>
#include <utility.h>
#include <derivative_jacobian_matrix.h>
#include <exp_twist.h>
#include <pseudo.h>
#include <forward_kin_general.h>
#include <inverse_kin_general.h>
#include <jacobian_matrix.h>
#include <m_c_g_matrix.h>
class jsexception: public std::exception
{
  virtual const char* what() const throw()
  {
    return "can not read robot json file";
  }
} jsex;

Robot loadRobot(const char* filename)
{
    Json::Value root;
    std::ifstream ifs;
    ifs.open(filename);

    Json::CharReaderBuilder builder;
    //builder["collectComments"] = true;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &root, &errs)) {
        throw jsex;
    }
    Robot robot;
    int n = root["dof"].asInt();
    robot.dof = n;
    robot.mass.set_size(n);
    robot.inertia.set_size(3, 3, n);
    robot.com.set_size(n, 3);
    robot.A.set_size(n, 6);
    robot.M.set_size(4, 4, n);
    for (int i = 0; i < n; i++)
    {
        robot.mass[i] = root["mass"][i].asDouble();
        Eigen::Map<Eigen::Matrix3d> inertia(&robot.inertia[0] + 9 * i);
        Eigen::Map<Eigen::Matrix4d> M(&robot.M[0] + 16 * i);
        inertia << root["inertia"][i][0].asDouble(), root["inertia"][i][5].asDouble(), root["inertia"][i][4].asDouble(),
            root["inertia"][i][5].asDouble(), root["inertia"][i][1].asDouble(), root["inertia"][i][3].asDouble(),
            root["inertia"][i][4].asDouble(), root["inertia"][i][3].asDouble(), root["inertia"][i][2].asDouble();
        //std::cout << inertia << std::endl;
        Eigen::Vector3d w, t;
        w << root["M"][i][0].asDouble(), root["M"][i][1].asDouble(), root["M"][i][2].asDouble();
        t << root["M"][i][3].asDouble(), root["M"][i][4].asDouble(), root["M"][i][5].asDouble();
        M << exp_r(w), t, 0, 0, 0, 1;
        //std::cout << M << std::endl;
        for (int j = 0; j < 6; j++)
            robot.A[i + n * j] = root["A"][i][j].asDouble();
        for (int j = 0; j < 3; j++)
            robot.com[i + n * j] = root["com"][i][j].asDouble();
    }

    for(int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            robot.ME[i + 4 * j] = root["ME"][i][j].asDouble();
            robot.TCP[i + 4 * j] = root["TCP"][i][j].asDouble();
        }
    robot.gravity[0] = root["gravity"][0].asDouble();
    robot.gravity[1] = root["gravity"][1].asDouble();
    robot.gravity[2] = root["gravity"][2].asDouble();

    /*Eigen::Map<Eigen::Matrix<double, 7, 6>> A(&robot.A[0]);
    Eigen::Map<Eigen::Matrix<double, 7, 3>> com(&robot.com[0]);
    Eigen::Map<Eigen::Matrix4d> ME(&robot.ME[0]);
    Eigen::Map<Eigen::Matrix4d> TCP(&robot.TCP[0]);
    Eigen::Map<Eigen::Vector3d> G(&robot.gravity[0]);
    std::cout << A << std::endl;
    std::cout << com << std::endl;
    std::cout << ME << std::endl;
    std::cout << TCP << std::endl;
    std::cout << G << std::endl;*/
    return robot;
}
void jacobian_matrix(const Robot* robot, const std::vector<double>& q, Eigen::MatrixXd& J, Eigen::Matrix4d& T)
{
    J.resize(6, q.size());
    coder::array<double, 2> J_array;
    J_array.set(J.data(), 6, J.cols());
    jacobian_matrix(robot, q, J_array, T.data());
}


void m_c_g_matrix(const Robot* robot, const std::vector<double>& q, const std::vector<double>& dq, Eigen::MatrixXd& M,
    Eigen::MatrixXd& C, Eigen::VectorXd& G, Eigen::MatrixXd& J, Eigen::MatrixXd& dJ,
    Eigen::MatrixXd& dM, Eigen::Matrix4d& dT, Eigen::Matrix4d& T)
{
    int n = q.size();
    M.resize(n, n);
    dM.resize(n, n);
    C.resize(n, n);
    J.resize(6, n);
    dJ.resize(6, n);
    G.resize(n);
    coder::array<double, 2> M_array, dM_array, C_array, J_array, dJ_array;
    coder::array<double, 1> G_array;
    M_array.set(M.data(), n, n);
    dM_array.set(dM.data(), n, n);
    C_array.set(C.data(), n, n);
    J_array.set(J.data(), 6, n);
    dJ_array.set(dJ.data(), 6, n);
    G_array.set(G.data(), n);
    m_c_g_matrix(robot, q, dq, M_array, C_array, G_array, J_array, dJ_array, dM_array, dT.data(), T.data());
}

void derivative_jacobian_matrix(const Robot* robot, const std::vector<double>& q, const std::vector<double>& dq, Eigen::MatrixXd& dJ, Eigen::MatrixXd& J, Eigen::Matrix4d& dT, Eigen::Matrix4d& T)
{
    int n = q.size();
    J.resize(6, n);
    dJ.resize(6, n);
    coder::array<double, 2> J_array;
    coder::array<double, 2> dJ_array;
    J_array.set(J.data(), 6, J.cols());
    dJ_array.set(dJ.data(), 6, dJ.cols());
    derivative_jacobian_matrix(robot, q, dq, dJ_array, J_array, dT.data(), T.data());
}

void gravityAndInertComp(const Robot& robot, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd, double _T[16], double _Tcb[16], double _Tcs[16], float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double>& pose, const double _G[36]) // force:fxyzTxyz _Tcs:sensor frame based on gravity center frame
{
    getExternalForce(force, mass, offset, cog, pose);
    double _Vb[6]{ 0 }, _dVb[6]{ 0 };
    getTaskSpaceMotion(robot, q, qd, qdd, _T, _Vb, _dVb);
    Eigen::Vector6d Vc = Eigen::Vector6d::Zero();
    Eigen::Vector6d dVc = Eigen::Vector6d::Zero();
    Eigen::Map<const Eigen::Vector6d> Vb(_Vb);
    Eigen::Map<const Eigen::Vector6d> dVb(_dVb);
    Eigen::Map<const Eigen::Matrix4d> Tcb(_Tcb);
    Vc = adjoint_T(Tcb) * Vb;
    dVc = adjoint_T(Tcb) * dVb;
    //  bodyTwist2SpatialTwist(_Tcb, _Vb, _dVb, _Vc, _dVc);
    Eigen::Map<const Eigen::Matrix6d> G(_G);
    Eigen::Map<const Eigen::Matrix4d> Tcs(_Tcs);

    Eigen::Vector6d Fc = Eigen::Vector6d::Zero();
    Eigen::Matrix6d adV = Eigen::Matrix6d::Zero();
    adV.block(0, 0, 3, 3) = so_w(Vc.topRows(3));
    adV.block(3, 0, 3, 3) = so_w(Vc.bottomRows(3));
    adV.block(3, 3, 3, 3) = so_w(Vc.topRows(3));
    Fc = G * dVc - adV.transpose() * G * Vc;
    Eigen::Vector6d Fsensor = Eigen::Vector6d::Zero();
    Fsensor = adjoint_T(Tcs).transpose() * Fc;
    force[0] -= static_cast<float>(Fsensor(3));
    force[1] -= static_cast<float>(Fsensor(4));
    force[2] -= static_cast<float>(Fsensor(5));
    force[3] -= static_cast<float>(Fsensor(0));
    force[4] -= static_cast<float>(Fsensor(1));
    force[5] -= static_cast<float>(Fsensor(2));
}


void getTaskSpaceMotion(const Robot& robot, const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd, double T[16], double V[6], double dV[6])
{
    int n = q.size();
    double dT[16] = { 0 };
    Eigen::Map<Eigen::Vector6d> twist(V), dtwist(dV);
    if (n == 7)
    {
        Eigen::Matrix6x7d Jb, dJb;
        Eigen::Map<const Eigen::Vector7d> js(&qd[0]), ja(&qdd[0]);
        derivative_jacobian_matrix(&robot, q, qd, coder_array_wrapper1(dJb), coder_array_wrapper2(Jb), dT, T);
        twist = Jb * js;
        dtwist = dJb * js + Jb * ja;
    }
    else if (n == 6)
    {
        Eigen::Matrix6d Jb, dJb;
        Eigen::Map<const Eigen::Vector6d> js(&qd[0]), ja(&qdd[0]);
        derivative_jacobian_matrix(&robot, q, qd, coder_array_wrapper1(dJb), coder_array_wrapper2(Jb), dT, T);
        twist = Jb * js;
        dtwist = dJb * js + Jb * ja;
    }
}

Eigen::Vector6d gravityAndInertiaCompensation(const Robot& robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Tsensor, const std::vector<double>& q, const std::vector<double>& qd,
    const std::vector<double>& qdd, const float* rawForce, float mass, const float offset[6], const float cog[3], const Eigen::Matrix3d& mI, double scale)
{
    Eigen::Vector3d com(cog[0], cog[1], cog[2]);
    Eigen::Vector3d Pcom = Tsensor.block(0, 3, 3, 1) + Tsensor.block(0, 0, 3, 3) * com;
    Eigen::Matrix4d Tcb = Eigen::Matrix4d::Identity();
    Tcb.block(0, 3, 3, 1) = -Pcom;
    Eigen::Matrix6d adTcb = adjoint_T(Tcb);
    Eigen::Matrix6d g = Eigen::Matrix6d::Identity();
    g(3, 3) = g(4, 4) = g(5, 5) = mass;
    g.topLeftCorner(3, 3) = mI;
    Eigen::Vector6d Vc, dVc, Vb, dVb;
    Eigen::Matrix4d T;
    getTaskSpaceMotion(robot, q, qd, qdd, T.data(), Vb.data(), dVb.data());
    Vc = adTcb * Vb;
    dVc = adTcb * dVb;
    Eigen::Vector6d Ftotal = g * dVc - adjoint_V(Vc).transpose() * g * Vc;
    Eigen::Vector6d Fg(0, 0, 0, 0, 0, 0);
    Fg.bottomRows(3) = mass * 9.8 * T.block(0, 0, 3, 3).transpose() * Eigen::Vector3d(0, 0, -1);
    Eigen::Vector6d rawWrench(rawForce[3] - offset[3], rawForce[4] - offset[4], rawForce[5] - offset[5],
        rawForce[0] - offset[0], rawForce[1] - offset[1], rawForce[2] - offset[2]);
    Eigen::Vector6d Fsensor = adjoint_T(invertT(Tsensor)).transpose() * rawWrench * scale;
    Eigen::Vector6d Fext = adTcb.transpose() * (Fg - Ftotal) - Fsensor;
    return adjoint_T(Tcp).transpose() * Fext;
}

void admittance_error_cal(const Robot* robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Td, const Eigen::Vector6d& Vd,
    const std::vector<double>& q, const std::vector<double>& qd, Eigen::Vector3d& re, Eigen::Vector3d& pe, Eigen::Vector3d& red, Eigen::Vector3d& ped, bool flag)
{
    int n = robot->dof;
    Eigen::Matrix4d T;
    Eigen::MatrixX<double> Jb(6, n);
    coder::array<double, 2> Jb_array;
    Jb_array.set(Jb.data(), 6, n);
    jacobian_matrix(robot, q, Jb_array, T.data());
    T = T * Tcp;
    Jb = adjoint_T(invertT(Tcp)) * Jb;
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Vector3d p = T.block(0, 3, 3, 1);
    Eigen::Matrix3d Rd = Td.block(0, 0, 3, 3);
    Eigen::Vector3d pd = Td.block(0, 3, 3, 1);
    Eigen::Vector6d V = Jb * Eigen::Map<const Eigen::MatrixX<double>>(&qd[0], n, 1);
    if (flag)
        pe = R.transpose() * (pd - p);
    ped = -so_w(V.topRows(3)) * pe + R.transpose() * Vd.bottomRows(3) - V.bottomRows(3);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R.transpose() * Rd, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d tem = svd.matrixU() * svd.matrixV().transpose();
    re = logR(tem);
    double re_norm = re.norm();
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    if (re_norm > 0)
    {
        Eigen::Matrix3d S = so_w(re);
        double re_norm2 = re_norm * re_norm;
        double re_norm3 = re_norm2 * re_norm;
        A = Eigen::Matrix3d::Identity() - (1 - cos(re_norm)) / re_norm2 * S + (re_norm - sin(re_norm)) / re_norm3 * S * S;
    }
    red = A.colPivHouseholderQr().solve(Rd.transpose() * Vd.topRows(3) - Rd.transpose() * R * V.topRows(3));
}

void admittance_control(const Robot* robot, const Eigen::Matrix4d& Tcp, const Eigen::Matrix4d& Td, const Eigen::Vector6d& Vd,
    const Eigen::Matrix3d& Mp, const Eigen::Matrix3d& Bp, const Eigen::Matrix3d& Kp,
    const Eigen::Matrix3d& Mr, const Eigen::Matrix3d& Br, const Eigen::Matrix3d& Kr,
    const std::vector<double>& q, const std::vector<double>& qd, const Eigen::Vector6d& F, double dt,
    Eigen::Vector3d& re, Eigen::Vector3d& pe, Eigen::Vector3d& red, Eigen::Vector3d& ped, bool flag, double* Tcmd)
{

    admittance_error_cal(robot, Tcp, Td, Vd, q, qd, re, pe, red, ped, flag);

    Eigen::Vector3d pedd = Mp.ldlt().solve(F.bottomRows(3) - Bp * ped - Kp * pe);
    Eigen::Vector3d redd = Mr.ldlt().solve(F.topRows(3) - Br * red - Kr * re);

    ped = ped + pedd * dt;
    pe = pe + ped * dt;
    red = red + redd * dt;
    re = re + red * dt;
    if (Tcmd)
    {
        Eigen::Map<Eigen::Matrix4d> T2(Tcmd);
        T2 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Tbs = Eigen::Matrix4d::Identity();
        Tbs.block(0, 0, 3, 3) = Td.block(0, 0, 3, 3).transpose();
        Eigen::Vector6d Vdb = adjoint_T(Tbs) * Vd * dt;
        Eigen::Matrix4d dTd;
        exp_twist(Vdb.data(), dTd.data());
        Eigen::Matrix4d Td2 = Td * dTd;
        T2.block(0, 0, 3, 3) = Td2.block(0, 0, 3, 3) * exp_r(-re);
        T2.block(0, 3, 3, 1) = Td2.block(0, 3, 3, 1) - T2.block(0, 0, 3, 3) * pe;
        T2 = T2 * invertT(Tcp);
    }
}

