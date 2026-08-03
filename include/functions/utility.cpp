#include "utility.h"
#include <string>
#include <iostream>
#include <thread>
#include <limits>
#define SLACK_TIME_IN_MICROS 300UL

Eigen::Matrix4d pose2T(const std::vector<double> &pose)
{
    Eigen::Vector3d rv(pose[3], pose[4], pose[5]);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 3, 3, 1) << pose[0], pose[1], pose[2];
    if (rv.norm() < 1e-10)
        return T;
    Eigen::AngleAxisd angax(rv.norm(), rv.normalized());
    T.block(0, 0, 3, 3) = angax.toRotationMatrix();
    return T;
}

Eigen::Matrix3d exp_r(const Eigen::Vector3d &r)
{
    if (r.norm() > 1e-10)
    {
        Eigen::AngleAxisd angax(r.norm(), r.normalized());
        return angax.toRotationMatrix();
    }
    return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d logR(const Eigen::Matrix3d& R)
{

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R - Eigen::Matrix3d::Identity(), Eigen::ComputeFullV);
    Eigen::Vector3d v = svd.matrixV().col(2);
    Eigen::Vector3d v_hat(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    double phi = atan2(v.dot(v_hat), R.trace() - 1);
    return phi * v;
    /*Eigen::AngleAxisd ax(R);
    return ax.angle() * ax.axis();*/
}

Eigen::Matrix4d exp_twist(const Eigen::Vector6d& twist)
{
    Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
    Eigen::Vector7d Sa = normalize_twist(twist);
    Eigen::Vector6d S = Sa.head(6);
    double theta = Sa(6);
    Eigen::Matrix3d W = so_w(S.head(3));
    tform.block<3, 3>(0, 0) = exp_r(twist.head(3));
    tform.block<3, 1>(0, 3) = (Eigen::Matrix3d::Identity() * theta + (1 - std::cos(theta)) * W + (theta - std::sin(theta)) * W * W) * S.segment<3>(3);
    return tform;
}

Eigen::Vector6d logT(const Eigen::Matrix4d& T)
{
    Eigen::Vector6d V = Eigen::Vector6d::Zero();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    Eigen::Vector3d w = logR(R);
    double theta = w.norm();
    if (theta < std::numeric_limits<double>::epsilon())
        V.tail(3) = p;
    else
    {
        V.head(3) = w;
        Eigen::Matrix3d W = so_w(w / theta);
        Eigen::Matrix3d W2 = W * W;
        V.tail(3) = (Eigen::Matrix3d::Identity() - theta / 2 * W + (1 - theta / (2 * std::tan(theta / 2))) * W2) * p;
    }

    return V;
}

std::vector<double> T2pose(const Eigen::Matrix4d &T)
{
    Eigen::Vector3d w;
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    w = logR(R);
    return {T(0, 3), T(1, 3), T(2, 3), w[0], w[1], w[2]};
}

Eigen::Vector3d cond_Matrix(const Eigen::MatrixXd& A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto singular_values = svd.singularValues();
    int n = singular_values.size();
    return Eigen::Vector3d(singular_values(0) / singular_values(n-1), singular_values(0),singular_values(1));
}

Eigen::MatrixXd pInv(const Eigen::MatrixXd& matrix, double tol)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(s.size());
    for(int i = 0; i < singularValuesInv.size(); ++i)
    {
        if(s(i) <= tol)
        {
            singularValuesInv(i) = 0;
        } else
        {
            singularValuesInv(i) = 1.0 / s(i);
        }
        
    }
    return svd.matrixV()*singularValuesInv.asDiagonal()*svd.matrixU().transpose(); 
}

void getExternalForce(float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose)
{
    Eigen::Matrix4d T = pose2T(pose);
    Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();

    Eigen::Vector3d g = R * Eigen::Vector3d(0, 0, -1) * mass;
    Eigen::Vector3d M = Eigen::Vector3d(cog[0], cog[1], cog[2]).cross(g);
    force[0] -= static_cast<float>(g(0) + offset[0]);
    force[1] -= static_cast<float>(g(1) + offset[1]);
    force[2] -= static_cast<float>(g(2) + offset[2]);
    force[3] -= static_cast<float>(M(0) + offset[3]);
    force[4] -= static_cast<float>(M(1) + offset[4]);
    force[5] -= static_cast<float>(M(2) + offset[5]);
}

Eigen::Matrix3d so_w(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d S;
    S << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return S;
}

Eigen::Matrix4d se_twist(const Eigen::Vector6d& V)
{
    Eigen::Matrix4d Tv = Eigen::Matrix4d::Zero();
    Tv.block<3, 3>(0, 0) = so_w(V.head(3));
    Tv.block<3, 1>(0, 3) = V.tail(3);
    return Tv;
}

Eigen::Vector7d normalize_twist(const Eigen::Vector6d& V)
{
    Eigen::Vector7d Sa = Eigen::Vector7d::Zero();
    Eigen::Vector3d w = V.head(3);
    Eigen::Vector3d v = V.tail(3);
    if (V.norm() < std::numeric_limits<double>::epsilon())
        Sa(5) = 1;
    else if (w.norm() < std::numeric_limits<double>::epsilon())
    {
        Sa(6) = v.norm();
        Sa.segment<3>(3) = v / Sa(6);
    }
    else
    {
        Sa(6) = w.norm();
        Sa.head(6) = V / Sa(6);
    }
    return Sa;
}

Eigen::Matrix4d invertT(const Eigen::Matrix4d &T)
{
    Eigen::Matrix4d invT = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();
    invT.block(0, 0, 3, 3) = R;
    invT.block(0, 3, 3, 1) = -R * T.block(0, 3, 3, 1);
    return invT;
}

Eigen::Matrix6d adjoint_T(const Eigen::Matrix4d &T)
{
    Eigen::Matrix6d AdT = Eigen::Matrix6d::Zero();
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    AdT.block(0, 0, 3, 3) = R;
    AdT.block(3, 3, 3, 3) = R;
    AdT.block(3, 0, 3, 3) = so_w(T.block(0, 3, 3, 1)) * R;
    return AdT;
}

Eigen::Matrix6d adjoint_V(const Eigen::Vector6d &V)
{
    Eigen::Matrix6d AdV;
    Eigen::Matrix3d sk = so_w(V.topRows(3));
    AdV << sk, Eigen::Matrix3d::Zero(), so_w(V.bottomRows(3)), sk;
    return AdV;
}

Eigen::MatrixXd J_sharp(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M)
{
    Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
    return tem * (J * tem).inverse();
}

Eigen::MatrixXd d_J_sharp(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& dM)
{
    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    Eigen::MatrixXd tem = ldlt.solve(J.transpose());
    Eigen::MatrixXd dtem = -ldlt.solve(dM) * ldlt.solve(J.transpose()) + ldlt.solve(dJ.transpose());
    Eigen::MatrixXd tem2 = J * tem;
    return (dtem - tem * (tem2.ldlt().solve((dJ * tem + J * dtem)))) * tem2.inverse();
}

Eigen::MatrixXd d_J_sharp_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& X)
{
    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    Eigen::MatrixXd tem = ldlt.solve(J.transpose());
    Eigen::MatrixXd dtem = -ldlt.solve(dM) * ldlt.solve(J.transpose()) + ldlt.solve(dJ.transpose());
    //Eigen::MatrixXd tem2 = J * tem;
    Eigen::LDLT<Eigen::MatrixXd> ldlt2(J * tem);
    return (dtem - tem * (ldlt2.solve((dJ * tem + J * dtem)))) * ldlt2.solve(X);
}
double c = 1000;
double cd = 5e-2;

Eigen::MatrixXd J_sharp_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& X)
{
    Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
    Eigen::MatrixXd tem2 = J * tem;
    Eigen::Vector3d cond = cond_Matrix(tem2);

    if (cond(0) > c)
    {
        //std::cout << "cond big " << cond(0) <<  "\n";
        double alpha = cd;//std::max(cd, (cond(1) * cond(1) - c * cond(2) * cond(2)) / (c- 1));
        //std::cout <<  (cond(1) * cond(1) + alpha) / (cond(2) * cond(2) + alpha) <<  "\n";
        Eigen::MatrixXd tem3 = tem2.transpose() * tem2;
        Eigen::VectorXd d(tem2.cols());
        d.setOnes();
        tem3.diagonal() += alpha * d;
        return tem * (tem3).ldlt().solve(tem2.transpose() * X);
    }

    else
       return tem * (tem2).ldlt().solve(X);
    //    return tem * pInv(tem2) * X;
}

Eigen::MatrixXd J_sharp_T_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& X)
{
    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    Eigen::MatrixXd tem = ldlt.solve(J.transpose());
    Eigen::MatrixXd tem2 = J * tem;
    Eigen::Vector3d cond = cond_Matrix(tem2);
    if (cond(0) > c)
    {
        //std::cout << "cond big " << cond(0) <<  "\n";
        double alpha = cd;//std::max(cd, (cond(1) * cond(1) - c * cond(2) * cond(2)) / (c- 1));
        //std::cout <<  (cond(1) * cond(1) + alpha) / (cond(2) * cond(2) + alpha) <<  "\n";
        Eigen::MatrixXd tem3 = tem2.transpose() * tem2;
        Eigen::VectorXd d(tem2.cols());
        d.setOnes();
        tem3.diagonal() += alpha * d;
        return (tem3).ldlt().solve(tem2.transpose() * J) * ldlt.solve(X);
    }
    else
        return (tem2).ldlt().solve(J) * ldlt.solve(X);
    // return pInv(tem2) * J * ldlt.solve(X);
}

Eigen::MatrixXd A_x(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M)
{
    Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
    return (J * tem).inverse();
}

Eigen::MatrixXd A_x_inv(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M)
{
    return J * M.ldlt().solve(J.transpose());
}

Eigen::MatrixXd A_x_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& X)
{
    Eigen::MatrixXd tem = M.ldlt().solve(J.transpose());
    return (J * tem).ldlt().solve(X);
}

Eigen::VectorXd null_proj(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::VectorXd& v)
{
    return v - J_sharp_X(J, M, J * v);
}

Eigen::MatrixXd null_z(const Eigen::MatrixXd& J)
{
    int m = J.rows();
    int n = J.cols();
    int r = n - m;
    Eigen::Matrix6d Jm = J.leftCols(m); // m x m
    Eigen::MatrixXd Jr = J.rightCols(r); // m x r
    Eigen::MatrixXd I(r, r);
    I.setIdentity();
    Eigen::MatrixXd Z(n, r);
    Z.topRows(m) = -Jm.partialPivLu().solve(Jr);
    Z.bottomRows(r) = I;
    return Z;
}

Eigen::MatrixXd z_sharp(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M)
{
    return (Z.transpose() * M * Z).ldlt().solve(Z.transpose() * M);
}

Eigen::MatrixXd Mu_x_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& C, const Eigen::MatrixXd& X)
{
    return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * J_sharp_X(J, M, X);
}

Eigen::MatrixXd Mu_x(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& C)
{
    return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * J_sharp(J, M);
}

Eigen::MatrixXd d_null_z(const Eigen::MatrixXd& J, const Eigen::MatrixXd& dJ)
{
    int m = J.rows();
    int n = J.cols();
    int r = n - m;
    Eigen::Matrix6d dJm = dJ.leftCols(m); // m x m
    Eigen::MatrixXd dJr = dJ.rightCols(r); // m x r
    Eigen::Matrix6d Jm = J.leftCols(m); // m x m
    Eigen::MatrixXd Jr = J.rightCols(r); // m x r

    Eigen::MatrixXd I(r, r);
    I.setZero();

    Eigen::MatrixXd dZ(n, r);
    dZ.topRows(m) = Jm.partialPivLu().solve(dJm) * Jm.partialPivLu().solve(Jr) - Jm.partialPivLu().solve(dJr);
    dZ.bottomRows(r) = I;
    return dZ;
}

Eigen::MatrixXd d_z_sharp(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM)
{
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Z.transpose() * M * Z);
    //Eigen::MatrixXd tem1 = Z.transpose() * M * Z;
    Eigen::MatrixXd dtem1 = dZ.transpose() * M * Z + Z.transpose() * dM * Z + Z.transpose() * M * dZ;

    Eigen::MatrixXd tem2 = Z.transpose() * M;
    Eigen::MatrixXd dtem2 = dZ.transpose() * M  + Z.transpose() * dM;

    return -ldlt.solve(dtem1) * ldlt.solve(tem2) + ldlt.solve(dtem2);

}

Eigen::MatrixXd A_v(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M)
{
    return Z.transpose() * M * Z;
}

Eigen::MatrixXd Mu_v(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& C)
{
    return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * Z;
}

Eigen::MatrixXd Mu_xv(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dJ, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& C)
{
    return (J_sharp_T_X(J, M, C) - A_x_X(J, M, dJ)) * Z;
}

Eigen::MatrixXd Mu_vx(const Eigen::MatrixXd& J, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& C)
{
    return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * J_sharp(J, M);
}

Eigen::MatrixXd Mu_vx_X(const Eigen::MatrixXd& J, const Eigen::MatrixXd& Z, const Eigen::MatrixXd& M, const Eigen::MatrixXd& dZ, const Eigen::MatrixXd& dM, const Eigen::MatrixXd& C, const Eigen::MatrixXd& X)
{
    return (Z.transpose() * C - A_v(Z, M) * d_z_sharp(Z, M, dZ, dM)) * J_sharp_X(J, M, X);
}

void cal_motion_error(const Eigen::Matrix4d& T, const Eigen::Matrix4d& T_d,
    const Eigen::Vector3d& v, const Eigen::Vector3d& w,
    const Eigen::Vector3d& v_d, const Eigen::Vector3d& w_d,
    const Eigen::Vector3d& a_d, const Eigen::Vector3d& alpha_d,
    Eigen::Vector6d& xe, Eigen::Vector6d& dxe, Eigen::Vector6d& ddx_d)
{

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix3d Rd = T_d.block<3, 3>(0, 0);
    xe.head<3>() = logR(R.transpose() * Rd);
    xe.tail<3>() = T_d.block<3, 1>(0, 3) - T.block<3, 1>(0, 3);
    dxe.head<3>() = R.transpose() * (w_d - w);
    dxe.tail<3>() = v_d - v;
    ddx_d.head<3>() = R.transpose() * (alpha_d - w.cross(w_d));
    ddx_d.tail<3>() = a_d;
}

void swapOrder(double* buf, int n)
{
    int offset = n / 2;
    for (int i = 0; i < offset; i++)
    {
        double tem = buf[i];
        buf[i] = buf[i + offset];
        buf[i + offset] = tem;
    }
}


void bodyTwist2SpatialTwist(const double _T[16], const double _Vb[6], const double _dVb[6], double _Vs[6], double _dVs[6])
{
    Eigen::Map<const Eigen::Matrix4d> T(_T);
    Eigen::Map<const Eigen::Vector6d> Vb(_Vb);
    
    Eigen::Map<Eigen::Vector6d> Vs(_Vs);
   
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Matrix4d Tsb = Eigen::Matrix4d::Identity();
    Tsb.block(0, 0, 3, 3) = R;
    Vs = adjoint_T(Tsb) * Vb;
    if (_dVb && _dVs)
    {
        Eigen::Map<const Eigen::Vector6d> dVb(_dVb);
        Eigen::Map<Eigen::Vector6d> dVs(_dVs);
        dVs.topRows(3) = R * dVb.topRows(3);
        Eigen::Vector3d w = Vs.topRows(3);
        Eigen::Vector3d v = Vs.bottomRows(3);
        dVs.bottomRows(3) = w.cross(v) + R * dVb.bottomRows(3);
    }
    
}

void spatialTwist2BodyTwist(const double _T[16], const double _Vs[6], const double _dVs[16], double _Vb[6], double _dVb[6])
{
    Eigen::Map<const Eigen::Matrix4d> T(_T);
    Eigen::Map<Eigen::Vector6d> Vb(_Vb);
    Eigen::Map<const Eigen::Vector6d> Vs(_Vs);
    Eigen::Matrix3d R = T.block(0, 0, 3, 3).transpose();
    Eigen::Matrix4d Tbs = Eigen::Matrix4d::Identity();
    Tbs.block(0, 0, 3, 3) = R;
    Vb = adjoint_T(Tbs) * Vs;
    if (_dVs && _dVb)
    {
        Eigen::Map<Eigen::Vector6d> dVb(_dVb);
        Eigen::Map<const Eigen::Vector6d> dVs(_dVs);
        Eigen::Vector3d w = Vs.topRows(3);
        Eigen::Vector3d v = Vs.bottomRows(3);
        dVb.topRows(3) = R * dVs.topRows(3);
        dVb.bottomRows(3) = R * (dVs.bottomRows(3) - w.cross(v));
    }
    
}
// spatial twist
Eigen::Vector6d twist_estimate(const Eigen::Matrix4d &Td, const Eigen::Matrix4d &Td_pre, double dt)
{
    Eigen::Matrix3d Rd = Td.block(0, 0, 3, 3);
    Eigen::Matrix3d RdT = Rd.transpose();
    Eigen::Matrix3d Rd_pre = Td_pre.block(0, 0, 3, 3);
    Eigen::Matrix3d dR = (Rd - Rd_pre) / dt;
    Eigen::Matrix3d W = dR * RdT;
    Eigen::Vector3d w(W(2, 1), W(0, 2), W(1, 0));
    Eigen::Vector3d v = (Td.block(0, 3, 3, 1) - Td_pre.block(0, 3, 3, 1)) / dt;
    Eigen::Vector6d Vd;
    Vd << w, v;
    return Vd;
}



int isTriggered(int signal)
{
    static int flag = 0;
    if (flag == 0 && signal)
    {
        flag = 1;
        return 1;
    }
    else if (flag == 1 && !signal)
    {
        flag = 0;
        return 1;
    }
    return 0;
}


 void preciseSleep(double seconds)
{
    using namespace std;
    using namespace std::chrono;

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate)
    {
        auto start = high_resolution_clock::now();
        std::this_thread::sleep_for(milliseconds(1));
        auto end = high_resolution_clock::now();

        double observed = duration<double>(end - start).count();
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / static_cast<double>(count);
        m2 += delta * (observed - mean);
        double stddev = sqrt(m2 / (static_cast<double>(count - 1)));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while (duration<double>((high_resolution_clock::now() - start)).count() < seconds)
        ;
}

#if defined(__linux__) || defined(__APPLE__)
  static timespec timepointToTimespec(std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> tp)
  {
    auto secs = std::chrono::time_point_cast<std::chrono::seconds>(tp);
    auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp) -
              std::chrono::time_point_cast<std::chrono::nanoseconds>(secs);

    return timespec{secs.time_since_epoch().count(), ns.count()};
  }
#endif

 void waitPeriod(const std::chrono::steady_clock::time_point& t_cycle_start, double dt)
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
        preciseSleep(dt - t_app_duration.count());
    }
#elif defined(__APPLE__)
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_app_duration.count()));
    }
#else
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
        auto cycle_time_in_ms = static_cast<int64_t>(dt * 1000);
        auto t_cycle_ideal_end = t_cycle_start + milliseconds(cycle_time_in_ms);
        auto t_cycle_end_with_slack = t_cycle_ideal_end - (microseconds(SLACK_TIME_IN_MICROS) +
            t_app_duration);

        struct timespec tv_cycle_end_with_slack {}, tv_cycle_end{}, curr{};
        tv_cycle_end = timepointToTimespec(time_point_cast<nanoseconds>(t_cycle_ideal_end));
        tv_cycle_end_with_slack = timepointToTimespec(time_point_cast<nanoseconds>(t_cycle_end_with_slack));
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tv_cycle_end_with_slack, NULL);

        clock_gettime(CLOCK_MONOTONIC, &curr);
        for (; curr.tv_nsec < tv_cycle_end.tv_nsec; clock_gettime(CLOCK_MONOTONIC, &curr))
            ;
    }
#endif
}
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#include <windows.h>
#endif

 bool setRealtimePriority(int priority)
 {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
     auto get_last_windows_error = []() -> std::string {
         DWORD error_id = GetLastError();
         LPSTR buffer = nullptr;
         size_t size = FormatMessageA(
             FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
             nullptr, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)(&buffer), 0, nullptr);
         return std::string(buffer, size);
     };

     if (priority == 0)
     {
         // priority not set explicitly by user, assume that max. priority is desired.
         priority = THREAD_PRIORITY_TIME_CRITICAL;
     }

     if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
     {
         std::cerr << "unable to set priority for the process: " << get_last_windows_error() << std::endl;
         return false;
     }

     if (!SetThreadPriority(GetCurrentThread(), priority))
     {
         std::cerr << "unable to set priority for the thread: " << get_last_windows_error() << std::endl;
         return false;
     }
     return true;
#else
     if (priority < 0)
     {
         std::cout << "realtime priority less than 0 specified, realtime priority will not be set on purpose!" <<
             std::endl;
         return false;
     }

     if (priority == 0)
     {
         // priority not set explicitly by user, assume that a fair max. priority is desired.
         const int thread_priority = sched_get_priority_max(SCHED_FIFO);
         if (thread_priority == -1)
         {
             std::cerr << "unable to get maximum possible thread priority: " << strerror(errno) <<
                 std::endl;
             return false;
         }
         // the priority is capped at 90, since any higher value would make the OS too unstable.
         priority = std::min(90, std::max(0, thread_priority));
     }

     sched_param thread_param{};
     thread_param.sched_priority = priority;
     if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0)
     {
         std::cerr << "unable to set realtime scheduling: " << strerror(errno) << std::endl;
         return false;
     }
     return true;
#endif
 }