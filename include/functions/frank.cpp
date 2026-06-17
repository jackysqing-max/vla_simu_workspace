#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <external_sensing.h>
#include <iostream>
#include <iterator>
#include <robot.h>
#include <utility.h>
#include "examples_common.h"
#include <DataComm.h>
#include <chrono>
#include <jacobian_matrix.h>
#include <OnlineTrajPlanner.h>
#include <MovingFilter.h>
#include "m_c_g_utility.h"
#include "DO_controller.h"
#include <derivative_jacobian_matrix.h>
#include "decoupleRCM.h"

using namespace std::chrono;

template <class T, size_t N>
std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
{
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}

void fill_robot_data(RobotData &robotData, const franka::RobotState &state, const franka::Model &model)
{
    std::array<double, 16> T = model.pose(franka::Frame::kEndEffector, state);
    Eigen::Map<Eigen::Matrix4d> poseT(T.data());
    std::vector<double> pose = T2pose(poseT);
    pose.push_back(0);
    for (int i = 0; i < 7; i++)
    {
        robotData.q[0][i] = (float)state.q[i];
        robotData.q[1][i] = (float)state.dq[i];
        robotData.q[2][i] = (float)state.tau_J[i];
        robotData.q[3][i] = (float)pose[i];
    }
}

Eigen::Vector7d saturateTorque(const Vectornd tau_d_calculated, const Vectornd tau_J_d)
{
    Vectornd tau_d_saturated;
    for (int i = 0; i < n; i++)
    {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, 1.0), -1.0);
    }
    return tau_d_saturated;
}

Eigen::Vector7d limitTorque(const Vectornd tau_d_calculated, double limit)
{
    if (limit < 0)
    {
        limit = -limit;
    }

    Vectornd tau_d_limited;
    for (int i = 0; i < n; i++)
    {
        tau_d_limited[i] = std::max(std::min(tau_d_calculated(i), limit), -limit);
    }
    return tau_d_limited;
}

Eigen::Matrix7d CoriolisMatrix(const Eigen::Vector7d &q, const Eigen::Vector7d &dq);
Eigen::Matrix7d MassMatrix(const Eigen::Vector7d &q);

void *frank(void *)
{
    // Robot myrobot = loadRobot("/home/mrsnlab/robot_control/franka.json");
    Robot myrobot = loadRobot("/home/luo/MyRos/PreData/panda_correct.json");
    // Robot myrobot = loadRobot("franka.json");
    RobotData robotData;
    try
    {
        franka::Robot robot("192.168.1.100");
        robot.automaticErrorRecovery();
        setDefaultBehavior(robot);
        franka::Model model(robot.loadModel());

        int cnt = 0;
        auto t0 = steady_clock::now();
        robot.read([&model, &cnt, &robotData, &t0, &myrobot](const franka::RobotState &state) -> bool
                   {
                       cnt++;
                       // std::cout << state.q << std::endl;
                       /*auto t = steady_clock::now();
                       robotData.t = std::chrono::duration<double>(t - t0).count();
                       fill_robot_data(robotData, state, model);
                       DataComm::getInstance()->sendRobotStatus(robotData);*/
                       std::array<double, 42> J_b = model.bodyJacobian(franka::Frame::kEndEffector, state);
                       std::array<double, 42> J_s = model.zeroJacobian(franka::Frame::kEndEffector, state);
                    //    std::array<double, 42> J_b = model.bodyJacobian(franka::Frame::kJoint7, state);
                    //    std::array<double, 42> J_s = model.zeroJacobian(franka::Frame::kJoint7, state);
                    //    std::cout << Eigen::Map<Eigen::Matrix6x7d>(J.data()) << "\n--------------\n";
                       Eigen::Map<Eigen::Matrix6x7d> Js(J_s.data());
                       Eigen::Map<Eigen::Matrix6x7d> bJ(J_b.data());
                       std::vector<double> q(state.q.begin(), state.q.end());
                    //    Eigen::Map<Eigen::Matrix4d> ME(myrobot.ME);
                    //    Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());
                       Eigen::Map<const Eigen::Vector7d> dq(state.dq.data());
                    //    Eigen::Matrix4d Tcp_d = ME*T;
                       Eigen::Matrix6x7d Jaco;
                       Eigen::Matrix4d T;
                       jacobian_matrix(&myrobot, q, coder_array_wrapper1(Jaco), T.data());
                    //    std::cout << Jaco << std::endl;
                       Eigen::MatrixXd M, C, Jb, dJb, dM;
                       Eigen::Matrix4d dTcp, Tcp;
                       Eigen::VectorXd g;
                       Eigen::Vector6d xe, dxe, ddx_d;
                       double lamda = 0.5;
                    //    myrobot.ME[14] = 0.107;
                        Eigen::Map<Eigen::Matrix4d> ME(myrobot.ME);
                        std::cout << ME << std::endl;
                       m_c_g_matrix(&myrobot, std::vector<double>(state.q.begin(), state.q.end()), std::vector<double>(state.dq.begin(), state.dq.end()),
                                   M, C, g, Jb, dJb, dM, dTcp, Tcp);
                        // std::cout << myrobot.ME[14] << std::endl;
                        ME(2,3) = 0;
                        Eigen::Matrix4d Tcp_d, Tb, Tbd, Tbcp, Td;
                        Eigen::Matrix6d adT, adTcp;
                        Eigen::Matrix6x7d J1, J2;
                        adT = adjoint_T(T);
                        adTcp = adjoint_T(ME);
                        Matrix6d Th = Matrix6d::Identity();
                        Th.block<3, 3>(3, 3) = T.block(0, 0, 3, 3);
                        Th.block<3, 3>(0, 0) = T.block(0, 0, 3, 3);
                        J1 = Th*Jb;
                        J2 = adTcp*J1;
                        Eigen::Matrix6x8d J_ext = J_external(J1, J2, lamda, T.block<3,1>(0,3), Tcp.block<3,1>(0,3));
                       std::cout << J1 << "\n\n"; 
                       std::cout << Js << "\n\n";
                       std::cout << Tcp - T << "\n\n";
                    //    std::cout << T << "\n\n";
                    //    std::cout << ((T*ME).block<3,1>(0,3) - T.block<3,1>(0,3)).norm() << "\n\n";
                    //    std::cout << J_ext.bottomRows(3) << "\n\n";

                       return false; });
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }

    return nullptr;
}

void *frank_move(void *)
{
    // RobotData robotData;
    // Robot myrobot = loadRobot("franka.json");
    Robot myrobot = loadRobot("/home/mrsnlab/robot_control/franka.json");
    try
    {
        RobotData robotData{0};
        franka::Robot robot("192.168.1.100");
        robot.automaticErrorRecovery();
        robot.setCollisionBehavior(
            {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        franka::Model model(robot.loadModel());
        std::array<double, 16> init_pose;
        std::array<double, 16> Tc = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        std::array<double, 7> Tau_c_array = {0, 0, 0, 0, 0, 0, 0};
        CartesianTrajPlanner planner(1.7, 9, 2.5, 17);
        Eigen::Matrix4d Te;
        double t = 0, t_offset = 0;
        MovingFilter<double> qdFilter(7, 10);
        Eigen::Vector7d dq;

        robot.control(
            [&robotData, &myrobot, &Tc, &t, &Tau_c_array, &qdFilter, &dq, &model](const franka::RobotState &state, franka::Duration period) -> franka::Torques
            {
                // qdFilter.filtering(state.dq.data(), dq.data());
                Eigen::Map<const Eigen::Vector7d> dq(state.dq.data());
                Eigen::Map<const Eigen::Vector7d> q(state.q.data());
                Eigen::Map<const Eigen::Vector7d> q_d(state.q_d.data());
                Eigen::Map<const Eigen::Vector7d> dq_d(state.dq_d.data());
                Eigen::Map<const Eigen::Vector7d> ddq_d(state.ddq_d.data());
                Eigen::Map<const Eigen::Matrix4d> Td(state.O_T_EE_d.data());
                // Eigen::Map<const Eigen::Matrix4d> Td(Tc.data());
                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
                // std::array<double, 42> jacobianb_array = model.bodyJacobian(franka::Frame::kEndEffector, state);
                Eigen::Vector7d qe = q_d - q;
                Eigen::Vector7d dqe = dq_d - dq;
                std::array<double, 49> M_array = model.mass(state);
                std::array<double, 7> C_array = model.coriolis(state);
                Eigen::Map<const Eigen::Matrix7d> M(M_array.data());
                Eigen::Map<const Eigen::Vector7d> C(C_array.data());
                Eigen::Map<Eigen::Vector7d> Tau_c(Tau_c_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(state.tau_J_d.data());
                // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobianb(jacobianb_array.data());
                Vector6d V;
                V = jacobian * dq;
                Eigen::Map<const Eigen::Matrix<double, 4, 4>> T(state.O_T_EE.data());
                Eigen::Matrix7d temp_m;
                Eigen::Vector7d temp_v;
                Eigen::Matrix6x7d Jb = Eigen::Matrix6x7d::Zero();
                Eigen::Matrix6x7d dJb = Eigen::Matrix6x7d::Zero();
                Eigen::Matrix4d Tcp = Eigen::Matrix4d::Zero();
                Eigen::Matrix4d dTcp = Eigen::Matrix4d::Zero();
                Eigen::Vector7d ddq_c = Eigen::Vector7d::Zero();
                Tau_c = M * ddq_d + (Eigen::Vector7d(50, 50, 50, 50, 50, 10, 10).array() * dqe.array()).matrix() +
                        (Eigen::Vector7d(2000, 2000, 2000, 2000, 2000, 1000, 1000).array() * qe.array()).matrix() + C;

                Matrix6d Kx = Matrix6d::Zero();
                Matrix6d Bx = Matrix6d::Zero();
                // Kx.diagonal() << 3000,3000,3000,300,300,300;
                // Bx.diagonal() << 110,110,110,10,10,10;
                Kx.diagonal() << 3000, 3000, 3000, 300, 300, 300;
                Bx.diagonal() << 100, 100, 100, 10, 10, 10;
                // Kx.diagonal() << 1000,1000,1000,100,100,100;
                // Bx.diagonal() << 50,50,50,10,10,10;
                Matrixnd Kn = 0 * Matrixnd::Identity();
                Matrixnd Bn = 0.4 * Matrixnd::Identity();

                coder::array<double, 2U> q_a, dq_a, Jb_a, dJb_a;
                double T_a[16], dT_a[16];
                q_a.set_size(7, 1);
                dq_a.set_size(7, 1);
                Jb_a.set_size(6, 7);
                dJb_a.set_size(6, 7);
                for (int i = 0; i < 7; i++)
                {
                    q_a[i] = q(i);
                    dq_a[i] = dq(i);
                }
                derivative_jacobian_matrix(&myrobot, q_a, dq_a, Jb_a, dJb_a, T_a, dT_a);
                for (int i = 0; i < 6; i++)
                {
                    for (int j = 0; j < 7; j++)
                    {
                        Jb(i, j) = Jb_a.at(i, j);
                        dJb(i, j) = dJb_a.at(i, j);
                    }
                }

                // std::cout << Jb <<"\n\n";
                m_c_g_matrix_E(&myrobot, q, dq, temp_m, temp_m, temp_v, Jb, dJb, temp_m, dTcp, Tcp);
                // std::cout << Jb <<"\n\n";
                Vector6d Vd, Vh;
                Matrix6d invAx;
                Matrix6x7d Jh, dJh;
                Matrix7x6d J_sharp;
                Matrix7d N;
                Matrix6d tt, dtt;
                Vector6d ddx_c;
                Matrix7d Mq;
                Matrix7d Cq;
                Vector6d dxe, xe;
                Mq = MassMatrix(q);
                Cq = CoriolisMatrix(q, dq);
                tt << Matrix3d::Identity(), Matrix3d::Zero(),
                    Matrix3d::Zero(), T.block(0, 0, 3, 3);
                Jh = tt * Jb;
                Vector3d w;
                w = V.tail(3);
                Matrix3d skw;
                skw << 0, -w(2), w(1),
                    w(2), 0, -w(0),
                    -w(1), w(0), 0;
                dtt << Matrix3d::Zero(), Matrix3d::Zero(),
                    Matrix3d::Zero(), skw * T.block(0, 0, 3, 3);
                dJh = dtt * Jb + tt * dJb;

                Matrix6x7d mid1, mid2, mid3, mid4;
                mid1 << Jb.bottomRows(3), Jb.topRows(3);
                mid2 << Jh.bottomRows(3), Jh.topRows(3);
                mid3 << dJb.bottomRows(3), dJb.topRows(3);
                mid4 << dJh.bottomRows(3), dJh.topRows(3);
                Jb = mid1;
                Jh = mid2;
                dJb = mid3;
                dJh = mid4;

                Vd = Jh * dq_d;
                Vh = Jh * dq;
                J_sharp = (M.inverse() * Jh.transpose()) * ((Jh * (M.inverse() * Jh.transpose())).inverse());
                N = Matrix7d::Identity() - J_sharp * Jh;
                Vector3d rxe;
                rxe = Vector3d::Zero();
                logR((T.block(0, 0, 3, 3).inverse()) * Td.block(0, 0, 3, 3), rxe);
                A_x_inv(Jh, M, invAx);
                Matrix6d mux;
                mux = Matrix6d::Zero();
                Mu_x(Jh, Mq, dJh, Cq, mux);
                xe << Td.block(0, 3, 3, 1) - T.block(0, 3, 3, 1), rxe;
                dxe = Vd - Vh;

                ddx_c = invAx * ((mux + Bx) * dxe + Kx * xe);
                // ddx_c = Bx*dxe + Kx*xe;
                ddq_c = J_sharp * (ddx_c - dJh * dq) + N * (ddq_d + M.inverse() * (Kn * qe + Bn * dqe));

                // DO
                static Vector7d td;
                td = Vector7d::Zero();
                Matrix7d Y;
                Y = 0.1 * Matrix7d::Identity();
                Vectornd P;
                P = Y * dq;
                Vector7d tao_d;
                tao_d = td + P;
                Matrix6d ttemp = Jh * (Mq.inverse() * Jh.transpose());
                tao_d = Jh.transpose() * (ttemp.inverse() * (Jh * (Mq.inverse() * tao_d)));

                Tau_c = M * ddq_c + C - tao_d;
                // Tau_c = C;
                // std::cout << Tau_c << "\n\n";
                td = Y * ((Mq.inverse()) * (C - Tau_c - P - td));
                td = td * 0.001;

                log2Channel(robotData, 0, xe.head(3).data(), 3);
                log2Channel(robotData, 1, xe.tail(3).data(), 3);
                log2Channel(robotData, 2, Tau_c.data(), 7);
                log2Channel(robotData, 3, tao_d.data(), 7);
                robotData.t = robotData.t + period.toSec();
                DataComm::getInstance()->sendRobotStatus(robotData);

                // Tau_c = C;

                Tau_c << saturateTorque(Tau_c, tau_J_d);

                return Tau_c_array;
            },
            [&init_pose, &t, &planner, &Tc, &t_offset, &Te](const franka::RobotState &state, franka::Duration period) -> franka::CartesianPose
            {
            Eigen::Vector6d V;
            Eigen::Vector6d dV;
            t += period.toSec();
            if (t == 0)
            {
                init_pose = state.O_T_EE_c; // must be c
                Eigen::Map<const Eigen::Matrix4d> Ts(init_pose.data());
                Te = Ts;
                Te(0, 3) += 0.2;
                std::cout << Te << "\n";
                if (planner.generateMotion(Ts.data(), Te.data(), 0.02, 0.1, 2))
                {
                    std::cout << "generated failed\n";
                    return franka::MotionFinished(init_pose);
                }
                Te = Ts;
                return init_pose;
            }
            else
            {
                if (planner.step(Tc.data(), V.data(), dV.data(), t - t_offset))
                {
                    return Tc;
                }
                else
                {
                
                    Eigen::Map<const Eigen::Matrix4d> Ts(state.O_T_EE_c.data());
                    if (planner.generateMotion(Ts.data(), Te.data(), 0.02, 0.1, 2))
                    {
                        std::cout << "generated failed\n";
                        return franka::MotionFinished(Tc);
                    }
                    else
                    {
                            Te = Ts;
                            t_offset = t;
                            return Tc;
                    }
                }
            } },

            false);
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }

    return nullptr;
}

void *frank_move2(void *)
{
    // Robot myrobot = loadRobot("franka.json");
    RobotData robotData;
    try
    {
        franka::Robot robot("192.168.1.100");
        robot.automaticErrorRecovery();
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 200, 200}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);
        RobotData robotData{0};
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        
        std::array<double, 16> initial_pose;
        double time = 0.0;
        robot.control([&time, &initial_pose, &robotData](const franka::RobotState &robot_state,
                                             franka::Duration period) -> franka::CartesianPose
                      {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      constexpr double kRadius = 0.01;
      double angle = M_PI * (1 - std::cos(M_PI / 8.0 * time));
    //   double delta_x = kRadius * std::sin(angle);
    //   double delta_y = kRadius * (std::cos(angle) - 1);
    double delta_x = kRadius * std::sin(angle);
    double delta_y = kRadius * std::cos(2*angle)/2;

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[13] += delta_y;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }

        log2Channel(robotData, 0, robot_state.q.data(), 7);
        log2Channel(robotData, 1, robot_state.dq.data(), 7);
        log2Channel(robotData, 2, &robot_state.m_load, 1);
        // log2Channel(robotData, 3, Tau_c.data(), 7);
        robotData.t = time;
        DataComm::getInstance()->sendRobotStatus(robotData);
        return new_pose; });
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }

    return nullptr;
}