#include <mutex>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "iiwa_cpp_controller/iiwa_dynamics.hpp"

class IiwaImpedanceControllerCpp : public rclcpp::Node {
public:
  IiwaImpedanceControllerCpp()
  : Node("iiwa_impedance_controller_cpp")
  {
    n_ = 7;

    // -------- parameters --------
    this->declare_parameter<double>("k", 2000.0);
    this->declare_parameter<double>("d", 90.0);
    this->declare_parameter<double>("tau_lim", 200.0);
    this->declare_parameter<double>("ctrl_hz", 200.0);

    this->declare_parameter<int>("ma_window", 50);
    this->declare_parameter<double>("max_delta_tau", 1.0);

    this->declare_parameter<bool>("use_gravity_comp", true);
    this->declare_parameter<double>("gravity_comp_scale", 1.0);

    this->declare_parameter<bool>("use_coriolis", false);
    this->declare_parameter<bool>("use_friction", false);

    k_ = this->get_parameter("k").as_double();
    d_ = this->get_parameter("d").as_double();
    tau_lim_ = std::abs(this->get_parameter("tau_lim").as_double());
    ctrl_hz_ = this->get_parameter("ctrl_hz").as_double();

    ma_window_ = this->get_parameter("ma_window").as_int();
    if (ma_window_ < 1) ma_window_ = 1;
    max_delta_tau_ = std::abs(this->get_parameter("max_delta_tau").as_double());

    use_gravity_comp_ = this->get_parameter("use_gravity_comp").as_bool();
    gravity_comp_scale_ = this->get_parameter("gravity_comp_scale").as_double();

    use_coriolis_ = this->get_parameter("use_coriolis").as_bool();
    use_friction_ = this->get_parameter("use_friction").as_bool();

    // -------- buffers --------
    tau_buf_.resize(n_);
    for (int i = 0; i < n_; i++) {
      tau_buf_[i] = std::deque<double>();
      tau_buf_[i].clear();
    }
    tau_last_pub_.assign(n_, 0.0);
    have_tau_last_ = false;

    q_.setZero();
    dq_.setZero();
    q_des_.setZero();
    have_js_ = false;
    have_qdes_ = false;

    // -------- ROS I/O --------
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/iiwa7/joint_states", 10,
      std::bind(&IiwaImpedanceControllerCpp::on_js, this, std::placeholders::_1));

    sub_qdes_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/iiwa7/joint_desired", 10,
      std::bind(&IiwaImpedanceControllerCpp::on_q_des, this, std::placeholders::_1));

    pub_tau_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/iiwa7/joint_torques", 10);

    // timer
    auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, ctrl_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&IiwaImpedanceControllerCpp::control, this));

    RCLCPP_INFO(this->get_logger(),
      "iiwa_impedance_controller_cpp started. k=%.3f d=%.3f tau_lim=%.3f ctrl_hz=%.3f",
      k_, d_, tau_lim_, ctrl_hz_);
    RCLCPP_INFO(this->get_logger(),
      "filter: ma_window=%d max_delta_tau=%.3f use_gravity=%d scale=%.3f use_coriolis=%d use_friction=%d",
      ma_window_, max_delta_tau_, (int)use_gravity_comp_, gravity_comp_scale_,
      (int)use_coriolis_, (int)use_friction_);
  }

private:
  void on_js(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < (size_t)n_ || msg->velocity.size() < (size_t)n_) return;

    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < n_; i++) {
      q_(i)  = msg->position[i];
      dq_(i) = msg->velocity[i];
    }
    have_js_ = true;

    // 第一次拿到状态时，把 q_des 锁到当前 q，避免突然跳变
    if (!have_qdes_) {
      q_des_ = q_;
    }
  }

  void on_q_des(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < (size_t)n_) return;

    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < n_; i++) {
      q_des_(i) = msg->data[i];
    }
    have_qdes_ = true;
  }

  static double clip(double x, double lim) {
    if (x > lim) return lim;
    if (x < -lim) return -lim;
    return x;
  }

  double moving_average(int i, double x) {
    auto &b = tau_buf_[i];
    b.push_back(x);
    if ((int)b.size() > ma_window_) b.pop_front();
    double s = 0.0;
    for (double v : b) s += v;
    return s / std::max(1, (int)b.size());
  }

  std::vector<double> rate_limit(const std::vector<double>& target, const std::vector<double>& prev) {
    std::vector<double> out(n_, 0.0);
    if (max_delta_tau_ <= 0.0) return target;
    for (int i = 0; i < n_; i++) {
      double dt = target[i] - prev[i];
      if (dt >  max_delta_tau_) out[i] = prev[i] + max_delta_tau_;
      else if (dt < -max_delta_tau_) out[i] = prev[i] - max_delta_tau_;
      else out[i] = target[i];
    }
    return out;
  }

  void control() {
    Vec7 q, dq, q_des;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_js_) return;
      q = q_;
      dq = dq_;
      q_des = q_des_;
    }

    // ---- joint-space impedance (PD) ----
    Vec7 tau_pd;
    for (int i = 0; i < n_; i++) {
      const double e  = q_des(i) - q(i);
      const double ed = 0.0 - dq(i);
      tau_pd(i) = k_ * e + d_ * ed;
    }

    // ---- dynamics terms from YOUR C++ model ----
    Vec7 tau_g = Vec7::Zero();
    if (use_gravity_comp_ && std::abs(gravity_comp_scale_) > 0.0) {
      tau_g = gravity_comp_scale_ * GravityVector(q);  // TODO: 替换为你的实现
    }

    Vec7 tau_c = Vec7::Zero();
    if (use_coriolis_) {
      tau_c = CoriolisMatrix(q, dq) * dq;              // TODO: 替换为你的实现
    }

    Vec7 tau_f = Vec7::Zero();
    if (use_friction_) {
      tau_f = FrictionTorque(dq);                      // TODO: 替换为你的实现
    }

    Vec7 tau = tau_pd + tau_g + tau_c + tau_f;

    // ---- clip + MA + rate limit ----
    std::vector<double> tau_raw(n_, 0.0), tau_ma(n_, 0.0);
    for (int i = 0; i < n_; i++) {
      tau_raw[i] = clip((double)tau(i), tau_lim_);
      tau_ma[i]  = moving_average(i, tau_raw[i]);
    }

    std::vector<double> tau_out;
    if (!have_tau_last_) {
      tau_out = tau_ma;
      have_tau_last_ = true;
    } else {
      tau_out = rate_limit(tau_ma, tau_last_pub_);
    }
    tau_last_pub_ = tau_out;

    std_msgs::msg::Float64MultiArray out;
    out.data = tau_out;
    pub_tau_->publish(out);

    // 低频打印（避免 200Hz 刷屏）
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "tau0=%.3f q0=%.3f qd0=%.3f qdes0=%.3f",
      tau_out[0], q(0), dq(0), q_des(0)
    );
  }

private:
  int n_;
  double k_, d_, tau_lim_, ctrl_hz_;
  int ma_window_;
  double max_delta_tau_;
  bool use_gravity_comp_;
  double gravity_comp_scale_;
  bool use_coriolis_;
  bool use_friction_;

  std::mutex mtx_;
  bool have_js_;
  bool have_qdes_;
  Vec7 q_, dq_, q_des_;

  std::vector<std::deque<double>> tau_buf_;
  std::vector<double> tau_last_pub_;
  bool have_tau_last_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_qdes_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_tau_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IiwaImpedanceControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
