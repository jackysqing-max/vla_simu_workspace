#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <json/json.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "KukaKinematics.h"

namespace {

constexpr int kNumJoints = 7;

struct TargetPose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_offset{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_offset_rpy{Eigen::Vector3d::Zero()};
};

std::string to_upper(std::string value) {
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return value;
}

double clamp_scalar(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

Eigen::Quaterniond quat_from_rpy_xyz(const Eigen::Vector3d &rpy) {
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(yaw * pitch * roll).normalized();
}

std::array<double, kNumJoints> clamp_joint_step(
  const std::array<double, kNumJoints> &q_now,
  const std::array<double, kNumJoints> &q_des,
  double max_step)
{
  std::array<double, kNumJoints> out{};
  const double limit = std::abs(max_step);
  for (int i = 0; i < kNumJoints; ++i) {
    const double delta = clamp_scalar(q_des[i] - q_now[i], -limit, limit);
    out[i] = q_now[i] + delta;
  }
  return out;
}

}  // namespace

class GestureEePoseNodeCpp : public rclcpp::Node {
public:
  GestureEePoseNodeCpp()
  : Node("gesture_ee_pose_node")
  {
    this->declare_parameter<std::string>("command_topic", "/touchless/command_json");
    this->declare_parameter<std::string>("joint_states_topic", "/iiwa7/joint_states");
    this->declare_parameter<std::string>("joint_desired_topic", "/iiwa7/joint_desired");
    this->declare_parameter<std::string>("control_mode_topic", "/iiwa7/control_mode");
    this->declare_parameter<std::string>("target_pose_topic", "/touchless/ee_target_pose");
    this->declare_parameter<std::string>("init_done_topic", "/iiwa7/init_done");
    this->declare_parameter<bool>("wait_for_init_done", true);
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<int>("control_mode_value", 1);
    this->declare_parameter<double>("max_joint_step_rad", 0.02);
    this->declare_parameter<double>("pan_y_m_per_px", 0.0015);
    this->declare_parameter<double>("pan_z_m_per_px", 0.0015);
    this->declare_parameter<double>("zoom_x_m_per_px", 0.0015);
    this->declare_parameter<double>("rotate_pitch_rad_per_px", 0.004);
    this->declare_parameter<double>("rotate_yaw_rad_per_px", 0.004);
    this->declare_parameter<std::vector<double>>(
      "max_position_offset_xyz_m", std::vector<double>{0.30, 0.30, 0.30});
    this->declare_parameter<std::vector<double>>(
      "max_orientation_offset_rpy_rad", std::vector<double>{0.90, 0.90, 0.90});
    this->declare_parameter<std::string>("orientation_reference_frame", "world");
    this->declare_parameter<bool>("reset_on_off_mode", false);

    command_topic_ = this->get_parameter("command_topic").as_string();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    joint_desired_topic_ = this->get_parameter("joint_desired_topic").as_string();
    control_mode_topic_ = this->get_parameter("control_mode_topic").as_string();
    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
    init_done_topic_ = this->get_parameter("init_done_topic").as_string();
    wait_for_init_done_ = this->get_parameter("wait_for_init_done").as_bool();
    publish_hz_ = this->get_parameter("publish_hz").as_double();
    control_mode_value_ = this->get_parameter("control_mode_value").as_int();
    max_joint_step_rad_ = this->get_parameter("max_joint_step_rad").as_double();
    reset_on_off_mode_ = this->get_parameter("reset_on_off_mode").as_bool();

    pan_y_m_per_px_ = this->get_parameter("pan_y_m_per_px").as_double();
    pan_z_m_per_px_ = this->get_parameter("pan_z_m_per_px").as_double();
    zoom_x_m_per_px_ = this->get_parameter("zoom_x_m_per_px").as_double();
    rotate_pitch_rad_per_px_ = this->get_parameter("rotate_pitch_rad_per_px").as_double();
    rotate_yaw_rad_per_px_ = this->get_parameter("rotate_yaw_rad_per_px").as_double();
    max_position_offset_xyz_m_ =
      read_triple_parameter("max_position_offset_xyz_m");
    max_orientation_offset_rpy_rad_ =
      read_triple_parameter("max_orientation_offset_rpy_rad");
    orientation_reference_frame_ =
      to_upper(this->get_parameter("orientation_reference_frame").as_string());

    init_done_ = !wait_for_init_done_;

    auto init_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    sub_init_done_ = this->create_subscription<std_msgs::msg::Bool>(
      init_done_topic_,
      init_qos,
      std::bind(&GestureEePoseNodeCpp::on_init_done, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      10,
      std::bind(&GestureEePoseNodeCpp::on_joint_state, this, std::placeholders::_1));
    sub_command_ = this->create_subscription<std_msgs::msg::String>(
      command_topic_,
      10,
      std::bind(&GestureEePoseNodeCpp::on_command, this, std::placeholders::_1));

    pub_control_mode_ = this->create_publisher<std_msgs::msg::Int8>(control_mode_topic_, 10);
    pub_joint_desired_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      joint_desired_topic_, 10);
    pub_target_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GestureEePoseNodeCpp::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "gesture_ee_pose_node_cpp started. Mapping: PAN -> EE Y/Z, ZOOM -> EE X, "
      "ROTATE -> yaw/pitch using C++ FK/IK.");
  }

private:
  std::array<double, 3> read_triple_parameter(const std::string &name) {
    const auto values = this->get_parameter(name).as_double_array();
    if (values.size() != 3) {
      std::ostringstream oss;
      oss << name << " must contain exactly 3 values";
      throw std::runtime_error(oss.str());
    }
    return {values[0], values[1], values[2]};
  }

  void on_init_done(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    init_done_ = msg->data;
    try_initialize_locked();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < kNumJoints) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < kNumJoints; ++i) {
      q_now_[i] = static_cast<double>(msg->position[i]);
    }
    have_q_now_ = true;
    try_initialize_locked();
  }

  void on_command(const std_msgs::msg::String::SharedPtr msg) {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errors;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    const bool ok = reader->parse(
      msg->data.data(),
      msg->data.data() + msg->data.size(),
      &root,
      &errors);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Ignoring malformed gesture command JSON");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!mapper_initialized_) {
      return;
    }

    const std::string mode = to_upper(root.get("mode", "").asString());
    if (mode == "OFF" && reset_on_off_mode_) {
      position_offset_.setZero();
      orientation_offset_rpy_.setZero();
      latest_target_ = build_target_locked();
      have_target_ = true;
      return;
    }

    const std::string action = to_upper(root.get("action", "").asString());
    if (action == "PAN") {
      apply_pan_locked(root.get("dx", 0.0).asDouble(), root.get("dy", 0.0).asDouble());
    } else if (action == "ZOOM") {
      apply_zoom_locked(root.get("delta_pinch", 0.0).asDouble());
    } else if (action == "ROTATE") {
      apply_rotate_locked(root.get("dx", 0.0).asDouble(), root.get("dy", 0.0).asDouble());
    }

    latest_target_ = build_target_locked();
    have_target_ = true;
  }

  void on_timer() {
    TargetPose target;
    std::array<double, kNumJoints> q_now{};
    std::array<double, kNumJoints> q_seed{};

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_q_now_ || !have_q_seed_ || !have_target_) {
        return;
      }
      target = latest_target_;
      q_now = q_now_;
      q_seed = q_seed_;
    }

    publish_target_pose(target);

    std::array<double, kNumJoints> q_ik{};
    if (!solve_ik(q_seed, target, q_ik)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "C++ IK failed for current target pose, keeping previous joint target.");
      return;
    }

    const auto q_des = clamp_joint_step(q_now, q_ik, max_joint_step_rad_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      q_seed_ = q_des;
    }

    publish_control_mode();
    publish_joint_target(q_des);
  }

  void try_initialize_locked() {
    if (!init_done_ || mapper_initialized_ || !have_q_now_) {
      return;
    }

    const Eigen::Matrix4d transform = forward_kinematics(q_now_);
    initial_position_ = transform.block<3, 1>(0, 3);
    initial_orientation_ = Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
    position_offset_.setZero();
    orientation_offset_rpy_.setZero();
    latest_target_ = build_target_locked();
    have_target_ = true;
    q_seed_ = q_now_;
    have_q_seed_ = true;
    mapper_initialized_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "[INIT] Locked EE pose p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
      initial_position_.x(), initial_position_.y(), initial_position_.z(),
      latest_target_.orientation.x(), latest_target_.orientation.y(),
      latest_target_.orientation.z(), latest_target_.orientation.w());
  }

  Eigen::Matrix4d forward_kinematics(const std::array<double, kNumJoints> &q) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    kinematics_.forwardKinematics(q.data(), transform.data(), kNumJoints);
    return transform;
  }

  bool solve_ik(
    const std::array<double, kNumJoints> &q_seed,
    const TargetPose &target,
    std::array<double, kNumJoints> &q_out)
  {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = target.orientation.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = target.position;

    const double kesai = kinematics_.cal_kesai(q_seed.data());
    if (kinematics_.inverseKinematics(kesai, transform.data(), q_seed.data(), q_out.data()) != 0) {
      return true;
    }

    int cfg[3] = {1, -1, 1};
    return kinematics_.inverseKinematics(cfg, transform.data(), q_out.data()) != 0;
  }

  TargetPose build_target_locked() const {
    TargetPose target;
    target.position = initial_position_ + position_offset_;
    target.position_offset = position_offset_;
    target.orientation_offset_rpy = orientation_offset_rpy_;

    const Eigen::Quaterniond delta = quat_from_rpy_xyz(orientation_offset_rpy_);
    if (orientation_reference_frame_ == "TOOL") {
      target.orientation = (initial_orientation_ * delta).normalized();
    } else {
      target.orientation = (delta * initial_orientation_).normalized();
    }
    return target;
  }

  void apply_pan_locked(double dx_px, double dy_px) {
    position_offset_.y() += dx_px * pan_y_m_per_px_;
    position_offset_.z() += -dy_px * pan_z_m_per_px_;
    clamp_position_offset_locked();
  }

  void apply_zoom_locked(double delta_pinch_px) {
    position_offset_.x() += delta_pinch_px * zoom_x_m_per_px_;
    clamp_position_offset_locked();
  }

  void apply_rotate_locked(double dx_px, double dy_px) {
    orientation_offset_rpy_.y() += -dy_px * rotate_pitch_rad_per_px_;
    orientation_offset_rpy_.z() += dx_px * rotate_yaw_rad_per_px_;
    clamp_orientation_offset_locked();
  }

  void clamp_position_offset_locked() {
    for (int i = 0; i < 3; ++i) {
      const double limit = std::abs(max_position_offset_xyz_m_[i]);
      position_offset_[i] = clamp_scalar(position_offset_[i], -limit, limit);
    }
  }

  void clamp_orientation_offset_locked() {
    for (int i = 0; i < 3; ++i) {
      const double limit = std::abs(max_orientation_offset_rpy_rad_[i]);
      orientation_offset_rpy_[i] = clamp_scalar(orientation_offset_rpy_[i], -limit, limit);
    }
  }

  void publish_control_mode() {
    std_msgs::msg::Int8 msg;
    msg.data = static_cast<int8_t>(control_mode_value_);
    pub_control_mode_->publish(msg);
  }

  void publish_joint_target(const std::array<double, kNumJoints> &q_des) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(q_des.begin(), q_des.end());
    pub_joint_desired_->publish(msg);
  }

  void publish_target_pose(const TargetPose &target) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.pose.position.x = target.position.x();
    msg.pose.position.y = target.position.y();
    msg.pose.position.z = target.position.z();
    msg.pose.orientation.x = target.orientation.x();
    msg.pose.orientation.y = target.orientation.y();
    msg.pose.orientation.z = target.orientation.z();
    msg.pose.orientation.w = target.orientation.w();
    pub_target_pose_->publish(msg);
  }

private:
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string joint_desired_topic_;
  std::string control_mode_topic_;
  std::string target_pose_topic_;
  std::string init_done_topic_;

  bool wait_for_init_done_{true};
  double publish_hz_{50.0};
  int control_mode_value_{1};
  double max_joint_step_rad_{0.02};
  bool reset_on_off_mode_{false};

  double pan_y_m_per_px_{0.0015};
  double pan_z_m_per_px_{0.0015};
  double zoom_x_m_per_px_{0.0015};
  double rotate_pitch_rad_per_px_{0.004};
  double rotate_yaw_rad_per_px_{0.004};
  std::array<double, 3> max_position_offset_xyz_m_{};
  std::array<double, 3> max_orientation_offset_rpy_rad_{};
  std::string orientation_reference_frame_{"WORLD"};

  std::mutex mutex_;
  bool init_done_{false};
  bool have_q_now_{false};
  bool have_q_seed_{false};
  bool mapper_initialized_{false};
  bool have_target_{false};

  std::array<double, kNumJoints> q_now_{};
  std::array<double, kNumJoints> q_seed_{};

  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond initial_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_offset_rpy_{Eigen::Vector3d::Zero()};
  TargetPose latest_target_{};

  KukaKinematics kinematics_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_init_done_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_command_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_control_mode_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_desired_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GestureEePoseNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
