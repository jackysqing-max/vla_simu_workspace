#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include "widget.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "robot_control_msgs/msg/robot_state.hpp"
#include <QTimer>
#include <deque>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <QLabel>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void pushMessage(double t, robot_control_msgs::msg::RobotState::SharedPtr msg);
    double getDataDuration();
    void clear();
    void log2file();
    void onTimer();
    void setJointDisplay();
    void setScaling(bool );
    void setWindowWidth();
    void setLineWidth();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void setLogging(bool checked);
protected:
    void closeEvent(QCloseEvent *event) override;
private:
    Ui::MainWindow *ui;
    Widget *widgets[4];
    QLabel *logging_message_;
    QLabel *keypoint_message_;
    unsigned int sampleCount = 50000;
    QList<QPointF> m_buffer[4][7];
    double time_width = 10;
    QTimer timer;
    std::vector<std::vector<double>> frames;
    std::vector<double> frames_time;
    bool isScaling;
    bool isLogging;
    std::shared_ptr<std::thread> tt;
    std::mutex mtx;
    rclcpp::Time start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_control_msgs::msg::RobotState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr prompt_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr keypoint_subscription_;
    bool first_time_;
    std::string current_prompt_;
    std::string keypoint_frame_id_;
    double keypoint_x_ = 0.0;
    double keypoint_y_ = 0.0;
    double keypoint_z_ = 0.0;
    bool have_keypoint_ = false;
    bool keypoint_valid_ = false;
    bool keypoint_stamp_is_zero_ = true;
    rclcpp::Time latest_keypoint_time_;
    double keypoint_timeout_sec_ = 1.0;
    rclcpp::Time last_keypoint_log_time_;
    bool have_keypoint_log_time_ = false;
};
#endif // MAINWINDOW_H
