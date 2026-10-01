#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtCharts/QValueAxis>
#include <QChart>
#include <QChartView>
#include <QMetaObject>
#include <QInputDialog>
#include <QLabel>
#include <QColor>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
constexpr int kPanelCount = 4;
constexpr int kJointCount = 7;
constexpr int kExpectedRobotStateSize = kPanelCount * kJointCount;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), first_time_(true)
{
    ui->setupUi(this);

    const QString titles[kPanelCount] = {
        "Joint Position",
        "Joint Velocity",
        "Joint Torque",
        "Desired Position",
    };

    ui->gridLayout->setContentsMargins(0, 0, 0, 0);
    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
    {
        widgets[panel_index] = new Widget(titles[panel_index], this);
        ui->gridLayout->addWidget(widgets[panel_index], panel_index / 2, panel_index % 2);
        for (int joint_index = 0; joint_index < kJointCount; joint_index++)
            m_buffer[panel_index][joint_index].reserve(sampleCount);
    }
    keypoint_widget_ = new Widget("Keypoint XYZ / m", this);
    ui->gridLayout->addWidget(keypoint_widget_, 2, 0, 1, 2);
    QLineSeries **keypoint_series = keypoint_widget_->getSeries();
    const QString keypoint_names[3] = {"x", "y", "z"};
    const QColor keypoint_colors[3] = {
        QColor(255, 90, 90),
        QColor(90, 220, 120),
        QColor(90, 170, 255),
    };
    for (int axis_index = 0; axis_index < 3; axis_index++)
    {
        keypoint_series[axis_index]->setName(keypoint_names[axis_index]);
        QPen pen = keypoint_series[axis_index]->pen();
        pen.setColor(keypoint_colors[axis_index]);
        pen.setWidthF(2.0);
        keypoint_series[axis_index]->setPen(pen);
        keypoint_buffer_[axis_index].reserve(sampleCount);
    }
    for (int axis_index = 3; axis_index < kJointCount; axis_index++)
        keypoint_series[axis_index]->setVisible(false);

    logging_message_ = new QLabel(this);
    keypoint_message_ = new QLabel(this);
    candidate_message_ = new QLabel(this);
    statusBar()->addPermanentWidget(logging_message_);
    statusBar()->addPermanentWidget(keypoint_message_, 1);
    statusBar()->addPermanentWidget(candidate_message_, 2);
    isScaling = true;
    isLogging = false;
    frames.reserve(sampleCount * 5);

    connect(ui->actionClear, &QAction::triggered, this, &MainWindow::clear);
    connect(ui->actionJoint_1, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_2, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_3, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_4, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_5, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_6, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionJoint_7, &QAction::triggered, this, &MainWindow::setJointDisplay);
    connect(ui->actionAuto_Scaling, &QAction::triggered, this, &MainWindow::setScaling);
    connect(ui->actionWindow_Width, &QAction::triggered, this, &MainWindow::setWindowWidth);
    connect(ui->actionLine_Width, &QAction::triggered, this, &MainWindow::setLineWidth);
    connect(ui->actionZoom_In, &QAction::triggered, this, &MainWindow::zoomIn);
    connect(ui->actionZoom_Out, &QAction::triggered, this, &MainWindow::zoomOut);
    connect(ui->actionZoom_Reset, &QAction::triggered, this, &MainWindow::zoomReset);
    connect(ui->actionLoging, &QAction::triggered, this, &MainWindow::setLogging);
    connect(ui->actionSave_Data, &QAction::triggered, this, &MainWindow::log2file);
    connect(&timer, &QTimer::timeout, this, &MainWindow::onTimer);

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(false);
    node_ = std::make_shared<rclcpp::Node>("robot_monitor", "", node_options);
    const auto prompt_topic = node_->declare_parameter<std::string>("prompt_topic", "/sam3/prompt");
    const auto valid_topic = node_->declare_parameter<std::string>("valid_topic", "/perception/valid");
    const auto keypoint_topic = node_->declare_parameter<std::string>(
        "keypoint_topic",
        "/perception/keypoint_3d"
    );
    const auto candidate_topic = node_->declare_parameter<std::string>(
        "candidate_topic",
        "/perception/keypoint_candidates_text"
    );
    keypoint_timeout_sec_ = node_->declare_parameter<double>("keypoint_timeout_sec", 1.0);
    start_time_ = node_->now();
    keypoint_start_time_ = start_time_;

    auto topic_callback =
        [this](robot_control_msgs::msg::RobotState::SharedPtr msg) -> void
    {
        if (msg->robot_state.size() < kExpectedRobotStateSize)
            return;

        rclcpp::Time time = msg->header.stamp;
        if (first_time_)
        {
            start_time_ = time;
            first_time_ = false;
        }
        pushMessage((time - start_time_).seconds(), msg);
    };
    subscription_ = node_->create_subscription<robot_control_msgs::msg::RobotState>(
        "robot_states",
        rclcpp::SensorDataQoS(),
        topic_callback
    );
    prompt_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        prompt_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            std::lock_guard<std::mutex> guard(mtx);
            current_prompt_ = msg->data;
        }
    );
    valid_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        valid_topic,
        10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
        {
            std::lock_guard<std::mutex> guard(mtx);
            keypoint_valid_ = bool(msg->data);
        }
    );
    keypoint_subscription_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        keypoint_topic,
        10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) -> void
        {
            std::lock_guard<std::mutex> guard(mtx);
            keypoint_frame_id_ = msg->header.frame_id;
            keypoint_x_ = msg->point.x;
            keypoint_y_ = msg->point.y;
            keypoint_z_ = msg->point.z;
            have_keypoint_ = std::isfinite(keypoint_x_) && std::isfinite(keypoint_y_) &&
                             std::isfinite(keypoint_z_);
            latest_keypoint_time_ = rclcpp::Time(msg->header.stamp);
            keypoint_stamp_is_zero_ = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
            if (have_keypoint_)
                pushKeypointSample(
                    (node_->now() - keypoint_start_time_).seconds(),
                    keypoint_x_,
                    keypoint_y_,
                    keypoint_z_
                );
        }
    );
    candidate_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        candidate_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            std::lock_guard<std::mutex> guard(mtx);
            current_candidate_summary_ = msg->data;
        }
    );
    RCLCPP_INFO(
        node_->get_logger(),
        "robot_monitor perception subscriptions: prompt=%s valid=%s keypoint=%s candidates=%s",
        prompt_topic.c_str(),
        valid_topic.c_str(),
        keypoint_topic.c_str(),
        candidate_topic.c_str()
    );

    tt = std::make_shared<std::thread>([this]()
                                       {
                                           auto executor =
                                               std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                           executor->add_node(node_);
                                           executor->spin();
                                           QMetaObject::invokeMethod(this, "close", Qt::QueuedConnection);
                                       });
    tt->detach();
    timer.start(50);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    timer.stop();
    if (rclcpp::ok())
        rclcpp::shutdown();
    QMainWindow::closeEvent(event);
}

void MainWindow::log2file()
{
    std::ofstream fout("log.txt");
    fout.setf(std::ios::fixed);
    fout.precision(9);

    std::lock_guard<std::mutex> guard(mtx);
    for (std::size_t frame_index = 0; frame_index < frames.size(); frame_index++)
    {
        fout << frames_time[frame_index] << "\n";
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            for (int joint_index = 0; joint_index < kJointCount; joint_index++)
                fout << frames[frame_index][kJointCount * panel_index + joint_index]
                     << (joint_index == kJointCount - 1 ? "\n" : " ");

        fout << "\n";
    }
}

void MainWindow::setWindowWidth()
{
    bool ok;
    double ww = QInputDialog::getDouble(this, tr("Input Window Width"),
                                        tr("Window Width:"), 10, 0, 100, 2, &ok);
    if (ok)
        time_width = ww;
}

void MainWindow::setLineWidth()
{
    bool ok;
    double ww = QInputDialog::getDouble(this, tr("Input Line Width"),
                                        tr("Line Width:"), 2, 0, 10, 2, &ok);
    if (ok)
    {
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            widgets[panel_index]->setLineWidth(ww);
        keypoint_widget_->setLineWidth(ww);
    }
}

void MainWindow::zoomIn()
{
    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
        widgets[panel_index]->getChart()->zoomIn();
}

void MainWindow::zoomOut()
{
    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
        widgets[panel_index]->getChart()->zoomOut();
}

void MainWindow::zoomReset()
{
    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
        widgets[panel_index]->getChart()->zoomReset();
}

void MainWindow::setScaling(bool checked)
{
    isScaling = checked;
}

void MainWindow::setLogging(bool checked)
{
    isLogging = checked;
    logging_message_->setText(isLogging ? "logging data" : "");
}

void MainWindow::setJointDisplay()
{
    int state[] = {
        ui->actionJoint_1->isChecked(),
        ui->actionJoint_2->isChecked(),
        ui->actionJoint_3->isChecked(),
        ui->actionJoint_4->isChecked(),
        ui->actionJoint_5->isChecked(),
        ui->actionJoint_6->isChecked(),
        ui->actionJoint_7->isChecked(),
    };
    QLineSeries **series[] = {
        widgets[0]->getSeries(),
        widgets[1]->getSeries(),
        widgets[2]->getSeries(),
        widgets[3]->getSeries(),
    };

    for (int joint_index = 0; joint_index < kJointCount; joint_index++)
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            series[panel_index][joint_index]->setVisible(state[joint_index]);
}

void MainWindow::onTimer()
{
    QLineSeries **series[] = {
        widgets[0]->getSeries(),
        widgets[1]->getSeries(),
        widgets[2]->getSeries(),
        widgets[3]->getSeries(),
    };

    std::unique_lock<std::mutex> guard(mtx);

    const std::string prompt = current_prompt_;
    const std::string candidate_summary = current_candidate_summary_;
    const std::string frame_id = keypoint_frame_id_;
    const bool have_keypoint = have_keypoint_;
    const bool keypoint_valid = keypoint_valid_;
    const double keypoint_x = keypoint_x_;
    const double keypoint_y = keypoint_y_;
    const double keypoint_z = keypoint_z_;
    const rclcpp::Time latest_keypoint_time = latest_keypoint_time_;
    const bool keypoint_stamp_is_zero = keypoint_stamp_is_zero_;

    double age_sec = 0.0;
    bool keypoint_fresh = have_keypoint;
    if (have_keypoint && !keypoint_stamp_is_zero && keypoint_timeout_sec_ > 0.0)
    {
        age_sec = (node_->now() - latest_keypoint_time).seconds();
        keypoint_fresh = age_sec <= keypoint_timeout_sec_;
    }

    QString keypoint_text;
    if (prompt.empty())
    {
        keypoint_text = "Prompt: <none> | Keypoint: waiting for /sam3/prompt";
    }
    else if (!have_keypoint)
    {
        keypoint_text = QString("Prompt: %1 | Keypoint: waiting").arg(QString::fromStdString(prompt));
    }
    else if (!keypoint_valid || !keypoint_fresh)
    {
        keypoint_text = QString("Prompt: %1 | Keypoint[%2]: invalid")
                            .arg(QString::fromStdString(prompt))
                            .arg(QString::fromStdString(frame_id.empty() ? "unknown" : frame_id));
        if (!keypoint_fresh)
            keypoint_text += QString(" (stale %1s)").arg(age_sec, 0, 'f', 2);
    }
    else
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "Prompt: " << prompt
               << " | Keypoint[" << (frame_id.empty() ? "unknown" : frame_id) << "]"
               << " x=" << keypoint_x
               << " y=" << keypoint_y
               << " z=" << keypoint_z;
        if (!keypoint_stamp_is_zero)
            stream << " age=" << std::setprecision(2) << age_sec << "s";
        keypoint_text = QString::fromStdString(stream.str());
    }
    keypoint_message_->setText(keypoint_text);

    QString candidate_text;
    if (candidate_summary.empty())
        candidate_text = "Candidates: waiting";
    else
        candidate_text = QString::fromStdString(candidate_summary);
    candidate_message_->setText(candidate_text);

    const auto now = node_->now();
    if (!have_keypoint_log_time_ || (now - last_keypoint_log_time_).seconds() >= 1.0)
    {
        RCLCPP_INFO(node_->get_logger(), "%s", keypoint_text.toStdString().c_str());
        last_keypoint_log_time_ = now;
        have_keypoint_log_time_ = true;
    }
    if (!candidate_summary.empty())
    {
        if (!have_candidate_log_time_ || (now - last_candidate_log_time_).seconds() >= 1.0)
        {
            RCLCPP_INFO(node_->get_logger(), "%s", candidate_summary.c_str());
            last_candidate_log_time_ = now;
            have_candidate_log_time_ = true;
        }
    }

    QLineSeries **keypoint_series = keypoint_widget_->getSeries();
    if (keypoint_buffer_[0].isEmpty())
    {
        for (int axis_index = 0; axis_index < 3; axis_index++)
            keypoint_series[axis_index]->clear();
    }
    else
    {
        std::vector<double> keypoint_lows;
        std::vector<double> keypoint_highs;
        for (int axis_index = 0; axis_index < 3; axis_index++)
        {
            keypoint_series[axis_index]->replace(keypoint_buffer_[axis_index]);
            auto min_max = std::minmax_element(
                keypoint_buffer_[axis_index].begin(),
                keypoint_buffer_[axis_index].end(),
                [](const QPointF &lhs, const QPointF &rhs)
                { return lhs.y() < rhs.y(); });
            keypoint_lows.push_back(min_max.first->y());
            keypoint_highs.push_back(min_max.second->y());
        }

        const double kp_t0 = keypoint_buffer_[0].first().x();
        const double kp_t1 = keypoint_buffer_[0].back().x();
        if (kp_t1 - kp_t0 <= time_width)
            keypoint_widget_->getXAxis()->setRange(kp_t0, kp_t0 + time_width);
        else
            keypoint_widget_->getXAxis()->setRange(kp_t1 - time_width, kp_t1);

        if (isScaling)
        {
            double ymin = *std::min_element(keypoint_lows.begin(), keypoint_lows.end());
            double ymax = *std::max_element(keypoint_highs.begin(), keypoint_highs.end());
            if (ymax - ymin < 1e-4)
            {
                ymin -= 0.05;
                ymax += 0.05;
            }
            keypoint_widget_->getYAxis()->setRange(ymin, ymax);
        }
    }

    if (m_buffer[0][0].isEmpty())
    {
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            for (int joint_index = 0; joint_index < kJointCount; joint_index++)
                series[panel_index][joint_index]->clear();
        return;
    }

    double t0 = m_buffer[0][0].first().x();
    double t1 = m_buffer[0][0].back().x();

    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
    {
        std::vector<double> lows;
        std::vector<double> highs;
        for (int joint_index = 0; joint_index < kJointCount; joint_index++)
        {
            series[panel_index][joint_index]->replace(m_buffer[panel_index][joint_index]);
            auto min_max = std::minmax_element(
                m_buffer[panel_index][joint_index].begin(),
                m_buffer[panel_index][joint_index].end(),
                [](const QPointF &lhs, const QPointF &rhs)
                { return lhs.y() < rhs.y(); });
            lows.push_back(min_max.first->y());
            highs.push_back(min_max.second->y());
        }

        if (isScaling)
        {
            double ymin = *std::min_element(lows.begin(), lows.end());
            double ymax = *std::max_element(highs.begin(), highs.end());
            if (ymax - ymin < 1e-9)
            {
                ymin = ymax - 1.0;
                ymax = ymax + 1.0;
            }
            widgets[panel_index]->getYAxis()->setRange(ymin, ymax);
        }

        if (t1 - t0 <= time_width)
            widgets[panel_index]->getXAxis()->setRange(t0, t0 + time_width);
        else
            widgets[panel_index]->getXAxis()->setRange(t1 - time_width, t1);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::pushMessage(double t, robot_control_msgs::msg::RobotState::SharedPtr msg)
{
    if (msg->robot_state.size() < kExpectedRobotStateSize)
        return;

    const double *state = msg->robot_state.data();
    std::lock_guard<std::mutex> guard(mtx);

    if (m_buffer[0][0].size() < sampleCount)
    {
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            for (int joint_index = 0; joint_index < kJointCount; joint_index++)
                m_buffer[panel_index][joint_index].append(
                    QPointF(t, state[panel_index * kJointCount + joint_index]));
    }
    else
    {
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            for (int joint_index = 0; joint_index < kJointCount; joint_index++)
            {
                m_buffer[panel_index][joint_index].removeFirst();
                m_buffer[panel_index][joint_index].append(
                    QPointF(t, state[panel_index * kJointCount + joint_index]));
            }
    }

    if (isLogging)
    {
        frames.push_back(msg->robot_state);
        frames_time.push_back(t);
    }
}

double MainWindow::getDataDuration()
{
    std::lock_guard<std::mutex> guard(mtx);
    if (m_buffer[0][0].isEmpty())
        return 0.0;
    return m_buffer[0][0].back().x() - m_buffer[0][0].first().x();
}

void MainWindow::clear()
{
    std::lock_guard<std::mutex> guard(mtx);
    for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
        for (int joint_index = 0; joint_index < kJointCount; joint_index++)
            m_buffer[panel_index][joint_index].clear();
    for (int axis_index = 0; axis_index < 3; axis_index++)
        keypoint_buffer_[axis_index].clear();
    frames.clear();
}

void MainWindow::pushKeypointSample(double t, double x, double y, double z)
{
    const double values[3] = {x, y, z};
    for (int axis_index = 0; axis_index < 3; axis_index++)
    {
        if (keypoint_buffer_[axis_index].size() >= sampleCount)
            keypoint_buffer_[axis_index].removeFirst();
        keypoint_buffer_[axis_index].append(QPointF(t, values[axis_index]));
    }
}
