#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtCharts/QValueAxis>
#include <QChart>
#include <QChartView>
#include <QMetaObject>
#include <QInputDialog>
#include <QLabel>
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

    logging_message_ = new QLabel(this);
    keypoint_message_ = new QLabel(this);
    statusBar()->addPermanentWidget(logging_message_);
    statusBar()->addPermanentWidget(keypoint_message_, 1);
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
    keypoint_timeout_sec_ = node_->declare_parameter<double>("keypoint_timeout_sec", 1.0);
    start_time_ = node_->now();

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
        }
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
        for (int panel_index = 0; panel_index < kPanelCount; panel_index++)
            widgets[panel_index]->setLineWidth(ww);
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

    if (have_keypoint && keypoint_valid && keypoint_fresh)
    {
        const auto now = node_->now();
        if (!have_keypoint_log_time_ || (now - last_keypoint_log_time_).seconds() >= 1.0)
        {
            RCLCPP_INFO(node_->get_logger(), "%s", keypoint_text.toStdString().c_str());
            last_keypoint_log_time_ = now;
            have_keypoint_log_time_ = true;
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
    frames.clear();
}
