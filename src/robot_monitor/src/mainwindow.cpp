#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtCharts/QValueAxis>
#include <QChart>
#include <QChartView>
#include <QInputDialog>
#include <QLabel>
#include <algorithm>
#include <fstream>

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

    message = new QLabel(this);
    statusBar()->addPermanentWidget(message);
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
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("robot_monitor", "", node_options);
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

    tt = std::make_shared<std::thread>([this]()
                                       {
                                           auto executor =
                                               std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                           executor->add_node(node_);
                                           executor->spin();
                                           this->close();
                                       });
    tt->detach();
    timer.start(50);
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
    message->setText(isLogging ? "logging data" : "");
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
