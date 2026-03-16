#include "widget.h"
#include <QPainter>


Widget::Widget(const QString &t, QWidget *parent) :
    QWidget(parent),
    m_chart(new QChart),
    title(t)
{
    chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    m_chart->setTheme(QChart::ChartThemeDark);
    axisX = new QValueAxis;
    axisX->setRange(0, 10);
    axisX->setLabelFormat("%.2f");
    axisX->setTitleText("time / s");
    axisY = new QValueAxis;
    axisY->setRange(-1, 1);
    axisY->setTitleText(title);
    axisY->setLabelFormat("%.4f");
    m_chart->addAxis(axisX, Qt::AlignBottom);
    m_chart->addAxis(axisY, Qt::AlignLeft);
    for(int i = 0; i < 7; i++)
    {
        m_series[i] = new QLineSeries;
        m_series[i]->setName(QString("J%1").arg(i + 1));
        m_chart->addSeries(m_series[i]);
        m_series[i]->attachAxis(axisY);
        m_series[i]->attachAxis(axisX);
    }
    m_chart->legend()->setAlignment(Qt::AlignRight);
    m_chart->setMargins(QMargins(1,1,1,1));
    m_chart->setTitle(title);
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0,0,0,0);
    mainLayout->addWidget(chartView);
    chartView->update();
}

QChart * Widget::getChart()
{
    return m_chart;
}

QChartView *Widget::getChartView()
{
    return chartView;
}
QValueAxis *Widget::getXAxis()
{
    return axisX;
}
QValueAxis *Widget::getYAxis()
{
    return axisY;
}
QLineSeries ** Widget::getSeries()
{
    return m_series;
}
void Widget::setLineWidth(double width)
{
    for(int i = 0; i < 7; i++)
    {
        QPen pen = m_series[i]->pen();
        pen.setWidthF(width);
        m_series[i]->setPen(pen);
    }
}
Widget::~Widget()
{
    delete m_chart;
}
