

#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
//#include <QtCharts/QChartGlobal>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtWidgets/QVBoxLayout>


class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QString & t, QWidget *parent = nullptr);
    ~Widget();
     QLineSeries ** getSeries();
     QValueAxis *getXAxis();
     QValueAxis *getYAxis();
     QChart *getChart();
     QChartView *getChartView();
     void setLineWidth(double width);
private:
    QChart *m_chart;
    QLineSeries *m_series[7] ;
    QString title;
    QValueAxis *axisX;
    QValueAxis *axisY;
    QChartView *chartView;
};

#endif // WIDGET_H
