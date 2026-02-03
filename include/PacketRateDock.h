#pragma once

#include <QDockWidget>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

QT_BEGIN_NAMESPACE
namespace Ui { class PacketRateDock; }
QT_END_NAMESPACE

class PacketRateDock : public QDockWidget
{
    Q_OBJECT

public:
    explicit PacketRateDock(QWidget* parent = nullptr);
    ~PacketRateDock();

    void addSample(double packetsPerSecond);
    void reset();

private:
    Ui::PacketRateDock* ui;

    QChart*       m_chart;
    QLineSeries* m_series;
    QValueAxis*  m_axisX;
    QValueAxis*  m_axisY;

    static constexpr int MAX_SAMPLES = 250;
    int m_sampleIndex = 0;
};
