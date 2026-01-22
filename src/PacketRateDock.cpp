#include "PacketRateDock.h"
#include "ui_PacketRateDock.h"

PacketRateDock::PacketRateDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::PacketRateDock)
    , m_chart(new QChart)
    , m_series(new QLineSeries)
    , m_axisX(new QValueAxis)
    , m_axisY(new QValueAxis)
{
    ui->setupUi(this);

    m_chart->addSeries(m_series);
    m_chart->legend()->hide();
    m_chart->setTitle("Packet Rate");

    m_axisX->setTitleText("Time (last 25 seconds @ 10 Hz)");
    m_axisX->setRange(0, MAX_SAMPLES);

    m_axisY->setTitleText("Packets / second");
    m_axisY->setRange(0, 1000);   // typical full packet rate is ~500/sec

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    m_series->attachAxis(m_axisX);
    m_series->attachAxis(m_axisY);

    ui->chartView->setChart(m_chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
}

PacketRateDock::~PacketRateDock()
{
    delete ui;
}

void PacketRateDock::reset()
{
    m_series->clear();
    m_sampleIndex = 0;
    m_axisX->setRange(0,MAX_SAMPLES);
}

void PacketRateDock::addSample(double packetsPerSecond)
{
    m_series->append(m_sampleIndex, packetsPerSecond);
    ++m_sampleIndex;

    if (m_series->count() > MAX_SAMPLES)
    {
        m_series->remove(0);

        // shift X-axis window
        m_axisX->setRange(
            m_sampleIndex - MAX_SAMPLES,
            m_sampleIndex
            );
    }

    // Optional: auto-scale Y if needed
    if (packetsPerSecond > m_axisY->max())
        m_axisY->setMax(packetsPerSecond * 1.1);
}
