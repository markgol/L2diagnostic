//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: PacketRateDock.cpp
//
//  Purpose:
//  Determine correct operation of the Unitreee L2 Lidar hardware
//  and software.  Establish platform independent software protocols
//  for using the L2 Lidar with its Ethernet interface.
//
//  Background:
//  Unitree provides undoucmented software files in the form:
//      include files
//      example application files
//      .a Archive Library
//
//  The source files rely on an Archive library using POSIX I/O
//  No source exists for the archive Library making it diffcult
//  to debug or port usage of the L2 Lidar for other platforms.
//  The hardware has 2 mutually exclusive communication interfaces:
//      Ethernet using UDP
//      Serial UART
//  The serial UART is limited in speed and does not operate at
//  the full sensor speed of 64K/sec sample points.
//
//  Solution:
//  This software skeleton was created using directed ChatGPT AI
//  conversation targetting a QT Creator development platform.
//  It reads UPD packets from the L2, caterorizes them, performs
//  error detection for bad packets (lost), display subsample
//  of packet.
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V1.0.0  2026-03-28  Offical release
//
//--------------------------------------------------------

//--------------------------------------------------------
// This app uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities.h
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies.  These have been minor in most
// instances. These are what are acutally being used:
//      unitree_lidar_protocolL2.h
//      unitree_lidar_utilitiesL2.h
//
// Copyright (c) 2024, Unitree Robotics
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
//
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of L2diagnsotic.
//
// L2diagnsotic is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// L2diagnsotic is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------

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
    m_chart->setTitle("Receive Packet Rate");

    m_axisX->setTitleText("");
    m_axisX->setRange(0, MAX_SAMPLES);
    m_axisX->setLabelsVisible(false);
    m_axisX->setTickCount(6);
    m_axisX->setMinorTickCount(4);

    m_axisY->setTitleText("Packets / second");
    m_axisY->setRange(0, 750);   // typical full packet rate is ~500/sec
    m_axisY->setTickCount(6);
    m_axisY->setMinorTickCount(2);

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
