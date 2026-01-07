//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: MainWindow.h
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
//  of packets and optionally saves them to a CSV file.
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//  V0.2.1  2026-01-05  Changed LidarDecoder.h and cpp to L2lidar
//                      Added user commands to control L2 lidar
//                      Changed class name from LidarDecoer to L2lidar
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
//
// Corrections/additions have been made to these 2 files
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


#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <deque>
// atomic is used for thread safe variables
// to provide indivisble read modify write operations
// such as incrementing a variable in a multi-threaded app
#include <atomic>
#include <QUdpSocket>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "ConfigDialog.h"
#include "L2lidar.h"

#define CHART_UPDATE_TIMER 100

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    // button controls
    void L2connect();
    void L2disconnect();
    void openConfig();

    // L2 commands
    void startRotation();
    void stopRotation();
    void sendReset();
    void getVersion();

    // ui updates
    void updateChart();

private:
    // Application MainWindow ui
    Ui::MainWindow* ui;

    // Socket class for UDP
    QUdpSocket udpSocket;

    // Chart timer to trigger chart update
    QTimer chartTimer;

    // variables for chart
    QLineSeries* rateSeries{ nullptr };
    std::deque<uint32_t> recentRates;
    const size_t maxPoints = 100;
    uint64_t LastRateCount {0};

    //-----------------------------------------------------
    // unitree L2 lidar hardware
    //-----------------------------------------------------
    L2lidar l2lidar;

    //-----------------------------------------------------
    // ??? this will need attention later
    // CSV file for saving packet data
    //-----------------------------------------------------
    QFile csvFile;
    QTextStream csvStream;
    ConfigDialog config;

    void saveSettings();
    void loadSettings();
    void saveCsv(const QByteArray& datagram);
};
