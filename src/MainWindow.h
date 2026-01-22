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
//  V0.2.3  2026-01-10  Add point cloud viewer
//  V0.2.4  2026-01-11  Changed OpenGL approach
//  V0.2.5  2026-01-12  Reogranized MainWindow.cpp
//                      removed save CSV file skeleton
//                      Added menu to enable/disable docks, point cloud viewer
//                      and packet stats/rate chart
//                      Added dockable windows
//                          Calibration and internal state
//                          IMU information
//                          packet stats information
//                          packet rate chart
//  V0.2.8  2026-01-16  Changed point cloud viewer to dockable window
//                      Added reset of layout
//  V0.3.2  2026-01-22  New renderer architecture
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

// Qt includes
#include <QMainWindow>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QUdpSocket>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QVector>
#include <QQueue>
#include <QElapsedTimer>

// project specific includes
#include "ConfigDialog.h"
#include "L2lidar.h"
#include "PointCloudWindow.h"
#include "DiagnosticsDock.h"
#include "IMUDock.h"
#include "StatsDock.h"
#include "ACKDock.h"
#include "ControlsDock.h"
#include "PacketRateDock.h"

#define LIDAR_MODE_3D 0
#define LIDAR_MODE_2D 1

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

using Frame = QVector<PCpoint>;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

public slots:
    // this is in response to set view button in the config dialog
    void handleResetView();

private slots:
    // The slots are triggered by ControlsDock class

    // // button controls
    void L2connect();
    void L2disconnect();
    void openConfig();

    // L2 commands
    void startRotation();
    void stopRotation();
    void sendReset();
    void getVersion();

private:
    // Application MainWindow ui
    Ui::MainWindow* ui;

    //-----------------------------------------------------
    // dockable diagnostics. imu and  stats ui
    //-----------------------------------------------------
    QTimer* mHeartBeat;  // start in L2 connect
            // initialize in MainWindow constructor

    void HeartbeatFire();   // this is the callback for
                            // heartbeat timer

    // dockable windows
    DiagnosticsDock *m_diagnosticsDock{nullptr};
    IMUDock *m_IMUDock{nullptr};
    StatsDock *m_StatsDock{nullptr};
    ACKDock *m_ACKDock{nullptr};
    ControlsDock *m_controlsDock{nullptr};
    PacketRateDock* m_packetRateDock{nullptr};

    // update the dockable windows
    void updateDiagnostics(); // runs off of timer to feed updated
                            // diagnostics data to diagnostic dock
    void updateIMU();
    void updateStats();
    void updatePacketRate();


    // ack packets are very low rate, only occurs
    // when a command is sent to the hardware
    // it should be connected to a signal
    // void MainWindow::updateACK()
    void updateACK();

    //-----------------------------------------------------
    // Packet Chart view with packet stats
    //-----------------------------------------------------
    QElapsedTimer*  m_rateTimer; // this measures actual elpased time
    QTimer*         mPacketBeat; // this is heartbeat for the packet rate chart
    uint64_t        m_lastPacketCount = 0;

    // L2 workmode
    int mLidarScanMode {LIDAR_MODE_3D}; // default is 3D scanning

    //-----------------------------------------------------
    // Point cloud veiwer
    //-----------------------------------------------------
    PointCloudWindow* m_pointCloudWindow{nullptr};

    PCsettings defaultPCsettings {10.0,145.0,20.0};
    void SetDefaultView();

    // Point cloud window renderer Timer
    QTimer* RendererTimer;

    // close point cloud viewer
    void closeEvent(QCloseEvent* e);

    // sends last frame received to renderer
    void onNewLidarFrame();

    // throttle for point cloud viewer
    uint32_t NumFramesToSkip {4};
    uint32_t CurrentSkipCount {0};

    // mutexf for point cloud frame updates
    QMutex m_cloudMutex;

    // ring storage for point cloud frames to display
    uint32_t mMax3Dframes2Buffer {MAX_3DPOINTS_PER_FRAME}; // maximum number of 3D frames
                                        // to buffer for display

    uint32_t mMax2Dframes2Buffer {MAX_2DPOINTS_PER_FRAME}; // maximum number of 2D frames
                                        // to buffer for display
    int mmaxFrames{0};  // maximum number of frames for current scan mode
    int mmaxPoints{0}; // computed maximum number of points

    QVector<Frame> m_frameRing;   // Fixed-capacity ring
    size_t m_ringWrite = 0;       // Next write index
    size_t m_ringCount = 0;       // Number of valid frames (<= MAX_FRAMES)

    // helper function for Config dialog when cancelling dialog
    void RestoreConfigSettings(); // reset the point cloud view back to defaults

    //-----------------------------------------------------
    // For unitree L2 lidar hardware interaction
    //-----------------------------------------------------
    L2lidar l2lidar;

    // configuration dialog
    ConfigDialog config;

    // helper functions
    void L2DisconnectedButtonsUIs(); // set buttons and UIs states when L2 disconnected
    void L2ConnectedButtonsUIs(); // set buttons and UIs states when L2 connected

    void StopPointCloudViewer();
    void StartPointCloudViewer();

    void StartPacketChart();
    void StopPacketChart();

    // these are only used in the class constructor
    // to help orgranize the flow for creating the
    // dockable windows and the point cloud viewer
    void SetupGUIrefreshTimers();
    void createDocksViewer();
    void AssignDocksObjectNames();
    void AddDocksViewer();
    void ConnectDocksViewerActions();
    void applyDocksVisibilityConstraint();
    void ShowWindows();

    //-----------------------------------------------------
    // INI settings functions
    //-----------------------------------------------------
    void resetWindowLayout();
    void saveSettings(bool resetRequested);
    void loadSettings(bool resetRequested);
    bool GetSettingsReset();
    void SetSettingsReset(bool Reset);


};
