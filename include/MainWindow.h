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
//  of packets.
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
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//  V0.3.5  2026-01-24  removed remnant from old renderer architecture
//                      Added display of 2d point cloud data
// V0.3.6   2026-01-25  Added IMU orientation correction to point cloud data
//                      Added measure latency button
// V0.3.8   2026-01-28  removed mesure latency button
// V0.3.9   2026-01-30  Added Sync L2 timestamp button
//                      Added SAVE/LOAD point cloud
// V0.3.10  2026-02-01  Added Get L2 params
//                      Added Get L2 workmode button
// V0.3.12  2026-02-05  Moved renderer timer to PointCloudWindow class
// V0.4.3   2026-02-20  Added 'n' frame aggregation for point cloud frame
//                      0 is no aggregation, 38 matches one hemishpere scan
//                      Moved L2lidar class into its own release
//  V0.4.4  2026-02-22  Added stats to the IMU variables
//  V1.0.0  2026-03-28  Official release
//  V1.3.0  2026-06-18  Updated to L2lidarCLass V1.3.4
//                      Added derived stats for roll, pitch and yaw
//                      Added use system time for packets option
//                      Added checks on timestamp correction parameters
//                      Moved quaterion and euler methods to quaternion.h
//                      Removed conditional use of timestamp correction
//                          for aggregation.
//  V1.3.1  2026-06-21  Added config params for settings gatewey IP address
//                      Corrected initial size of IMUstats window
//                      Added save current view as default view
//  V1.3.2  2026-07-07  Updated to V1.3.6 of L2LidarClass
//                      Added point cloud flattened scan capability
//                      Added starting angle, angle width for point cloud capture
//  V2.0.0  2026-07-11  Adding calibration mode and diagnostic mode to the app.
//                      Startup mode is always diagnostic.
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies.  These have been minor in most
// instances.
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
#pragma once

// Qt includes
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QMainWindow>
#include <QQueue>
#include <QTextStream>
#include <QTimer>
#include <QUdpSocket>
#include <QVector>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <cstdio>

// project specific includes
#include "ConfigDialog.h"
#include "L2lidar.h"
#include "PointCloudWindow.h"
#include "DiagnosticsDock.h"
#include "IMUDock.h"
#include "StatsDock.h"
#include "ACKDock.h"
#include "ControlsDock.h"
#include "CalibrationDock.h"
#include "WorkModeDialog.h"
#include "PacketRateDock.h"
#include "RangeCalinfoDock.h"
#include "quaternion.h"

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
    //MainWindow::MainWindow(bool OpenGLES,int major, int minor, maQWidget* parent)
    explicit MainWindow(bool OpenGLES = false,
                        int major = 3,
                        int minor = 3,
                        QWidget* parent = nullptr);
    ~MainWindow();

    void SetOpenGLVersion(bool OpenGLtype, int MajorVersion, int MinorVersion) {
        mOpenGLES=OpenGLtype;
        mOpenGLmajorV = MajorVersion;
        mOpenGLminorV = MinorVersion;
    }

    bool IsOpenGlES() {return mOpenGLES;}
    int GetOpenGLmajorV() { return mOpenGLmajorV;}
    int GetOpenGLminorV() { return mOpenGLminorV;}

public slots:
    // this is in response to set view button in the config dialog
    void handleResetView();
    void handleCurrentPCView();
    void handleConfigureUDP();
    void handleSetL2MAC();
    void sendSetWorkmode();
    void sendGetL2Workmode();
    void sendReset();
    void CalDockRangeCorrectionChanged();

private slots:
    // // button controls
    void L2connect();
    void L2disconnect();
    void openConfig();
    void openWorkmode();
    void ClearPCwindow();
    void SyncL2Clock();
    bool SavePC();
    void LoadPC();
    void SetCalibrationMode();
    void SetDiagnosticMode();
    void UpdateRangeCalInfo();

    // L2 commands
    void startRotation();
    void stopRotation();
    void getVersion();
    void GetL2Params();

    // workmode
    void ClosedWorkmodeDialog();

    // Range Calibration GUI request
    void RangeCalGUI(bool clear, bool visible);
    void Stage3accepted();
    void Stage3rejected();

    void ResetConfigScanSettings(); // restore the StartScanAngle,
                                    //ScanAngleWidth, Flatten flag,
                                    //IMUadjust flag, IMUrollPith only flag

    void SavePC4Stage2();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    // operating mode
    bool mCalibrationMode {false};
        // false    diagnostic mode
        // true     calibration mode

    // helper conversion functions
    QByteArray convertMacStringToByteArray(const QString &macString);

    // Application MainWindow ui
    Ui::MainWindow* ui;

    bool mOpenGLES {false}; // true if OpenGL ES, else OpenGL Core
    int mOpenGLmajorV {0};
    int mOpenGLminorV {0};
    int mNoGraphics {false};

    //-----------------------------------------------------
    // dockable diagnostics. imu and  stats ui
    //-----------------------------------------------------
    QTimer* mHeartBeat;  // start in L2 connect
            // initialize in MainWindow constructor

    void HeartbeatFire();   // this is the callback for
                            // heartbeat timer

    //-----------------------------------------------------
    // For unitree L2 lidar hardware interaction
    //-----------------------------------------------------
    L2lidar l2lidar;

    // dockable windows
    DiagnosticsDock *m_diagnosticsDock{nullptr};
    IMUDock *m_IMUDock{nullptr};
    StatsDock *m_StatsDock{nullptr};
    ACKDock *m_ACKDock{nullptr};
    ControlsDock *m_controlsDock{nullptr};
    PacketRateDock* m_packetRateDock{nullptr};
    CalibrationDock* m_calibrationDock {nullptr};
    RangeCalinfoDock* m_RangeCalinfoDock {nullptr};
    CalGraphDock* m_CalGraphDock {nullptr};

    // update the dockable windows
    void updateDiagnostics(); // runs off of timer to feed updated
                            // diagnostics data to diagnostic dock
    void updateIMU();
    void updateStats();
    void updatePacketRate();


    // ack packets are very low rate, only occurs
    // when a command is sent to the hardware
    void updateACK();

    //-----------------------------------------------------
    // Packet Chart view with packet stats
    //-----------------------------------------------------
    QElapsedTimer*  m_rateTimer; // this measures actual elpased time
    QTimer*         mPacketBeat; // this is heartbeat for the packet rate chart
    uint64_t        m_lastPacketCount = 0;
    float           mPacketRate{0.0}; // current packet rate

    //-----------------------------------------------------
    // Point cloud veiwer
    //-----------------------------------------------------
    PointCloudWindow* m_pointCloudWindow{nullptr};

    // This does all the dirty work for opening
    // The PointCloudWindow class
    bool OpenPointCloudWindow();

    PCsettings defaultPCsettings {10.0,145.0,20.0,1.0,0.0,40.0};
    void SetDefaultView();

    int mNumFramestoAggregate {38};

    // sends last frame received to renderer
    void onNewLidarFrame(bool Frame3D);
    void onNew3DLidarFrame() {onNewLidarFrame(true); mLastTypePacketReceived = true;};
    void onNew2DLidarFrame() {onNewLidarFrame(false); mLastTypePacketReceived = false;};

    bool mLastTypePacketReceived = {true}; // false - 2D packet received
                                            // true - 3D packet recieved

    // throttle for point cloud viewer
    uint32_t mNumFramesToSkip {4};

    // mutexf for point cloud frame updates
    QMutex m_cloudMutex;

    int mmaxPoints{0}; // computed maximum number of points

    // IMU operations
    bool menableIMUstats {true}; // calculate stats on IMU data
    StatsIMU mImuStats;
    void onNewLidarIMU();   // this is only used when enableIMUstatus is true
    void CalcIMUstats(LidarImuDataPacket Imu, StatsIMU& ImuStats);
    void MeanDev(float Value, float* MeanValue, float* sigmaValue, float Alpha);

    // helper function for Config dialog when cancelling dialog
    void RestoreConfigSettings(); // reset the point cloud view back to defaults

    // configuration dialog
    ConfigDialog config;

    // workmode dialog
    WorkmodeDialog WorkMode;

    // retrieve workmode from L2
    void GetL2workmode();

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

    //-----------------------------------------------------
    // logging functions
    //-----------------------------------------------------
    bool mLogging {false};
    QString mLogFilename {"L2diagnostic.log"};
    FILE *mLogFile {NULL};
    void logParameter(const char *Name, double value);
    void logParameter(const char *Name, bool value);
    void logParameter(const char *Name, float value);
    void logParameter(const char *Name, int value);
    void logParameter(const char *Name, QString text);
};
