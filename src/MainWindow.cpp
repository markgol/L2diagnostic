//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: MainWindow.cpp
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
//                      Changed class name from LidarDecoder to L2lidar
//                      Added L2 information to GUI
//                      Added L2 start rotation, stop rotation,
//                              reset and get Version info buttons
//                      Changed config dialog
//                          for src and dest ip, port
//                      Updated @notes for unitree_lidar_protocols.h
//  V0.2.2  2026-01-07  Added ACK packets decodes to ui
//          2026-01-08  Added Mutex access to packet copies
//                      Added 3d point packet to 3d PCL cloud conversion
//                          to demonstrate packets are being received
//                          and processed correctly
//  V0.2.3  2026-01-08  Added point cloud viewer
//                      updated PC stats
//  V0.2.4  2026-01-10  Started add of Calibration and internal State dialog
//                      Started add of set work mode dialog
//                      Changed OpenGL approach to add coloring
//                      Added SkipFrame settings for point cloud display
//  V0.2.5  20260-01-12 Start of mainwindow GUI reorg
//                      Added heartbeat timer to update some of the
//                      dockable uis
//                      Added IMU dockable window
//                      Added ACK dockable window
//                      Added packet stats dockable window
//                      Added Calib and internal state dockable window
//  V0.2.8  2026-01-16  Changed point cloud viewer to dockable window
//  V0.3.0  2026-01-18  Changed point cloud veiwer to OpenGL window
//                      PC viewer as dockable window intractable
//                      Added point cloud view parameters and controls
//                      to configuration dialog
//                      Added separate renderer timer
//  V0.3.2  2026-01-22  New renderer architecture
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//                      L2 Workmode implemented
//                      2D packets decoded but not displayed
// V0.3.5   2026-01-24  Removal of MainWindow ring buffer for frames
//                      removed remnants from old renderer architecture
//                      Added display of 2d point cloud data
// V0.3.6   2026-01-25  Added IMU orientation correction to point cloud data
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

//--------------------------------------------------------
//  Data flow for point cloud
//      MainWindow:onNewLidarFrame()  this operates at L2 point cloud
//          |              packet rate ~200-250 packets/sec
//          |
//      throttling         save only every nth PC frame ( 0 every frame)
//          |
//      PointCloudWindow::appendFrame()
//          |
//      VBO sub-write       accumulates cloud points from frames
//          |
//      requestUpdate->renderer   queued for next paintGL
//          |
//      QOpenGLWindow::paintGL()  timer driven typically at 30-60Hz
//
//--------------------------------------------------------


//--------------------------------------------------------
// Main project includes required before anything else
//--------------------------------------------------------
#include "MainWindow.h"
#include "unitree_lidar_protocol.h"
#include "unitree_lidar_utilities.h"

//--------------------------------------------------------
// Autogenerated Qt desktop GUIM include
// This is generated from the MainWindow.ui file
// The CMakeFIle.txt must include:
//      set(CMAKE_AUTOMOC ON)
//      set(CMAKE_AUTOUIC ON)
//      set(CMAKE_AUTORCC ON)
//--------------------------------------------------------
#include "ui_MainWindow.h"

//--------------------------------------------------------
//  Qt includes
//--------------------------------------------------------
#include <QHostAddress>
#include <QDateTime>
#include <QSettings>
#include <QStandardPaths>
#include <QDebug>
#include <QMessageBox>
//--------------------------------------------------------
//  Project specific includes not part of MainWindow.h
//--------------------------------------------------------

//--------------------------------------------------------
//  MainWIndow class constructor
//--------------------------------------------------------
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    createDocksViewer();
    AssignDocksObjectNames();
    AddDocksViewer();

    // Load previously saved user settings
    loadSettings(GetSettingsReset());
    SetSettingsReset(false);

    // create cloud viewer window
    // This is not a Qt window but a OpenGL managed window
    if(mmaxPoints>=50000) {
        OpenPointCloudWindow();
    }

    SetupGUIrefreshTimers();
    ConnectDocksViewerActions();

    // these are only done after loadsettings()
    applyDocksVisibilityConstraint();

    NumFramesToSkip = config.getSkipFrame();

    // load the com parameters in the l2lidar class
    l2lidar.LidarSetCmdConfig(config.getSRCip(),config.getSRCport(),
                              config.getDSTip(),config.getDSTport());

    // initial state of buttons and uis is L2 disconnected
    L2DisconnectedButtonsUIs(); // set buttons and UIs states when L2 disconnected

    // connect config request from set view button
    connect(&config, &ConfigDialog::requestViewReset,
            this, &MainWindow::handleResetView);

    ShowWindows(); // show windows effects all windows including point cloud window
}

//--------------------------------------------------------
//  MainWIndow class destructor
//--------------------------------------------------------
MainWindow::~MainWindow()
{
    // Make sure live capture is stopped
    L2disconnect();
    // Save current user settings
    // make sure requested reset is not cleared
    saveSettings(GetSettingsReset());

    delete ui;
}

//========================================================
// constructor helpers
//========================================================

//--------------------------------------------------------
// OpenPointCloudWindow
//--------------------------------------------------------
void MainWindow::OpenPointCloudWindow()
{
    // only open point cloud window if there is a minimum
    // number point cloud buffer size
    // configure OpenGL before creating PointCloudWindow class
    // so that it has the correct OpenGL context
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setRenderableType(QSurfaceFormat::OpenGL);

    QSurfaceFormat::setDefaultFormat(format);
    m_pointCloudWindow = new PointCloudWindow(mmaxPoints);
    // set default view settings
    SetDefaultView();
    m_pointCloudWindow->setTransientParent(windowHandle());
    m_pointCloudWindow->Initialize();
}

//--------------------------------------------------------
// SetDefaultView
//--------------------------------------------------------
void MainWindow::SetDefaultView() {
    defaultPCsettings.Distance = config.getPCWdistance();
    defaultPCsettings.Yaw = config.getPCWyaw();
    defaultPCsettings.Pitch = config.getPCWpitch();
    defaultPCsettings.PointSize = config.getPointSize();
    defaultPCsettings.MinDistance = config.getMinDistance();
    defaultPCsettings.MaxDistance = config.getMaxDistance();
    m_pointCloudWindow->setDefaultPCsettings(defaultPCsettings);
}

//--------------------------------------------------------
// SetupGUIrefreshTimers
//--------------------------------------------------------
void MainWindow::SetupGUIrefreshTimers()
{
    //--------------------------------------------------------
    //  This timer is used to trigger updates
    //  for Diagnostics, IMU and Stats
    //--------------------------------------------------------
    mHeartBeat = new QTimer(this);
    mHeartBeat->setInterval(config.getDiagUpdateRate());
    connect(mHeartBeat, &QTimer::timeout, this, &MainWindow::HeartbeatFire);

    //--------------------------------------------------------
    //  These timers are used to trigger updates
    //  for packet rate chart update
    //--------------------------------------------------------
    mPacketBeat = new QTimer(this);
    mPacketBeat->setInterval(config.getPacketUpdateRate());
    connect(mPacketBeat, &QTimer::timeout, this, &MainWindow::updatePacketRate);
    // The elpased timer is used for calculating the packet rate/sec
    // It does not tigger and signals
    m_rateTimer = new QElapsedTimer;
    m_rateTimer->restart();

    //--------------------------------------------------------
    // setup timer point cloud renderering
    //--------------------------------------------------------
    RendererTimer = new QTimer(this);
    RendererTimer->setInterval(config.getRenderRate());
    connect(RendererTimer,
            &QTimer::timeout,
            m_pointCloudWindow,
            &PointCloudWindow::onRenderTick);
    RendererTimer->stop(); // started in L2 connect
}

//--------------------------------------------------------
// createDocksViewer
//--------------------------------------------------------
void MainWindow::createDocksViewer()
{
    //--------------------------------------------------------
    //  setup the dockable diagnsotics gui
    //--------------------------------------------------------
    m_diagnosticsDock = new DiagnosticsDock(this);
    //--------------------------------------------------------
    //  setup dockable IMU gui
    //--------------------------------------------------------
    m_IMUDock = new IMUDock(this);
    //--------------------------------------------------------
    //  setup dockable Controls gui
    //--------------------------------------------------------
    m_controlsDock = new ControlsDock(this);
    //--------------------------------------------------------
    //  setup dockable packet Stats gui
    //--------------------------------------------------------
    m_StatsDock = new StatsDock(this);
    //--------------------------------------------------------
    //  setup dockable ACK gui (this is not timer driven)
    //--------------------------------------------------------
    m_ACKDock = new ACKDock(this);
    //--------------------------------------------------------
    //  packetRateDock setup
    //--------------------------------------------------------
    m_packetRateDock = new PacketRateDock(this);
}

//--------------------------------------------------------
//  closeEvent
//--------------------------------------------------------
void MainWindow::closeEvent(QCloseEvent* e)
{
    if (m_pointCloudWindow) {
        // save window geometry and state before closing
        m_pointCloudWindow->saveWindowState();
        // kill window
        m_pointCloudWindow->close();
        delete m_pointCloudWindow;
        m_pointCloudWindow = nullptr;
    }

    WorkMode.close();

    QMainWindow::closeEvent(e);
}

//--------------------------------------------------------
//  handleResetView from config dialog set view button
//  changes aren't permanent till okay button is pressed
//--------------------------------------------------------
void MainWindow::handleResetView()
{
    defaultPCsettings.Distance = config.getPCWdistance();
    defaultPCsettings.Yaw = config.getPCWyaw();
    defaultPCsettings.Pitch = config.getPCWpitch();
    defaultPCsettings.PointSize = config.getPointSize();
    defaultPCsettings.MinDistance = config.getMinDistance();
    defaultPCsettings.MaxDistance = config.getMaxDistance();

    m_pointCloudWindow->setPCsettings(defaultPCsettings);
}

//--------------------------------------------------------
// AssignDocksObjectNames
//--------------------------------------------------------
void MainWindow::AssignDocksObjectNames()
{
    //--------------------------------------------------------
    //  setup the dockable diagnsotics gui
    //--------------------------------------------------------
    m_diagnosticsDock->setObjectName("DiagnosticsDock");
    //--------------------------------------------------------
    //  setup dockable IMU gui
    //--------------------------------------------------------
    m_IMUDock->setObjectName("IMUDock");
    //--------------------------------------------------------
    //  setup dockable Controls gui
    //--------------------------------------------------------
    m_controlsDock->setObjectName("ControlsDock");
    //--------------------------------------------------------
    //  setup dockable packet Stats gui
    //--------------------------------------------------------
    m_StatsDock->setObjectName("StatsDock");
    //--------------------------------------------------------
    //  setup dockable ACK gui (this is not timer driven)
    //--------------------------------------------------------
    m_ACKDock->setObjectName("ACKDock");
    //--------------------------------------------------------
    //  packetRateDock setup
    //--------------------------------------------------------
    m_packetRateDock->setObjectName("PacketRateDock");
    //--------------------------------------------------------
}

//--------------------------------------------------------
// addDocksViewer
//--------------------------------------------------------
void MainWindow::AddDocksViewer()
{
    //--------------------------------------------------------
    //  setup the dockable diagnsotics gui
    //--------------------------------------------------------
    addDockWidget(Qt::RightDockWidgetArea, m_diagnosticsDock);
    //--------------------------------------------------------
    //  setup dockable IMU gui
    //--------------------------------------------------------
    addDockWidget(Qt::RightDockWidgetArea, m_IMUDock);

    //--------------------------------------------------------
    //  setup dockable Controls gui
    //--------------------------------------------------------
    m_controlsDock->setAllowedAreas(Qt::LeftDockWidgetArea);
    addDockWidget(Qt::LeftDockWidgetArea, m_controlsDock);
    //--------------------------------------------------------
    //  setup dockable packet Stats gui
    //--------------------------------------------------------
    addDockWidget(Qt::LeftDockWidgetArea, m_StatsDock);
    //--------------------------------------------------------
    //  setup dockable ACK gui (this is not timer driven)
    //--------------------------------------------------------
   addDockWidget(Qt::LeftDockWidgetArea, m_ACKDock);
    //--------------------------------------------------------
    //  packetRateDock setup
    //--------------------------------------------------------
    m_packetRateDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    addDockWidget(Qt::BottomDockWidgetArea, m_packetRateDock);
}

//--------------------------------------------------------
// ConnectDocksViewerActions
// create all connections needed
//--------------------------------------------------------
void MainWindow::ConnectDocksViewerActions()
{
    //--------------------------------------------------------
    //  Controls gui
    //  This is the buttons dialog.
    //--------------------------------------------------------
    //added the controls button connections
    connect(m_controlsDock, &ControlsDock::L2connectRequested,
            this, &MainWindow::L2connect);

    connect(m_controlsDock, &ControlsDock::L2disconnectRequested,
            this, &MainWindow::L2disconnect);

    connect(m_controlsDock, &ControlsDock::ConfigRequested,
            this, &MainWindow::openConfig);

    connect(m_controlsDock, &ControlsDock::WorkmodeRequested,
            this, &MainWindow::openWorkmode);

    connect(m_controlsDock, &ControlsDock::startRotationRequested,
            this, &MainWindow::startRotation);

    connect(m_controlsDock, &ControlsDock::stopRotationRequested,
            this, &MainWindow::stopRotation);

    connect(m_controlsDock, &ControlsDock::L2resetRequested,
            this, &MainWindow::sendReset);

    connect(m_controlsDock, &ControlsDock::ClearPCwindowRequested,
            this, &MainWindow::ClearPCwindow);

    // dialogs connections (will open a dialog)
    connect(m_controlsDock, &ControlsDock::GetVersionRequested,
            this, &MainWindow::getVersion);

    // reset windows button in ControlsDock window
    connect(m_controlsDock, &ControlsDock::ResetWindowsRequested,
            this, &MainWindow::resetWindowLayout);

    connect(&WorkMode, &WorkmodeDialog::RequestSetL2workmode,
            this, &MainWindow::sendSetWorkmode);

    connect(&WorkMode, &WorkmodeDialog::RequestL2reset,
            this, &MainWindow::sendReset);

    //--------------------------------------------------------
    //  setup dockable ACK gui (this is not timer driven)
    //  It is event driven.  ACKs are extermely low rate events
    //--------------------------------------------------------
    connect(&l2lidar,
            &::L2lidar::ackReceived,
            this,
            &MainWindow::updateACK,
            Qt::QueuedConnection);

    //--------------------------------------------------------
    //  setup point cloud viewer GUI
    //--------------------------------------------------------
    connect(&l2lidar,
            &::L2lidar::PCL3DReceived,
            this,
            &MainWindow::onNew3DLidarFrame,
            Qt::QueuedConnection);

    connect(&l2lidar,
            &::L2lidar::PCL2DReceived,
            this,
            &MainWindow::onNew2DLidarFrame,
            Qt::QueuedConnection);

    //--------------------------------------------------------
    //  Workmode dialog
    //--------------------------------------------------------
    connect(&WorkMode, &QDialog::finished, this,
            &MainWindow::ClosedWorkmodeDialog);
   }

//--------------------------------------------------------
// applyDocksVisibilityConstraint
//--------------------------------------------------------
void MainWindow::applyDocksVisibilityConstraint()
{
    // set absolute geometry and state
    resizeDocks({ m_IMUDock },{ 220 },Qt::Horizontal);
    resizeDocks({ m_diagnosticsDock },{ 220 },Qt::Horizontal);
    resizeDocks({ m_StatsDock },{ 220 },Qt::Horizontal);

    m_controlsDock->setMinimumWidth(480);
    m_controlsDock->setMinimumHeight(200);
    m_controlsDock->setFeatures(QDockWidget::NoDockWidgetFeatures); // can not float or move
    // This would let it float or move but X will not close it
    // m_controlsDock->setFeatures(QDockWidget::DockWidgetMovable |
    //                             QDockWidget::DockWidgetFloatable);
    m_controlsDock->setContextMenuPolicy(Qt::PreventContextMenu); // do not allow context menu close
    ShowWindows();
}

//--------------------------------------------------------
// ShowWindows
//--------------------------------------------------------
void MainWindow::ShowWindows()
{
    m_diagnosticsDock->setVisible(config.isDiagEnabled());
    m_IMUDock->setVisible(config.isIMUenabled());
    m_ACKDock->setVisible(config.isACKenabled());
    m_packetRateDock->setVisible(config.isPacketRateChartEnabled());
    m_StatsDock->setVisible(config.isStatsEnabled());

    if(config.isPCviewerEnabled()) {
        if (!m_pointCloudWindow) return;
        m_pointCloudWindow->show();
        m_pointCloudWindow->raise();
    } else {
        if (!m_pointCloudWindow) return;
        m_pointCloudWindow->hide();
    }

    m_controlsDock->show(); // always show controls
    m_controlsDock->raise();
}

//--------------------------------------------------------
//  L2ConnectedButtonsUIs
//--------------------------------------------------------
void MainWindow::L2ConnectedButtonsUIs()
{ // set buttons and UIs states when L2 disconnected
    // disable start, enable stop
    m_controlsDock->setConnectState(true);

    mHeartBeat->start(); // for the stats windows

    StartPacketChart();
    StartPointCloudViewer();
}

//--------------------------------------------------------
//  L2ConnectedButtonsUIs
//--------------------------------------------------------
void MainWindow::L2DisconnectedButtonsUIs()
{ // set buttons and UIs states when L2 connected
    m_controlsDock->setConnectState(false);

    StopPointCloudViewer();
    StopPacketChart();

    // turn off docks
    mHeartBeat->stop();
}

//--------------------------------------------------------
//  StartPacketChart
//--------------------------------------------------------
void  MainWindow::StartPacketChart()
{
    m_packetRateDock->reset();
     mPacketBeat->start();
}

//--------------------------------------------------------
//  StopPacketChart
//--------------------------------------------------------
void  MainWindow::StopPacketChart()
{
    mPacketBeat->stop();
}

//--------------------------------------------------------
//  StartPointCloudViewer
//--------------------------------------------------------
void MainWindow::StartPointCloudViewer()
{
    RendererTimer->start();
}

//--------------------------------------------------------
//  StopPointCloudViewer
//--------------------------------------------------------
void MainWindow::StopPointCloudViewer()
{
    RendererTimer->stop();
}

//========================================================
//
//  Dockable windows
//
//  timer driven separate GUI windows from main GUI window
//
//========================================================

void MainWindow::HeartbeatFire()
{
    updateDiagnostics();
    updateIMU();
    updateStats();
    return;
}

//--------------------------------------------------------
//  updateStats
//--------------------------------------------------------
void MainWindow::updateStats()
{
    // update detailed packet stats
    PacketStats Stats;
    Stats.countPackets = l2lidar.totalPackets();
    Stats.lostPackets = l2lidar.lostPackets();
    Stats.count3DPCL = l2lidar.total3D();
    Stats.count2DPCL = l2lidar.total2D();
    Stats.countIMU = l2lidar.totalIMU();
    Stats.countACK = l2lidar.totalACK();
    Stats.countOther = l2lidar.totalOther();
    LidarTimeStampData TimeStamp = l2lidar.timestamp();
    Stats.TimeSec = TimeStamp.data.sec;
    Stats.TimeNsec = TimeStamp.data.nsec;
    m_StatsDock->updateStats(Stats);

    return;
}

//--------------------------------------------------------
//
// timer driven update of Packet Rate dock
//
//--------------------------------------------------------
void MainWindow::updatePacketRate()
{
    const qint64 elapsedMs = m_rateTimer->elapsed();
    if (elapsedMs < 100)
        return;

    const uint64_t total = l2lidar.totalPackets();
    const uint64_t deltaPackets = total - m_lastPacketCount;

    const double rate =
        (deltaPackets * 1000.0) / static_cast<double>(elapsedMs);

    m_packetRateDock->addSample(rate); // add time, sample rate

    m_lastPacketCount = total;
    m_rateTimer->restart();
}

//--------------------------------------------------------
//  updateDiagnostics
//--------------------------------------------------------
void MainWindow::updateDiagnostics()
{
    LidarVersionData Version = l2lidar.version();
    if(mLastTypePacketReceived) {
        LidarPointDataPacket PCLpacket = l2lidar.Pcl3Dpacket();
        m_diagnosticsDock->updateDiagnostics(PCLpacket.data.state, PCLpacket.data.param);
   } else {
        Lidar2DPointDataPacket PCLpacket = l2lidar.Pcl2Dpacket();
        m_diagnosticsDock->updateDiagnostics(PCLpacket.data.state, PCLpacket.data.param);
   }

    m_diagnosticsDock->updateVersion(Version);

    return;
}

//--------------------------------------------------------
//  updateIMU
//--------------------------------------------------------
void MainWindow::updateIMU()
{
    LidarImuData Imu = l2lidar.imu();


    m_IMUDock->updateIMU(Imu);

    return;
}

//--------------------------------------------------------
//  updateACK, event driven not timer driven
//--------------------------------------------------------
void MainWindow::updateACK()
{
    LidarAckData ACKdata = l2lidar.ack();
    m_ACKDock->updateACK(ACKdata);

    return;
}

//========================================================
//
//  Point cloud viewer data generator
//
//  timer driven separate GUI window from main GUI window
//
//========================================================

//--------------------------------------------------------
//  onNewLidarFrame()
//  signal recieved from l2lidar class that a new frame
//  of point cloud data is availabe
//  This removes the oldest frame from the fifo if the fifo
//  is full and and adds the new frame to the fifo
//  for display
//
//  This is updated at the packet receive rate
//
//  The Point cloud viewer architecture changed allows means
//  it doesn't need to have any awareness of the frame or
//  frame size.
//
//  The frame is converted to a point cloud format and then
//  appended to the point cloud display.
//
//--------------------------------------------------------
void MainWindow::onNewLidarFrame(bool Frame3D)
{
    // skip packet logic to reduce load
    // skip 0 take severy packet
    // skip 1 takes every other packet
    // skip 2 takes every 3rd packet
    // ...
    // ...
    static uint32_t frameCounter {0};
   bool adjustWithIMU {false};
    double time;
    LidarImuData Imu;
    Quaternion Quat;

    if (NumFramesToSkip > 0 &&
        (++frameCounter % (NumFramesToSkip + 1)) != 0)
    {
        return;
    }

    // Retrieve packet
    unilidar_sdk2::PointCloudUnitree cloud;

    if(Frame3D) {
        // 3D packet
        LidarPointDataPacket packet = l2lidar.Pcl3Dpacket();
        unilidar_sdk2::parseFromPacketToPointCloud(
                    cloud, packet, false, 0, 100);
        time = (double)packet.data.info.stamp.sec + (double)packet.data.info.stamp.nsec * 1.0e-9;
    } else {
        // 2D packet
        Lidar2DPointDataPacket packet = l2lidar.Pcl2Dpacket();
        unilidar_sdk2::parseFromPacketPointCloud2D(
                    cloud, packet, false, 0, 100);
        time = (double)packet.data.info.stamp.sec + (double)packet.data.info.stamp.nsec * 1.0e-9;
    }

    if(mIMUadjust) {
        // check if latest IMU packet is within 10 msec
        double IMUtime;
        Imu = l2lidar.imu();
        IMUtime = (double)Imu.info.stamp.sec +(double)Imu.info.stamp.nsec * 1e-9;
        if(abs(time-IMUtime) < .010) {
            adjustWithIMU = true;
            Quat.w = Imu.quaternion[0];
            Quat.x = Imu.quaternion[1];
            Quat.y = Imu.quaternion[2];
            Quat.z = Imu.quaternion[3];
        } else {
            adjustWithIMU = false;
        }
    }

    Frame frame;
    frame.reserve(cloud.points.size());

    for (auto& p : cloud.points)
    {
        if(adjustWithIMU) {
            rotateByQuaternion(Quat,p.x,p.y,p.z);
        }

        frame.push_back({
            p.x,
            p.y,
            p.z,
            p.intensity,
            p.time,
            p.ring
        });
    }


    // QMutexLocker lock(&m_cloudMutex);

    if (m_pointCloudWindow)
    {
        QMetaObject::invokeMethod(
            m_pointCloudWindow,
            [this, frame]() {
                m_pointCloudWindow->appendFrame(frame);
            },
            Qt::QueuedConnection
            );
    }
}

//--------------------------------------------------------
//
//  GUI mainwindow
//  button presses
//
//--------------------------------------------------------

//--------------------------------------------------------
//  openWorkmode
//--------------------------------------------------------
void MainWindow::openWorkmode()
{
    // WorkMode.exec(); // modal dialog
    // WorkMode.GetWorkmode();

    WorkMode.show(); // non-modal dialog
    // non-modal dialog needs to use slot
    // that gets signals when closed
}

//--------------------------------------------------------
//  ClosedWorkmodeDialog
//--------------------------------------------------------
void MainWindow::ClosedWorkmodeDialog()
{
    WorkMode.GetWorkmode();
    // This does not send a set workmode command is saves the
    // current settings for the workmode dialog
}

//--------------------------------------------------------
//  openConfig
//  button press
//
//  open the configuration dialog to allow the user to change
//  application settings.
//--------------------------------------------------------
void MainWindow::openConfig()
{
    // make sure requested reset is not cleared
    bool ResetRequested = GetSettingsReset();
    if(ResetRequested) {
        QMessageBox msgBox;
        msgBox.setText("Reset of GUI requeseted");
        msgBox.setInformativeText("exit and restart");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    float PointSizeRange[2];
    m_pointCloudWindow->getPointSizeRange((PointSizeRange));
    config.setPointSizeRange(PointSizeRange);

    if (config.exec() == QDialog::Accepted) {
        // update the L2 UDP connection
        l2lidar.LidarSetCmdConfig(config.getSRCip(),config.getSRCport(),
                                  config.getDSTip(),config.getDSTport());

        // update frames to skip
        NumFramesToSkip = config.getSkipFrame();

        // check if buffering has changed
        if(mmaxPoints!=config.getMaxPoints()) {
            //ask user if they really want to changes settings
            QMessageBox msgBox;
            msgBox.setText("Critical buffer size changed\nThe app must exit\nAre you sure you want to proceed?");
            msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            msgBox.setDefaultButton(QMessageBox::No);
            // Show the dialog and wait for user response
            int ret = msgBox.exec();
            if (ret == QMessageBox::Yes) {
                // User clicked Yes
                mmaxPoints = config.getMaxPoints();
                saveSettings(false); // do not reset window geometries
                QApplication::quit();
                return;
            } else {
                // User clicked No
                RestoreConfigSettings();
                return;
            }
        }

        if(m_controlsDock->GetConnectedState()) {
            if(config.getDiagUpdateRate()!=mHeartBeat->interval() ||
                config.getPacketUpdateRate()!=mPacketBeat->interval() ||
                config.getRenderRate()!=RendererTimer->interval() ){

                // of any timer changes then stop all, reset all, restart
                L2DisconnectedButtonsUIs(); // this stops everything
                mHeartBeat->setInterval(config.getDiagUpdateRate());
                mPacketBeat->setInterval(config.getPacketUpdateRate());
                RendererTimer->setInterval(config.getRenderRate());
                L2ConnectedButtonsUIs(); // this restarts everything
            }
        }

        // update PC window settings
        PCsettings CurrentPC;
        m_pointCloudWindow->getPCsettings(CurrentPC);
        CurrentPC.MinDistance = config.getMinDistance();
        CurrentPC.MaxDistance = config.getMaxDistance();
        CurrentPC.PointSize = config.getPointSize();
        m_pointCloudWindow->setPCsettings(CurrentPC);


        // Save current user settings
        saveSettings(false); // do not reset window geometries
        SetDefaultView(); // saves the default point cloud view
        ShowWindows(); // update window visibility

    } else {
        // reset the point cloud view back to defaults
        RestoreConfigSettings();
    }
}

//--------------------------------------------------------
//  RestoreConfigSettings
//  when Config dialog is cancelled
//--------------------------------------------------------
void MainWindow::RestoreConfigSettings()
{
    // reset the point cloud view back to defaults
    config.setPCWdistance(defaultPCsettings.Distance);
    config.setPCWyaw(defaultPCsettings.Yaw);
    config.setPCWpitch(defaultPCsettings.Pitch);
    config.setPointSize(defaultPCsettings.PointSize);
    config.setMinDistance(defaultPCsettings.MinDistance);
    config.setMaxDistance(defaultPCsettings.MaxDistance);
    // reset the point cloud buffering back to current setting
    config.setMaxPoints(mmaxPoints);
}

//--------------------------------------------------------
//  L2connect()
//  button press
//--------------------------------------------------------
void MainWindow::L2connect()
{
    //
    if(!l2lidar.ConnectL2()) {
        QMessageBox msgBox;
        msgBox.setText("Connect to L2 LiDAR");
        msgBox.setInformativeText("Could not open");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    m_lastPacketCount = l2lidar.totalPackets();
    m_rateTimer->restart(); // elapsed timer
    mPacketBeat->start();

    L2ConnectedButtonsUIs();
}

//--------------------------------------------------------
//  L2disconnect()
//  button press
//--------------------------------------------------------
void MainWindow::L2disconnect()
{
    // close UDP connection for receiving data
    l2lidar.DisconnectL2();

    l2lidar.ClearCounts();
    //recentRates.clear(); // clear the rate history

    L2DisconnectedButtonsUIs();
}

//--------------------------------------------------------
//  startRotation
//  button press
//--------------------------------------------------------
void MainWindow::startRotation()
{
    l2lidar.LidarStartRotation();
    return;
}

//--------------------------------------------------------
//  stopRotation
//  button press
//--------------------------------------------------------
void MainWindow::stopRotation()
{
    l2lidar.LidarStopRotation();
    return;
}

//--------------------------------------------------------
//  sendReset
//  button press
//--------------------------------------------------------
void MainWindow::sendReset()
{
    l2lidar.LidarReset();
    return;
}

//--------------------------------------------------------
//  sendSetWorkmode
//  button press
//--------------------------------------------------------
void MainWindow::sendSetWorkmode()
{
    uint32_t workmode;
    workmode = WorkMode.GetWorkmode();
    l2lidar.SetWorkMode(workmode);
    return;
}

void MainWindow::ClearPCwindow()
{
    m_pointCloudWindow->clearPointCloud();
}

//--------------------------------------------------------
//  getVersion
//  button press
//--------------------------------------------------------
void MainWindow::getVersion()
{
    l2lidar.LidarGetVersion();
    return;
}

//--------------------------------------------------------
//  resetWindowLayout
//  request windows layout reset
//--------------------------------------------------------
void MainWindow::resetWindowLayout()
{
    SetSettingsReset(true);
}


