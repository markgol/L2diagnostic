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
//  of packets and optionally saves them to a CSV file.
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

//--------------------------------------------------------
//  Data flow for point cloud
//      MainWindow:onNewLidarFrame()  this operates at L2 point cloud
//          |              packet rate ~200-250 packets/sec
//          |
//      throttling         fifo, save only every nth PCpoint
//          |
//      MainWindow:updatePointCloud() create flattened point cloud
//          |              cloudTimer Timer driven
//          |
//      MainWindow:flattenedCloudReady() signals next step
//          |
//      PointCloudWindow::setPointCloud()
//          |              push flattened cloud to viewer
//          |
//      PointCloudWindow::update()
//                         repaint window
//                         renderTimer Timer driven
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
// #include "ConfigDialog.h"

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

    // create cloud viewer window
    // This is not a Qt window but a OpenGL managed window
    m_pointCloudWindow = new PointCloudWindow();
    m_pointCloudWindow->setTransientParent(windowHandle());
    m_pointCloudWindow->setFlag(Qt::Window);

    // Load previously saved user settings
    loadSettings(GetSettingsReset());
    SetSettingsReset(false);

    SetupGUIrefreshTimers();
    ConnectDocksViewerActions();

    // these are only done after loadsettings()
    applyDocksVisibilityConstraint();

    // 3D point cloud viewer buffering
    m_frameRing.resize(MAX_FRAMES); // ??? ring buffer pre allocated
    NumFramesToSkip = config.getSkipFrame();

    // load the com parameters in the l2lidar class
    l2lidar.LidarSetCmdConfig(config.getSRCip(),config.getSRCport(),
                              config.getDSTip(),config.getDSTport());

    // initial state of buttons and uis is L2 disconnected
    L2DisconnectedButtonsUIs(); // set buttons and UIs states when L2 disconnected

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
// SetupGUIrefreshTimers
//--------------------------------------------------------
void MainWindow::SetupGUIrefreshTimers()
{
    //--------------------------------------------------------
    //  This timer is used to trigger updates
    //  for Diagnostics, IMU and Stats
    //--------------------------------------------------------
    mHeartBeat = new QTimer(this);
    mHeartBeat->setInterval(config.getDiagUpdateRate()); // suggest 4 Hz
    connect(mHeartBeat, &QTimer::timeout, this, &MainWindow::HeartbeatFire);

    //--------------------------------------------------------
    //  These timers are used to trigger updates
    //  for packet rate chart update
    //--------------------------------------------------------
    mPacketBeat = new QTimer(this);
    mPacketBeat->setInterval(config.getPacketUpdateRate()); // suggest 10 Hz
    connect(mPacketBeat, &QTimer::timeout, this, &MainWindow::updatePacketRate);
    // The elpased timer is used for calculating the packet rate/sec
    // It does not tigger and signals
    m_rateTimer = new QElapsedTimer;
    m_rateTimer->restart();

    //--------------------------------------------------------
    // setup timer for point cloud updating
    //--------------------------------------------------------
    cloudTimer = new QTimer(this);
    cloudTimer->setInterval(config.getPCupdateRate()); // suggest 60 Hz
    connect(cloudTimer,
            &QTimer::timeout,
            this,
            &MainWindow::updatePointCloud);
    //--------------------------------------------------------
    // setup timer point cloud rendereing
    //--------------------------------------------------------
    // ???
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
    QMainWindow::closeEvent(e);
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
    //added the seven button connections
    connect(m_controlsDock, &ControlsDock::L2connectRequested,
            this, &MainWindow::L2connect);

    connect(m_controlsDock, &ControlsDock::L2disconnectRequested,
            this, &MainWindow::L2disconnect);

    connect(m_controlsDock, &ControlsDock::ConfigRequested,
            this, &MainWindow::openConfig);

    connect(m_controlsDock, &ControlsDock::startRotationRequested,
            this, &MainWindow::startRotation);

    connect(m_controlsDock, &ControlsDock::stopRotationRequested,
            this, &MainWindow::stopRotation);

    connect(m_controlsDock, &ControlsDock::L2resetRequested,
            this, &MainWindow::sendReset);

    connect(m_controlsDock, &ControlsDock::GetVersionRequested,
            this, &MainWindow::getVersion);

    connect(m_controlsDock, &ControlsDock::ResetWindowsRequested,
            this, &MainWindow::resetWindowLayout);

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
    // point cloud viewer connections
    connect(&l2lidar,
            &::L2lidar::PCL3DReceived,
            this,
            &MainWindow::onNewLidarFrame,
            Qt::QueuedConnection);

    connect(this,
            &MainWindow::flattenedCloudReady,
            m_pointCloudWindow,
            &PointCloudWindow::setPointCloud,
            Qt::QueuedConnection);
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

    m_controlsDock->setMinimumWidth(440);
    m_controlsDock->setMinimumHeight(160);

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
}

//--------------------------------------------------------
//  L2ConnectedButtonsUIs
//--------------------------------------------------------
void MainWindow::L2ConnectedButtonsUIs()
{ // set buttons and UIs states when L2 disconnected
    // disable start, enable stop
    m_controlsDock->setConnectState(true);

    mHeartBeat->start();

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
    cloudTimer->start();
}

//--------------------------------------------------------
//  StopPointCloudViewer
//--------------------------------------------------------
void MainWindow::StopPointCloudViewer()
{
    cloudTimer->stop();
}

//--------------------------------------------------------
//
//  Dockable windows
//
//  timer driven separate GUI windows from main GUI window
//
//--------------------------------------------------------

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

    m_packetRateDock->addSample(rate);

    m_lastPacketCount = total;
    m_rateTimer->restart();
}

//--------------------------------------------------------
//  updateDiagnostics
//--------------------------------------------------------
void MainWindow::updateDiagnostics()
{
    LidarVersionData Version = l2lidar.version();
    LidarPointDataPacket PCLpacket = l2lidar.Pcl3Dpacket();

    m_diagnosticsDock->updateDiagnostics(PCLpacket.data.state, PCLpacket.data.param);
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

//--------------------------------------------------------
//
//  Point cloud viewer data generator
//
//  timer driven separate GUI window from main GUI window
//
//--------------------------------------------------------

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
//--------------------------------------------------------
void MainWindow::onNewLidarFrame()
{
    // skip packet logic to reduce load
    // skip 0 take severy packet
    // skip 1 takes every other packet
    // skip 2 takes every 3rd packet
    // ...
    // ...
    static uint64_t frameCounter = 0;

    if (NumFramesToSkip > 0 &&
        (++frameCounter % (NumFramesToSkip + 1)) != 0)
    {
        return;
    }

    // Retrieve packet
    auto packet = l2lidar.Pcl3Dpacket();

    unilidar_sdk2::PointCloudUnitree cloud;
    unilidar_sdk2::parseFromPacketToPointCloud(
        cloud, packet, false, 0, 100);

    Frame frame;
    frame.reserve(cloud.points.size());

    for (const auto& p : cloud.points)
    {
        frame.push_back({
            p.x,
            p.y,
            p.z,
            p.intensity,
            p.time,
            p.ring
        });
    }


    QMutexLocker lock(&m_cloudMutex);

    // Overwrite oldest frame in-place
    m_frameRing[m_ringWrite] = std::move(frame);

    m_ringWrite = (m_ringWrite + 1) % MAX_FRAMES;
    if (m_ringCount < MAX_FRAMES)
        ++m_ringCount;
}

//--------------------------------------------------------
//  updatePointCloud()
//  timer driven emitter for point cloud viewer
//  The acculated frames are flattened and sent
//  to the viewer at the viewer display rate
//  This timer set and started in L2connect()
//--------------------------------------------------------
void MainWindow::updatePointCloud()
{
    auto cloud = buildFlattenedCloud();
    if (!cloud.isEmpty())
        emit flattenedCloudReady(cloud);
}


//--------------------------------------------------------
//  buildFlattenedCloud()
//  help function that converts the frame fifo
//  into a flattened point cloud array
//--------------------------------------------------------
QVector<PCpoint> MainWindow::buildFlattenedCloud()
{
    QVector<Frame> localFrames;

    {
        QMutexLocker lock(&m_cloudMutex);

        if (m_ringCount == 0)
            return {};

        const size_t oldest =
            (m_ringWrite + MAX_FRAMES - m_ringCount) % MAX_FRAMES;

        localFrames.reserve(m_ringCount);
        for (size_t i = 0; i < m_ringCount; ++i) {
            size_t idx = (oldest + i) % MAX_FRAMES;
            localFrames.push_back(m_frameRing[idx]); // shallow copy
        }
    } // mutex released here

    // Now flatten WITHOUT holding the mutex
    QVector<PCpoint> PCcloud;
    size_t totalPoints = 0;
    for (const auto& f : localFrames)
        totalPoints += f.size();

    PCcloud.reserve(totalPoints);
    for (const auto& f : localFrames)
        PCcloud += f;

    return PCcloud;
}

//--------------------------------------------------------
//
//  GUI mainwindow
//  button presses
//
//--------------------------------------------------------

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

    if (config.exec() == QDialog::Accepted) {
        l2lidar.LidarSetCmdConfig(config.getSRCip(),config.getSRCport(),
                                  config.getDSTip(),config.getDSTport());
        NumFramesToSkip = config.getSkipFrame();
        // Save current user settings
        saveSettings(false); // do not reset

        ShowWindows(); // update window visibility
    }
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


