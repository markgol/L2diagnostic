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
// V0.3.7   2026-01-26  Added ConfigureUPD button
//                      Added measure latency button
// V0.3.9   2026-01-30  Removed measure latency button
//                      Added Sync L2 timestamp button
//                      Added SAVE/LOAD point cloud
// V0.3.10  2026-02-01  Added Get L2 params button
//                      Added Get L2 workmode button
//                      Updated all uses of "unitree_lidar_protocolL2.h"
//                      to esnure that the pragma pack(push,1) is used
//                      so that all packets deinfitions are byte packed
// V0.3.11  2026-02-4   Moved point clouud conversion to l2lidar class
//                      Only open point cloud window when first enabled
//                      Adjusted sizing of ControlsDock and ConfigDialog
//                          to adjust for use on Ubuntu x64 and ARM64 platforms
// V0.3.12  2026-02-05  Moved renderer timer to PointCloudWindow class
// V0.4.0   2026-02-06  Added initialization of OpenGL
//                      If no cmd line arguments are present then
//                          OpenGL Core 3.3 is used.
//                      if one command line argument is present then
//                          OpenGLES V3.0 is used
//                      if 2 or more command line arguement then no graphics
// V0.4.3   2026-02-19  Added point cloud frame aggregation
//                          This only only applies to 3D point cloud data
//                          Aggregates up to 'n' frames
//                          Aggregation requires enableL2TimeStampFix and
//                              mL2EnableSyncHost to be true
//                          Aggregation uses first scan time as base time
//                              and all cloud points relative to that time.
//                          The aggregation process is used here to show
//                              correct implementation for use in a ROS2
//                              publisher node that needs to publish
//                              aggregated frames to be compatible with
//                              LIO-SAM and Fast-LIO processing
//                      Refactored folders/files to folder L2lidarClass
//                          to be able to split repo so L2Lidar class has
//                          its own repo
//                      Added 'n' frame aggregation for point cloud frame
//                      0 is no aggregation, 38 matches one hemishpere scan
//                      Moved L2lidar class into its own release
//  V0.4.4  2026-02-22  Changes in L2lidar class propogated into onNewLidarFrame()
//                      and the point cloud data
//                      These changes relate to the point timestamps
//                      to prevent truncation error because of lack of
//                      precision using doubles and float in time calculations
//                      Added stats to the IMU variables
//  V1.0.0  2026-03-28  Offical release
//  V1.1.0  2026-04-20  Added override of range calibration params
//  V1.1.1  2026-04-29  Added CloudCompare PCD compatible output file
//                      with just x,y,z,intensity,range
//  V1.2.0  2026-05-12  Updated L2lidarClass for more precise timestamp calculations
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
//#include <QDebug>
#include <QMessageBox>
//--------------------------------------------------------
//  Project specific includes not part of MainWindow.h
//--------------------------------------------------------

//--------------------------------------------------------
//  MainWIndow class constructor
//--------------------------------------------------------
MainWindow::MainWindow(bool OpenGLES, int major, int minor,QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
    mOpenGLES(OpenGLES),
    mOpenGLmajorV(major),
    mOpenGLminorV(minor)

{
    ui->setupUi(this);

    createDocksViewer();
    AssignDocksObjectNames();
    AddDocksViewer();

    // Load previously saved user settings
    loadSettings(GetSettingsReset());
    SetSettingsReset(false);

    if(!mOpenGLES && mOpenGLmajorV==0 && mOpenGLminorV==0) {
        mNoGraphics = true;
    }

    if(mNoGraphics) {
        config.setPCviewerEnabled(false);
        config.setPacketRateChartEnabled(false);
    }

    // create cloud viewer window
    // This is not a Qt window but a OpenGL managed window
    // Do this immeditaely if point cloud window enabled
    // otherwise delay it until window is enabled for the first time.
    if(config.isPCviewerEnabled()) {
        if(mmaxPoints>=7200) {
            if(OpenPointCloudWindow()) {
                m_pointCloudWindow->RendererTimerStart();
            } else {
                // opnGL is not available
                config.setPCviewerEnabled(false);
            }
        }
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

    // connect config request from Set L2 UDP config button
    connect(&config, &ConfigDialog::requestConfigureUDP,
            this, &MainWindow::handleConfigureUDP);

    // connect config request Get L2 UDP config button
    connect(&config, &ConfigDialog::requestSetL2MAC,
            this, &MainWindow::handleSetL2MAC);

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
// OpenPointCloudWind
//--------------------------------------------------------
bool MainWindow::OpenPointCloudWindow()
{
    if(mNoGraphics) return false;

    // prerequesite for application
    // point cloud window requires OpenGLES V3.x
    // or Open Core 3.3
    // Check against version requested at startup
    if(!mOpenGLES &&  (mOpenGLmajorV!=3 || mOpenGLminorV!=3)) {
        return false;
    } else {
        if(mOpenGLES && mOpenGLmajorV!=3)
            return false;
    }

    m_pointCloudWindow = new PointCloudWindow(mmaxPoints,mOpenGLES);
    // indicates problem or wrong version of OpenGL
    if(m_pointCloudWindow->PointCouldInitFailed()) {
        delete m_pointCloudWindow;
        m_pointCloudWindow = nullptr;
        return false;
    }

    // set default view settings
    SetDefaultView();
    m_pointCloudWindow->setTransientParent(windowHandle());
    m_pointCloudWindow->Initialize();
    m_pointCloudWindow->InitializeRenderTimer(config.getRenderRate());

    return true;
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
    if(m_pointCloudWindow!=nullptr){
        m_pointCloudWindow->setDefaultPCsettings(defaultPCsettings);
    }
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
    //  only for graphics enableds
    //--------------------------------------------------------
    mPacketBeat = new QTimer(this);
    mPacketBeat->setInterval(config.getPacketUpdateRate());
    connect(mPacketBeat, &QTimer::timeout, this, &MainWindow::updatePacketRate);
    // The elpased timer is used for calculating the packet rate/sec
    // It does not tigger and signals
    m_rateTimer = new QElapsedTimer;
    m_rateTimer->restart();
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
    if(!mNoGraphics) {
        m_packetRateDock = new PacketRateDock(this);
    }
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
//  handleConfigureUDP from config dialog Configure UDP button
//  This requires a power cycle of the L2
//--------------------------------------------------------
void MainWindow::handleConfigureUDP()
{
    QString hostIP = config.getSRCip();
    uint32_t hostPort = config.getSRCport();
    QString LidarIP = config.getDSTip();
    uint32_t LidarPort = config.getDSTport();

    // convert IP string to numbers
    int L2ip[4] {0,0,0,0};
    int Hip[4] {0,0,0,0};

    // make sure valid ip
    QStringList parts = hostIP.split('.');
    bool result;

    L2ip[0] = parts[0].toUInt(&result);
    if(!result) return;
    if(L2ip[0]<0 || L2ip[0]>255) return;

    L2ip[1] = parts[1].toUInt(&result);
    if(!result) return;
    if(L2ip[1]<0 || L2ip[1]>255) return;

    L2ip[2] = parts[2].toUInt(&result);
    if(!result) return;
    if(L2ip[2]<0 || L2ip[2]>255) return;

    L2ip[3] = parts[3].toUInt(&result);
    if(!result) return;
    if(L2ip[3]<0 || L2ip[3]>255) return;

    parts = LidarIP.split('.');
    Hip[0] = parts[0].toUInt(&result);
    if(!result) return;
    if(Hip[0]<0 || Hip[0]>255) return;

    Hip[1] = parts[1].toUInt(&result);
    if(!result) return;
    if(Hip[1]<0 || Hip[1]>255) return;

    Hip[2] = parts[2].toUInt(&result);
    if(!result) return;
    if(Hip[2]<0 || Hip[2]>255) return;

    Hip[3] = parts[3].toUInt(&result);
    if(!result) return;
    if(Hip[3]<0 || Hip[3]>255) return;

    if(l2lidar.setL2UDPconfig(hostIP, hostPort, LidarIP, LidarPort)) {
        QMessageBox msgBox;
        msgBox.setText("Restart application");
        msgBox.setInformativeText("L2 must be powered cycle and app restarted");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    } else {
        QMessageBox msgBox;
        msgBox.setText("command failed");
        msgBox.setInformativeText("L2 is not turned on or not connected");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

//--------------------------------------------------------
//  handleConfigureUDP from config dialog Configure UDP button
//  This requires a power cycle of the L2
//--------------------------------------------------------
void MainWindow::handleSetL2MAC()
{
    LidarMacAddressConfig MACconfig {};
    QString MACstring;
    QByteArray MACid;
    MACstring = config.GetMAC();

    MACid = convertMacStringToByteArray(MACstring);
    if(MACid.size()!=6) {
        // bad format
        QMessageBox msgBox;
        msgBox.setText("MAC address NOT SET");
        msgBox.setInformativeText("This is not a MAC address\nformat should 6 hex values separated\nby :");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    if(!(MACid[0] & 0x02) || (MACid[0] & 0x01)) {
        // not a locally managed and unicast cast MAC ID
        QMessageBox msgBox;

        msgBox.setWindowTitle("Confirmation required");
        msgBox.setText("First hex value bit 0x02 must be 1 for locally managed MAC\nFirst hex value bit 0x01 must be 0 for unicast device\nDo you really want to proceed?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);

        // Execute the message box and capture the user's response
        int ret = msgBox.exec();

        if(ret == QMessageBox::No ) {
            return;
        }
    }

    MACconfig.mac[0] = MACid[0];
    MACconfig.mac[1] = MACid[1];
    MACconfig.mac[2] = MACid[2];
    MACconfig.mac[3] = MACid[3];
    MACconfig.mac[4] = MACid[4];
    MACconfig.mac[5] = MACid[5];
    MACconfig.reserve[0] = 0;
    MACconfig.reserve[1] = 0;

    if(!l2lidar.SetL2MAC(MACconfig)){
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Set MAC failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }

    QMessageBox msgBox;
    msgBox.setWindowTitle("Settting MAC");
    msgBox.setText("This requires a rest or power cycle to take effect");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
}

//--------------------------------------------------------
// convertMacStringToByteArray
// helper function
//--------------------------------------------------------
QByteArray MainWindow::convertMacStringToByteArray(const QString &macString) {
    QByteArray byteArray;
    QStringList macParts = macString.split(':');

    for (const QString &part : macParts) {
        bool ok;
        byteArray.append(part.toUInt(&ok, 16)); // Convert hex string to byte
        if (!ok) {
            //qWarning() << "Invalid MAC address part:" << part;
            return QByteArray(); // Return empty if conversion fails
        }
    }

    return byteArray;
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
    if(m_packetRateDock!=nullptr) {
        m_packetRateDock->setObjectName("PacketRateDock");
    }
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
    if(m_packetRateDock!=nullptr){
        m_packetRateDock->setAllowedAreas(Qt::BottomDockWidgetArea);
        addDockWidget(Qt::BottomDockWidgetArea, m_packetRateDock);
   }
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

    connect(m_controlsDock, &ControlsDock::SyncL2CLock,
            this, &MainWindow::SyncL2Clock);

    // dialogs connections (will open a dialog)
    connect(m_controlsDock, &ControlsDock::GetVersionRequested,
            this, &MainWindow::getVersion);

    // Save PC button in ControlsDock window
    connect(m_controlsDock, &ControlsDock::SavePC,
            this, &MainWindow::SavePC);

    // Load PC button in ControlsDock window
    connect(m_controlsDock, &ControlsDock::LoadPC,
            this, &MainWindow::LoadPC);

    // Save PC button in ControlsDock window
    connect(m_controlsDock, &ControlsDock::ResetWindowsRequested,
            this, &MainWindow::resetWindowLayout);

    // Get L2 Params button in ControlsDock window
    connect(m_controlsDock, &ControlsDock::GetL2Params,
            this, &MainWindow::GetL2Params);

    // workmode dialog connects
    connect(&WorkMode, &WorkmodeDialog::RequestSetL2workmode,
            this, &MainWindow::sendSetWorkmode);

    connect(&WorkMode, &WorkmodeDialog::RequestL2reset,
            this, &MainWindow::sendReset);

    connect(&WorkMode, &WorkmodeDialog::RequestGetL2Workmode,
            this, &MainWindow::sendGetL2Workmode);

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

    connect(&l2lidar,
            &::L2lidar::WorkmodeReceived,
            this,
            &MainWindow::GetL2workmode,
            Qt::QueuedConnection);

    connect(&l2lidar,
            &::L2lidar::imuReceived,
            this,
            &MainWindow::onNewLidarIMU,
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

    m_controlsDock->setMinimumWidth(500);
    m_controlsDock->setMinimumHeight(220);
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
    if(m_packetRateDock!=nullptr) {
        m_packetRateDock->setVisible(config.isPacketRateChartEnabled());
    }
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
{ // set buttons and UIs states when L2 connected
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
{ // set buttons and UIs states when L2 disconnected
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
    if(m_packetRateDock==nullptr) return;

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
    if(m_pointCloudWindow==nullptr)
        return;
    m_pointCloudWindow->RendererTimerStart();
}

//--------------------------------------------------------
//  StopPointCloudViewer
//--------------------------------------------------------
void MainWindow::StopPointCloudViewer()
{
    if(m_pointCloudWindow==nullptr)
        return;

    m_pointCloudWindow->RendererTimerStop();
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
    Stats.PacketRate = mPacketRate;
    LidarTimeStampData timestamp = l2lidar.timestamp();
    Stats.TimeSec = timestamp.data.sec;
    Stats.TimeNsec = timestamp.data.nsec;
    TimeStamp Now;
    unilidar_sdk2::getSystemTimeStamp(Now);
    m_StatsDock->updateStats(Stats,Now);

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

    if(m_packetRateDock!=nullptr) {
        m_packetRateDock->addSample(rate); // add time, sample rate
    }
    mPacketRate = (float)rate;
    m_rateTimer->restart();
    m_lastPacketCount = total;
}

//--------------------------------------------------------
//  updateDiagnostics
//--------------------------------------------------------
void MainWindow::updateDiagnostics()
{
    LidarVersionData Version = l2lidar.version();
    Latency LatestLatency = l2lidar.GetLatency();

    if(mLastTypePacketReceived) {
        LidarPointDataPacket PCLpacket = l2lidar.Pcl3Dpacket();
        m_diagnosticsDock->updateDiagnostics(PCLpacket.data.state, PCLpacket.data.param,
                PCLpacket.data.range_min, PCLpacket.data.range_max,
                PCLpacket.data.info.seq,
                LatestLatency);
   } else {
        Lidar2DPointDataPacket PCLpacket = l2lidar.Pcl2Dpacket();
        m_diagnosticsDock->updateDiagnostics(PCLpacket.data.state, PCLpacket.data.param,
                PCLpacket.data.range_min, PCLpacket.data.range_max,
                PCLpacket.data.info.seq,
                LatestLatency);
   }

    m_diagnosticsDock->updateVersion(Version);

    return;
}

//--------------------------------------------------------
//  updateIMU
//--------------------------------------------------------
void MainWindow::updateIMU()
{
    if(!enableIMUstats) {
        LidarImuDataPacket Imu = l2lidar.imu();
        m_IMUDock->updateIMU(Imu.data);
    } else {
        // stats generated in OnNewLidarIMU()
        m_IMUDock->updateIMU(ImuStats);
    }

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
//  This includes a demonstration of frame aggregation for 3D
//  point cloud frames.
//  Requirements:
//      The l2lidar settings for
//          enableSynHost = true,
//          enableTScorrection = true
//      NumFramesToSkip must be 0
//      This only applies to 3D point cloud data
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

    if (NumFramesToSkip > 0 &&
        (++frameCounter % (NumFramesToSkip + 1)) != 0)
    {
        return;
    }


    if(NumFramesToSkip>0 || NumFramestoAggregate==0 || !Frame3D) {
        // no aggregation, basic per frame update of point cloud
        Frame frame;
        // convert latestL2 point cloud packet to Frame of cloud points
        if(!l2lidar.ConvertL2data2pointcloud(frame, Frame3D, mIMUadjust,
                                            mCalOveride, mCalScale, mCalBias)) {
            // if packet is missing or IMU pose correction failed
            // with mIMUadjust is true
            // do not add to point cloud
            return;
        }
        if (m_pointCloudWindow) {
            QMetaObject::invokeMethod(
                m_pointCloudWindow,
                [this, frame]() {
                    m_pointCloudWindow->appendFrame(frame);
                },
                Qt::QueuedConnection
                );
        }
        return;
    }

    // if we get here we are aggregating frames

    static Frame aggframe;
    static int CurrentAggFrame {0};

    // since the call m_pointCloudWindow->appendFrame(frame)
    // is queued then time must be allowed for execution
    // before clearing aggframe.
    // This allows one frame time to occur which should be
    // enough time
    if(CurrentAggFrame >= NumFramestoAggregate) {
        CurrentAggFrame=0;
        aggframe.clear();
    }

    Frame frame;
    // convert latestL2 point cloud packet to Frame of cloud points
    if(!l2lidar.ConvertL2data2pointcloud(frame, Frame3D, mIMUadjust,
                                        mCalOveride, mCalScale, mCalBias)) {
        // if packet is missing or IMU pose correction failed
        // with mIMUadjust is true
        // do not add to point cloud
        return;
    }
    // add frame to aggframe
    static float starttime;
    int64_t oldAggsize = aggframe.size();
    int64_t newFramesize = frame.size();
    int64_t InsertPos;

    if(CurrentAggFrame == 0) {
        InsertPos = 0;
        // this sets up for the time entry to be true time
        // and all other entries to be relative time to first entry
        // if you were using this in ROS2 LIO SAM then the frame stamp
        // for publising would be set to the frame[0].time
        // and the starttime = frame[0].time instead of 2x
        starttime = frame[0].time;
    } else {
        InsertPos = oldAggsize;
        //starttime = aggframe[0].time;
    }

    aggframe.resize(oldAggsize+newFramesize); // increase aggframe for new points

    for(int64_t i=0; i<newFramesize; i++, InsertPos++) {
        aggframe[InsertPos] = frame[i];
        aggframe[InsertPos].time = aggframe[InsertPos].time - starttime;
    }

    CurrentAggFrame++;
    if(CurrentAggFrame < NumFramestoAggregate) {
        // keep building up aggregated frame
        return;
    }

    // Once fully aggregated send aggframe
    if (m_pointCloudWindow) {
        QMetaObject::invokeMethod(
            m_pointCloudWindow,
            [this, frame]() {
                m_pointCloudWindow->appendFrame(aggframe);
            },
            Qt::QueuedConnection
            );
    }
}

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
//  This includes a demonstration of frame aggregation for 3D
//  point cloud frames.
//  Requirements:
//      The l2lidar settings for
//          enableSynHost = true,
//          enableTScorrection = true
//      NumFramesToSkip must be 0
//      This only applies to 3D point cloud data
//--------------------------------------------------------
void MainWindow::onNewLidarIMU()
{
    if(enableIMUstats) {
        LidarImuDataPacket Imu = l2lidar.imu();
        CalcIMUstats(Imu,ImuStats);
        return; // only calculate stats if enabled
    }
}

//--------------------------------------------------------
//
//--------------------------------------------------------
void MainWindow::CalcIMUstats(LidarImuDataPacket Imu, StatsIMU& ImuStats)
{
    float Alpha = 1.0/100.0; // Time contast for 1st order stats filter calculation
    ImuStats.last0 = Imu.data.quaternion[0];
    ImuStats.last1 = Imu.data.quaternion[1];
    ImuStats.last2 = Imu.data.quaternion[2];
    ImuStats.last3 = Imu.data.quaternion[3];

    ImuStats.lastXA = Imu.data.linear_acceleration[0];
    ImuStats.lastYA = Imu.data.linear_acceleration[1];
    ImuStats.lastZA = Imu.data.linear_acceleration[2];

    ImuStats.lastXG = Imu.data.angular_velocity[0];
    ImuStats.lastYG = Imu.data.angular_velocity[1];
    ImuStats.lastZG = Imu.data.angular_velocity[2];

    MeanDev(ImuStats.last0, &ImuStats.Mean0, &ImuStats.Sigma0, Alpha);
    MeanDev(ImuStats.last1, &ImuStats.Mean1, &ImuStats.Sigma1, Alpha);
    MeanDev(ImuStats.last2, &ImuStats.Mean2, &ImuStats.Sigma2, Alpha);
    MeanDev(ImuStats.last3, &ImuStats.Mean3, &ImuStats.Sigma3, Alpha);

    MeanDev(ImuStats.lastXA, &ImuStats.XAmean, &ImuStats.XAsigma, Alpha);
    MeanDev(ImuStats.lastYA, &ImuStats.YAmean, &ImuStats.YAsigma, Alpha);
    MeanDev(ImuStats.lastZA, &ImuStats.ZAmean, &ImuStats.ZAsigma, Alpha);

    MeanDev(ImuStats.lastXG, &ImuStats.XGmean, &ImuStats.XGsigma, Alpha);
    MeanDev(ImuStats.lastYG, &ImuStats.YGmean, &ImuStats.YGsigma, Alpha);
    MeanDev(ImuStats.lastZG, &ImuStats.ZGmean, &ImuStats.ZGsigma, Alpha);

    return;
}

//--------------------------------------------------------
//
//--------------------------------------------------------
void MainWindow::MeanDev(float Value, float* MeanValue, float* sigmaValue, float Alpha)
{
    {
        float delta = Value - *MeanValue;

        // Update mean (EWMA)
        *MeanValue += Alpha * delta;

        // Update variance (EWMA)
        *sigmaValue = (1.0 - Alpha) * (*sigmaValue)
                    + Alpha * delta * (Value - (*MeanValue));
    }

    return;
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
    if(m_pointCloudWindow!=nullptr){
        m_pointCloudWindow->getPointSizeRange((PointSizeRange));
    } else {
        PointSizeRange[0] = 1;
        PointSizeRange[1] = 32;
    }
    config.setPointSizeRange(PointSizeRange);

    if (config.exec() == QDialog::Accepted) {
        // update the L2 UDP connection
        l2lidar.LidarSetCmdConfig(config.getSRCip(),config.getSRCport(),
                                  config.getDSTip(),config.getDSTport());

        // update frames to skip
        NumFramesToSkip = config.getSkipFrame();

        // L2 corrections
        l2lidar.EnableL2TimeCorrection(config.isL2TimeCorrectionEnabled());
        l2lidar.EnableL2TSsync(config.isL2TsyncHostEnabled());
        l2lidar.SetL2TSsyncRate(config.getL2syncRate());

        // latency measurements
        l2lidar.EnableLatencyMeasure(config.isLatencyEnabled());

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
                config.getPacketUpdateRate()!=mPacketBeat->interval()) {

                // of any timer changes then stop all, reset all, restart
                L2DisconnectedButtonsUIs(); // this stops everything
                mHeartBeat->setInterval(config.getDiagUpdateRate());
                mPacketBeat->setInterval(config.getPacketUpdateRate());
                L2ConnectedButtonsUIs(); // this restarts everything
            }
        }

        if(config.isPacketRateChartEnabled() && mNoGraphics) {
            config.setPacketRateChartEnabled(false);
        }

        if(config.isPCviewerEnabled() && m_pointCloudWindow==nullptr) {
            if(mmaxPoints>=7200) {
                if(OpenPointCloudWindow()) {
                    m_pointCloudWindow->RendererTimerStart();
                } else {
                    // opnGL is not available
                    config.setPCviewerEnabled(false);
                }
            }
        }

        if(m_pointCloudWindow!=nullptr && config.getRenderRate()!=m_pointCloudWindow->GetInterval()){
            m_pointCloudWindow->SetInterval(config.getRenderRate());
        }

        // update PC window settings
        PCsettings CurrentPC;
        if(m_pointCloudWindow!=nullptr) {
            m_pointCloudWindow->getPCsettings(CurrentPC);
        }
        CurrentPC.MinDistance = config.getMinDistance();
        CurrentPC.MaxDistance = config.getMaxDistance();
        CurrentPC.PointSize = config.getPointSize();
        if(m_pointCloudWindow!=nullptr) {
            m_pointCloudWindow->setPCsettings(CurrentPC);
        }
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
    // L2 time base correction settings
    l2lidar.EnableL2TimeCorrection(config.isL2TimeCorrectionEnabled());
    l2lidar.EnableL2TSsync(config.isL2TsyncHostEnabled());
    l2lidar.SetL2TSsyncRate(config.getL2syncRate());
    // latency measurements
    l2lidar.EnableLatencyMeasure(config.isLatencyEnabled());

    if(!l2lidar.ConnectL2()) {
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Connect failed");
        msgBox.setInformativeText("Error: "+errorstr);
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
    if(!l2lidar.LidarStartRotation()) {
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Start rotation failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
    return;
}

//--------------------------------------------------------
//  stopRotation
//  button press
//--------------------------------------------------------
void MainWindow::stopRotation()
{
    if(!l2lidar.LidarStopRotation()) {
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Stop rotation failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
    return;
}

//--------------------------------------------------------
//  sendReset
//  button press
//--------------------------------------------------------
void MainWindow::sendReset()
{
    if(!l2lidar.LidarReset()) {
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("L2 reset failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
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
    if(!l2lidar.SetWorkMode(workmode)){
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Set workmode failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
    return;
}

//--------------------------------------------------------
//  sendSetWorkmode
//  button press
//--------------------------------------------------------
void MainWindow::sendGetL2Workmode()
{
    // this will trigger a slot when the
    // the workmode is captured
    if(!l2lidar.GetWorkMode()){
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Get workmode failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();

    }
    return;
}

//--------------------------------------------------------
//  ClearPCwindow
//  button press
//--------------------------------------------------------
void MainWindow::ClearPCwindow()
{
    if(m_pointCloudWindow!=nullptr){
        m_pointCloudWindow->clearPointCloud();
    }
}

//--------------------------------------------------------
//  SyncL2CLock
//  button press
//--------------------------------------------------------
void MainWindow::SyncL2Clock()
{
    l2lidar.SyncL2Clock();
}

//--------------------------------------------------------
//  getVersion
//  button press
//--------------------------------------------------------
void MainWindow::getVersion()
{
    if(!l2lidar.LidarGetVersion()) {
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Request Version failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
    return;
}

//--------------------------------------------------------
//  GetL2Params
//  button press
//--------------------------------------------------------
void MainWindow::GetL2Params()
{
    if(!l2lidar.GetL2Params()){
        QString errorstr = l2lidar.GetLastUDPError();
        QMessageBox msgBox;
        msgBox.setText("Requets L2 params failed");
        msgBox.setInformativeText("Error: "+errorstr);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
    // after ack recived you can use L2ParamsPacket()
    // to get
    return;
}

//--------------------------------------------------------
//  GetL2workmode
//  button press
//--------------------------------------------------------
void MainWindow::GetL2workmode()
{
    uint32_t work = l2lidar.GetL2Workmode();
    if(work >=256) return;
    WorkMode.SetWorkmode(work);
    return;
}

//--------------------------------------------------------
//  SavePC
//  button press
//--------------------------------------------------------
void MainWindow::SavePC()
{
    if (m_pointCloudWindow == nullptr) {
        QMessageBox msgBox;
        msgBox.setText("Can not save point cloud");
        msgBox.setInformativeText("point cloud window must be enabled first");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    // 1. Ask user which format to save
    QMessageBox formatBox;
    formatBox.setWindowTitle("Select Point Cloud Format");
    formatBox.setText("Choose output format:");

    QPushButton *fullBtn = formatBox.addButton("Full Point Cloud (x,y,z,i,r,t)", QMessageBox::AcceptRole);
    QPushButton *ccBtn   = formatBox.addButton("Point Cloud (x,z,y,i,r)", QMessageBox::AcceptRole);
    QPushButton *cancel  = formatBox.addButton(QMessageBox::Cancel);

    formatBox.exec();

    if (formatBox.clickedButton() == cancel)
        return;

    bool useCC = (formatBox.clickedButton() == ccBtn);

    // 2. Choose file
    QString file = QFileDialog::getSaveFileName(
        this,
        "Save Point Cloud",
        "",
        "PointCloud (*.pcd)"
        );

    if (file.isEmpty())
        return;

    // 3. Dispatch to correct save function
    if (useCC) {
        m_pointCloudWindow->savePCDCC(file);
    } else {
        m_pointCloudWindow->savePCD(file);
    }
}
//--------------------------------------------------------
//  LoadPC
//  button press
//--------------------------------------------------------
void MainWindow::LoadPC()
{
    if(m_pointCloudWindow!=nullptr){
        QString file = QFileDialog::getOpenFileName(this,
                                                "Load Point Cloud", "", "PointCloud (*.pcd)");
        if(file=="") return;
        if(!m_pointCloudWindow->loadPCD(file)){
            QMessageBox msgBox;
            msgBox.setText("Can not load point cloud");
            msgBox.setInformativeText("bad file format");
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.exec();
        }

    } else {
        QMessageBox msgBox;
        msgBox.setText("Can not load point cloud");
        msgBox.setInformativeText("point cloud window must be enable first");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

//--------------------------------------------------------
//  resetWindowLayout
//  request windows layout reset
//--------------------------------------------------------
void MainWindow::resetWindowLayout()
{
    SetSettingsReset(true);
}
