//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ControlsDock.cpp
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
//  V.02.6  2026-01-13  added button controls dockable dialog
//  V0.3.6  2026-01-25  Added clear point cloud window button
//
//--------------------------------------------------------
#include "ControlsDock.h"

#include <QCloseEvent>

#include "ui_ControlsDock.h"

//--------------------------------------------------------
//  ControlsDock constructor
//--------------------------------------------------------
ControlsDock::ControlsDock(QWidget *parent)
    : QDockWidget(parent)
    , ui(new Ui::ControlsDock)
{
    ui->setupUi(this);

    //connections to the buttons to signals

    // start rotation button
    connect(ui->btnStart, &QPushButton::clicked,
            this, &ControlsDock::startRotationRequested);

    // stop rotation button
    connect(ui->btnStop, &QPushButton::clicked,
            this, &ControlsDock::stopRotationRequested);

    // reset window button
    connect(ui->btnReset, &QPushButton::clicked,
            this, &ControlsDock::L2resetRequested);

    // get version button
    connect(ui->btnVersion, &QPushButton::clicked,
            this, &ControlsDock::GetVersionRequested);

    // config button
    connect(ui->btnConfig, &QPushButton::clicked,
            this, &ControlsDock::ConfigRequested);

    // workmode button
    connect(ui->btnWorkmode, &QPushButton::clicked,
            this, &ControlsDock::WorkmodeRequested);

    // l2 connect button
    connect(ui->btnL2Connect, &QPushButton::clicked,
            this, &ControlsDock::L2connectRequested);

    // l2 disconnect button
    connect(ui->btnL2Disconnect, &QPushButton::clicked,
            this, &ControlsDock::L2disconnectRequested);

    // reset window geometry and state button
    connect(ui->btnResetWindows, &QPushButton::clicked,
            this, &ControlsDock::ResetWindowsRequested);

    // clear point cloud button
    connect(ui->btnClearDisplay, &QPushButton::clicked,
            this, &ControlsDock::ClearPCwindowRequested);

    setConnectState(false); // L2 is disconnected at start
}

//--------------------------------------------------------
//  ControlsDock destructor
//--------------------------------------------------------
ControlsDock::~ControlsDock()
{
    delete ui;
}

//--------------------------------------------------------
//  setConnectState
//  this disables/enables various button depending on
//  wether the L2 is connected
//--------------------------------------------------------
void ControlsDock::setConnectState(bool connected)
{
    // true - L2 connected
    // false - L2 disconnected
    ui->btnL2Connect->setEnabled(!connected);
    ui->btnStart->setEnabled(connected);
    ui->btnStop->setEnabled(connected);
    ui->btnReset->setEnabled(connected);
    ui->btnL2Disconnect->setEnabled(connected);
    ui->btnVersion->setEnabled(connected);

    // this button is always enabled
    ui->btnConfig->setEnabled(true);
    ConnectState = connected;
}

bool ControlsDock::GetConnectedState()
{
    return ConnectState;
}

void ControlsDock::closeEvent(QCloseEvent* event)
{
    event->ignore();
}

