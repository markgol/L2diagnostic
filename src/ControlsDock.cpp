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
//  V0.3.7  2026-01-28  Added measure latency button
//  V0.3.8  2026-01-29  Remove measure latency button
//                      Updated latency measurement
//  V0.3.9  2026-02-01  Corrected visibility for workmode button
//                      Added SAVE/LOAD point cloud buttons
//  V0.3.10 2026-02-01  Added Get L2 Params button
//                      Adjusted sizing of ControlsDock and ConfigDialog
//                          to adjust for use on Ubuntu x64 and ARM64 platforms
//  V1.0.0  2026-03-28  Offical release
//  V2.0.1  2026-08-24  This is the intial V2.x release
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection file
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

    // Sync L2 clock button
    connect(ui->btnL2SetClock, &QPushButton::clicked,
            this, &ControlsDock::SyncL2CLock);

    //  SavePC clock button
    connect(ui->btnSavePC, &QPushButton::clicked,
            this, &ControlsDock::SavePC);

    //  LoadPC button
    connect(ui->btnLoadPC, &QPushButton::clicked,
            this, &ControlsDock::LoadPC);

    //  LoadPC button
    connect(ui->btnL2GetParams, &QPushButton::clicked,
            this, &ControlsDock::GetL2Params);

    //  Swith to calibration mode button
    connect(ui->btnCalibrate, &QPushButton::clicked,
            this, &ControlsDock::CalibrationMode);

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
    ui->btnStart->setEnabled(connected); // Start rotation, diagnostic mode
    ui->btnStop->setEnabled(connected);
    ui->btnReset->setEnabled(connected);
    ui->btnL2Disconnect->setEnabled(connected);
    ui->btnVersion->setEnabled(connected);
    ui->btnL2SetClock->setEnabled(connected);
    ui->btnWorkmode->setEnabled(connected);
    ui->btnL2GetParams->setEnabled(connected);

    // this button is always enabled
    ui->btnConfig->setEnabled(true);
    ConnectState = connected;
}

//--------------------------------------------------------
//  GetConnectedState
//--------------------------------------------------------
bool ControlsDock::GetConnectedState()
{
    return ConnectState;
}

//--------------------------------------------------------
//  closeEvent
//--------------------------------------------------------
void ControlsDock::closeEvent(QCloseEvent* event)
{
    event->ignore();
}
