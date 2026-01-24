//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: WorkmodeDialog.h
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
//  V0.2.6  2026-01-13 Add WorkMode dialog
//  V0.3.4  2026-01-23 Implement WorkMode dialog
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

#include <QDialog>
#include <QString>
#include "ui_WorkModeDialog.h"


//--------------------------------------------------------
// The workmode is used to set the L2 work mode
// This controls what the startup conditions are on power on
// 2D vs 3D scanner mode
// IMU enable/disable (when disabled the L2 does not send IMU packets
// After chaning the workmode a reset L2 or power cycle is
// needed for the changes to take effect.
// This is done by:
//  l2lidar.SetWorkMode(workmode);
//  l2lidar.LidarReset();
//--------------------------------------------------------

//--------------------------------------------------------
// class declarations
//--------------------------------------------------------
class WorkmodeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WorkmodeDialog(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        ui.setupUi(this);

        // connect the butttons

        // OK / Cancel wiring
        connect(ui.btnClose, &QPushButton::clicked,
                this, &QDialog::close);

        connect(ui.btnSetWM, &QPushButton::clicked,
                 this, &WorkmodeDialog::SendSetL2workmode);

        // Reset View button
         connect(ui.btnResetL2, &QPushButton::clicked,
                 this, &WorkmodeDialog::SendL2reset);
        SetWorkmode(0);
    }


    uint GetFOVmode();
    uint GetPCmode();
    uint GetIMUenabled();
    uint GetComMode();
    uint GetPwrOnMode();
    // There is now known open source way to read the
    // workmode from the L2
    // This only reflects what the app settings for the
    // workmode are set.
    uint GetWorkmode();

    void SetFOVmode(int mode);
    void SetPCmode(int mode);
    void SetIMUenabled(int enable);
    void SetComMode(int mode);
    void SetPwrOnMode(int mode);
    // Setting the workmomde does not perform a L2 reset
    // This only sets the desired workmod in the app
    // To actually set the workmode you must send
    // a SetWorkmode
    void SetWorkmode(int mode);

signals:
    void RequestSetL2workmode();
    void RequestL2reset();


private:
    void SendSetL2workmode();
    void SendL2reset();
    Ui::WorkmodeDialog ui;

    uint32_t mCurrentWorkMode{0};
};

