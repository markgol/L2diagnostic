//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: StatsDock.cpp
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
//  V.02.5  2026-01-10  added Stats dockable window
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

#include "StatsDock.h"
#include "ui_StatsDock.h"

#include <QString>

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
StatsDock::StatsDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::StatsDock)
{
    ui->setupUi(this);
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
StatsDock::~StatsDock()
{
    delete ui;
}

//--------------------------------------------------------
//  updateStats
//  signal callback to update Stats window
//--------------------------------------------------------
void StatsDock::updateStats(PacketStats& Stats)
{
    // Calibration
    QString ResultString;

    ResultString = ResultString.asprintf("%6llu",Stats.TimeSec);
    ui->lblTimeSecValue->setText(ResultString);

    ResultString = ResultString.asprintf("%9llu",Stats.TimeNsec);
    ui->lblTimeNSecValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.countPackets);
    ui->lblTotalValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.count3DPCL);
    ui->lbl3DValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.count2DPCL);
    ui->lbl2DValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.countIMU);
    ui->lblIMUValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.countACK);
    ui->lblACKValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.countOther);
    ui->lblOtherValue->setText(ResultString);

    ResultString = ResultString.asprintf("%6llu",Stats.lostPackets);
    ui->lblLostValue->setText(ResultString);
}
