//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: IMUDock.cpp
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
//  V.02.5  2026-01-10  added Calibration and internal State dock
//								added IMU dock
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

#include "IMUDock.h"
#include "ui_IMUDock.h"

#include <QString>

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
IMUDock::IMUDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::IMUDock)
{
    ui->setupUi(this);
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
IMUDock::~IMUDock()
{
    delete ui;
}

//--------------------------------------------------------
//  updateDiagnostics
//  signal callback to update diagnostics
//--------------------------------------------------------
void IMUDock::updateIMU(const LidarImuData& Imu )
{
    // Calibration
    QString ResultString;

    ResultString = ResultString.asprintf("%.3f m",Imu.linear_acceleration[0]);
    ui->lblAccelXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f m",Imu.linear_acceleration[1]);
    ui->lblAccelYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f rad",Imu.linear_acceleration[2]);
    ui->lblAccelZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f rad",Imu.angular_velocity[0]);
    ui->lblGyroXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f rad",Imu.angular_velocity[1]);
    ui->lblGyroYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f rad",Imu.angular_velocity[2]);
    ui->lblGyroZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f mm",Imu.quaternion[0]);
    ui->lblQuat0Value->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f rad",Imu.quaternion[1]);
    ui->lblQuat1Value->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f",Imu.quaternion[2]);
    ui->lblQuat2Value->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f",Imu.quaternion[3]);
    ui->lblQuat3Value->setText(ResultString);

}
