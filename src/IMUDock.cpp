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
//  V0.2.5  2026-01-10  added Calibration and internal State dock
//								added IMU dock
//  V0.2.6  2026-01-12  Corrected IMU units
//  V0.3.5  2026-01-24  Minor formatting fix to accelerometer data
//  V0.4.4  2026-03-02  Added stats to the IMU variables
//  V1.0.0  2026-03-28  Offical release
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
//  updateIMU
//  signal callback to update IMU window
//--------------------------------------------------------
void IMUDock::updateIMU(const LidarImuData& Imu)
{
    // no stats on values
    // Calibration
    QString ResultString;

    ResultString = ResultString.asprintf("%8.3f",Imu.linear_acceleration[0]);
    ui->lblAccelXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",Imu.linear_acceleration[1]);
    ui->lblAccelYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",Imu.linear_acceleration[2]);
    ui->lblAccelZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",Imu.angular_velocity[0]);
    ui->lblGyroXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",Imu.angular_velocity[1]);
    ui->lblGyroYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",Imu.angular_velocity[2]);
    ui->lblGyroZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",Imu.quaternion[0]);
    ui->lblQuat0Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",Imu.quaternion[1]);
    ui->lblQuat1Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",Imu.quaternion[2]);
    ui->lblQuat2Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",Imu.quaternion[3]);
    ui->lblQuat3Value->setText(ResultString);

    ResultString = ResultString.asprintf("%d", Imu.info.seq);
    ui->lblSeqIDvalue->setText(ResultString);
}

//--------------------------------------------------------
//  updateIMU
//  signal callback to update IMU window
//--------------------------------------------------------
void IMUDock::updateIMU(const StatsIMU& ImuStats)
{
    // no stats on values
    // Calibration
    QString ResultString;

    // last reported value for all IMU data
    ResultString = ResultString.asprintf("%8.2f",ImuStats.lastXA);
    ui->lblAccelXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",ImuStats.lastYA);
    ui->lblAccelYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",ImuStats.lastZA);
    ui->lblAccelZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",ImuStats.lastXG);
    ui->lblGyroXValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",ImuStats.lastYG);
    ui->lblGyroYValue->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",ImuStats.lastZG);
    ui->lblGyroZValue->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.last0);
    ui->lblQuat0Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.last1);
    ui->lblQuat1Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.last2);
    ui->lblQuat2Value->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.last3);
    ui->lblQuat3Value->setText(ResultString);

    // report mean Quaternions values
    ResultString = ResultString.asprintf("%9.6f",ImuStats.Mean0);
    ui->lbl0mean->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.Mean1);
    ui->lbl1mean->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.Mean2);
    ui->lbl2mean->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",ImuStats.Mean3);
    ui->lbl3mean->setText(ResultString);

    // report mean Acceleration values
    ResultString = ResultString.asprintf("%8.2f",ImuStats.XAmean);
    ui->lblXAmean->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",ImuStats.YAmean);
    ui->lblYAmean->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",ImuStats.ZAmean);
    ui->lblZAmean->setText(ResultString);

    // report mean Gyro values
    ResultString = ResultString.asprintf("%8.3f",ImuStats.XGmean);
    ui->lblXGmean->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",ImuStats.YGmean);
    ui->lblYGmean->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",ImuStats.ZGmean);
    ui->lblZGmean->setText(ResultString);

    // report stddev Quaternions values
    ResultString = ResultString.asprintf("%9.6f",sqrt(ImuStats.Sigma0));
    ui->lbl0dev->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",sqrt(ImuStats.Sigma1));
    ui->lbl1dev->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",sqrt(ImuStats.Sigma2));
    ui->lbl2dev->setText(ResultString);

    ResultString = ResultString.asprintf("%9.6f",sqrt(ImuStats.Sigma3));
    ui->lbl3dev->setText(ResultString);

    // report stddev Acceleration values
    ResultString = ResultString.asprintf("%8.2f",sqrt(ImuStats.XAsigma));
    ui->lblXAdev->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",sqrt(ImuStats.YAsigma));
    ui->lblYAdev->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f",sqrt(ImuStats.ZAsigma));
    ui->lblZAdev->setText(ResultString);

    // report stddev Gyro values
    ResultString = ResultString.asprintf("%8.3f",sqrt(ImuStats.XGsigma));
    ui->lblXGdev->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",sqrt(ImuStats.YGsigma));
    ui->lblYGdev->setText(ResultString);

    ResultString = ResultString.asprintf("%8.3f",sqrt(ImuStats.ZGsigma));
    ui->lblZGdev->setText(ResultString);

    //  -------------------------------------

    ResultString = ResultString.asprintf("");
    ui->lblSeqIDvalue->setText(ResultString);
}
