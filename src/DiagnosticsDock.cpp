//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: DiagnosticsDock.cpp
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
//  V.02.5  2026-01-10  added Calibration and internal State dialog
//  V0.3.7  2026-01-29  added RTT latency measurement
//                      added seq ID for point cloud and imu packets
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

#include "DiagnosticsDock.h"
#include "ui_DiagnosticsDock.h"

#include <QString>

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
DiagnosticsDock::DiagnosticsDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::DiagnosticsDock)
{
    ui->setupUi(this);
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
DiagnosticsDock::~DiagnosticsDock()
{
    delete ui;
}

//--------------------------------------------------------
//  updateDiagnostics
//  signal callback to update diagnostics
//--------------------------------------------------------
void DiagnosticsDock::updateDiagnostics(const LidarInsideState& State,
                       const LidarCalibParam& Calib,
                       const float range_min, const float range_max,
                       const uint32_t SeqID,
                       const double MeasuredLatency,
                       const double MinLatency)
{
    // Calibration
    QString ResultString;

    ResultString = ResultString.asprintf("%.6f m",Calib.a_axis_dist);
    ui->lblAaxisValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f m",Calib.b_axis_dist);
    ui->lblBaxisValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.4f rad",Calib.theta_angle_bias);
    ui->lblThetaBiasValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f rad",Calib.alpha_angle_bias);
    ui->lblAlphaBiasValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f rad",Calib.beta_angle);
    ui->lblBetaAngleValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f rad",Calib.xi_angle);
    ui->lblXiAngleValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f mm",Calib.range_bias);
    ui->lblRangeBiasValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f",Calib.range_scale);
    ui->lblRangeScaleValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.1f m",range_min);
    ui->lblRangeMinValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.1f m",range_max);
    ui->lblRangeMaxValue->setText(ResultString);

    ResultString = ResultString.asprintf("%d",SeqID);
    ui->lblSequenceIDvalue->setText(ResultString);

    if(MeasuredLatency <0.0) {
        ui->lblRTTvalue->setText("Invalid");
    } else {
        ResultString = ResultString.asprintf("%.3fmsec",MeasuredLatency);
        ui->lblRTTvalue->setText(ResultString);
        ResultString = ResultString.asprintf("%.3fmsec",MinLatency);
        ui->lblMinRTTvalue->setText(ResultString);
    }

    // Internal state
    ResultString = ResultString.asprintf("%u",State.sys_rotation_period);
    ui->lblSysSpeedValue->setText(ResultString);

    ResultString = ResultString.asprintf("%u",State.com_rotation_period);
    ui->lblComSpeedValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.1f",State.dirty_index);
    ui->lblDirtyValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f",State.packet_lost_up);
    ui->lblLostUpValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f",State.packet_lost_down);
    ui->lblLostDownValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.1fC",State.apd_temperature);
    ui->lblApdTempValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.2f",State.apd_voltage);
    ui->lblApdVoltageValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.2f",State.laser_voltage);
    ui->lblLaserVoltageValue->setText(ResultString);

    ResultString = ResultString.asprintf("%.1fC",State.imu_temperature);
    ui->lblImuTempValue->setText(ResultString);
}

//--------------------------------------------------------
//  updateVersion
//  signal callback to Version info
//--------------------------------------------------------
void DiagnosticsDock::updateVersion(const LidarVersionData& version)
{
    if(version.hw_version[0]==0) {
        // nothing to display
        return;
    }

    // update Version information if present
    QString HWversion = QString().asprintf("HW: %d.%d.%d.%d",
                                           version.hw_version[0],
                                           version.hw_version[1],
                                           version.hw_version[2],
                                           version.hw_version[3]);
    QString FWversion = QString().asprintf("FW: %d.%d.%d.%d",
                                           version.sw_version[0],
                                           version.sw_version[1],
                                           version.sw_version[2],
                                           version.sw_version[3]);
    QString BuildDate = QString().asprintf("Compile date: 20%c%c-%c%c-%c%c",
                                           version.date[0],
                                           version.date[1],
                                           version.date[2],
                                           version.date[3],
                                           version.date[4],
                                           version.date[5]);

    QString Product = QString::fromUtf8((const char *)version.reserve);

     ui->lblProductValue->setText(Product);
     ui->lblHardwareValue->setText(HWversion);
     ui->lblFirmwareValue->setText(FWversion);
     ui->lblDateValue->setText(BuildDate);
}
