//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: StatsIMU.h
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
//  V0.4.4  2026-03-02  Added stats to the IMU variables
//  V1.0.0  2026-03-28  Offical release
//  V1.3.0  2026-06-15  Added derived stats for roll, pitch and yaw
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

struct StatsIMU
{
    float lastXA {0.0};
    float lastYA {0.0};
    float lastZA {0.0};
    float lastXG {0.0};
    float lastYG {0.0};
    float lastZG {0.0};
    float last0;
    float last1;
    float last2;
    float last3;
    float lastRoll {0.0};
    float lastPitch{0.0};
    float lastYaw {0.0};
    float lastRollGr {0.0};
    float lastPitchGr {0.0};
    float XAmean {0.0};
    float YAmean {0.0};
    float ZAmean {0.0};
    float XAsigma {0.0};
    float YAsigma {0.0};
    float ZAsigma {0.0};
    float XGmean {0.0};
    float YGmean {0.0};
    float ZGmean {0.0};
    float XGsigma {0.0};
    float YGsigma {0.0};
    float ZGsigma {0.0};
    float Mean0 {0.0};
    float Mean1 {0.0};
    float Mean2 {0.0};
    float Mean3 {0.0};
    float Sigma0 {0.0};
    float Sigma1 {0.0};
    float Sigma2 {0.0};
    float Sigma3 {0.0};
    float RollMean {0.0};
    float RollSigma {0.0};
    float PitchMean {0.0};
    float PitchSigma {0.0};
    float YawMean {0.0};
    float YawSigma {0.0};
    float RollMeanGr {0.0};
    float RollSigmaGr {0.0};
    float PitchMeanGr {0.0};
    float PitchSigmaGr {0.0};

};
