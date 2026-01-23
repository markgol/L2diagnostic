//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: WorkmodeDialog.cpp
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
//  V0.2.6  20260-01-13 Add WorkMode dialog
//  V0.3.3  2026-01-22 Implement WorkMode dialog
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
// Workmode dialog
//--------------------------------------------------------

#include "WorkModeDialog.h"
#include "ui_WorkModeDialog.h"
#include "unitree_lidar_protocol.h"

//--------------------------------------------------------
//  SetWorkmode
//--------------------------------------------------------
void WorkmodeDialog::SetWorkmode(int workmode)
{
    mCurrentWorkMode = workmode;
    SetFOVmode(workmode);
    SetPCmode(workmode);
    SetIMUenabled(workmode);
    SetComMode(workmode);
    SetPwrOnMode(workmode);
}

//--------------------------------------------------------
//  GetWorkmode
//--------------------------------------------------------
uint WorkmodeDialog::GetWorkmode()
{
    int workmode {0};

    workmode =  GetFOVmode() |
                GetPCmode()|
                GetIMUenabled() |
                GetComMode() |
                GetPwrOnMode();

    mCurrentWorkMode = workmode;
    return workmode;
}

//--------------------------------------------------------
//  GetFOVmode
//--------------------------------------------------------
uint WorkmodeDialog::GetFOVmode()
{
    // WORK_MODE_WFOV
    if(ui.chkWideFOV->isChecked()) {
        return WORK_MODE_WFOV;  // bit field value
    }

    return 0; // STD FOV is 0
}

//--------------------------------------------------------
//  GetPCmode
//--------------------------------------------------------
uint WorkmodeDialog::GetPCmode()
{
    // WORK_MODE_2D
    if(ui.chk2D->isChecked()) {
        return WORK_MODE_2D; // bit field value
    }

    return 0; // 3D mode is 0
}

//--------------------------------------------------------
//  GetIMUenabled
//--------------------------------------------------------
uint WorkmodeDialog::GetIMUenabled()
{
    // WORK_MODE_IMU_DISABLE
    if(ui.chkIMUoff->isChecked()) {
        return WORK_MODE_IMU_DISABLE; // bit field value
    }

    return 0; // IMU on is 0
}

//--------------------------------------------------------
//  GetComMode
//--------------------------------------------------------
uint WorkmodeDialog::GetComMode()
{
    // WORK_MODE_SERIAL
    if(ui.chkSerial->isChecked()) {
        return WORK_MODE_SERIAL; // bit field value
    }

    return 0; // UDP mode is 0
}

//--------------------------------------------------------
//  GetPwrOnMode
//--------------------------------------------------------
uint WorkmodeDialog::GetPwrOnMode()
{
    // WORK_MODE_STARTUP_WAIT
    if(ui.chkPWRstdby->isChecked()) {
        return WORK_MODE_STARTUP_WAIT; // bit field value
    }

    return 0; // Power on is 0
}

//--------------------------------------------------------
//  SetFOVmode
//--------------------------------------------------------
void WorkmodeDialog::SetFOVmode(int mode)
{
    // mode is the workmode integer (bitfield)
    // WORK_MODE_WFOV
    if((mode & WORK_MODE_WFOV)==0) {
        ui.chkStdFOV->setChecked(true);
    } else {
        ui.chkWideFOV->setChecked(true);
    }
}

//--------------------------------------------------------
//  SetPCmode
//--------------------------------------------------------
void WorkmodeDialog::SetPCmode(int mode)
{
    // mode is the workmode integer (bitfield)
    // WORK_MODE_2D
    if((mode & WORK_MODE_2D)==0) {
        ui.chk3D->setChecked(true);
    } else {
        ui.chk2D->setChecked(true);
    }
}

//--------------------------------------------------------
//  SetIMUenabled
//--------------------------------------------------------
void WorkmodeDialog::SetIMUenabled(int mode)
{
    // mode is the workmode integer (bitfield)
    // WORK_MODE_IMU_DISABLE
    if((mode & WORK_MODE_IMU_DISABLE)==0) {
        ui.chkIMUon->setChecked(true);
    } else {
        ui.chkIMUoff->setChecked(true);
    }
}

//--------------------------------------------------------
//  SetComMode
//--------------------------------------------------------
void WorkmodeDialog::SetComMode(int mode)
{
    // mode is the workmode integer (bitfield)
    // WORK_MODE_SERIAL
    if((mode & WORK_MODE_SERIAL)==0) {
        ui.chkUDP->setChecked(true);
    } else {
        ui.chkSerial->setChecked(true);
    }
}

//--------------------------------------------------------
//  SetPwrOnMode
//--------------------------------------------------------
void WorkmodeDialog::SetPwrOnMode(int mode)
{
    // mode is the workmode integer (bitfield)
    // WORK_MODE_STARTUP_WAIT
     if((mode & WORK_MODE_STARTUP_WAIT)==0) {
        ui.chkPWRon->setChecked(true);
    } else {
        ui.chkPWRstdby->setChecked(true);
    }
}

