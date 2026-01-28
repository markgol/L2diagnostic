//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ConfigDialog.cpp
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
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//  V0.2.4  2026-01-10  removed CSV checkbox, never fully implemented
//                      Added SkipFrame spinbox
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
// This is the user configuration dialog
//--------------------------------------------------------
#include "ConfigDialog.h"
#include <QMessageBox>

//--------------------------------------------------------
//  private slot
//  ConfigureUDP
//--------------------------------------------------------
void ConfigDialog::ConfigureUDP()
{
    // Distance
    // Yaw
    // Pitch

    emit requestConfigureUDP();

}

//--------------------------------------------------------
//  private slot
//  ResetPCview
//--------------------------------------------------------
void ConfigDialog::ResetPCview()
{
    // Distance
    // Yaw
    // Pitch

    emit requestViewReset();

}

//--------------------------------------------------------
//  signal
//  setPointSizeRange
//--------------------------------------------------------
void ConfigDialog::setPointSizeRange(float *PointSizeRange)
{
    // set the limits on the cloud point size based on system settings
    ui.spinPointSize->setRange(PointSizeRange[0],PointSizeRange[1]);
    return;
}


