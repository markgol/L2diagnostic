//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Settings.cpp
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
//                      Moved Load/Save ini setting into
//                      separate source file: Settings.cpp
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
// Main project includes required before anything else
//--------------------------------------------------------
#include "MainWindow.h"

//--------------------------------------------------------
//  Qt includes
//--------------------------------------------------------
#include <QHostAddress>
#include <QDateTime>
#include <QSettings>
#include <QStandardPaths>
#include <QDebug>

//--------------------------------------------------------
//  saveSettings()
//  save user settings in an ini file format
//--------------------------------------------------------
void MainWindow::saveSettings()
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    // Window geometry
    settings.beginGroup("window");
    settings.setValue("geometry", saveGeometry());
    settings.endGroup();

    // Network
    settings.beginGroup("net");
    settings.setValue("srcIP", config.getSRCip());
    settings.setValue("dstIP", config.getDSTip());
    settings.setValue("srcPort", config.getSRCport());
    settings.setValue("dstPort", config.getDSTport());
    settings.endGroup();

    // Recording
    settings.beginGroup("record");
    settings.setValue("csv", config.csvEnabled());
    settings.endGroup();
}

//--------------------------------------------------------
//  loadSettings()
//  load user settings from an ini file format
//--------------------------------------------------------
void MainWindow::loadSettings()
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    // Window geometry
    settings.beginGroup("window");
    restoreGeometry(settings.value("geometry").toByteArray());
    settings.endGroup();

    // Network
    settings.beginGroup("net");
    config.setSRCip(settings.value("srcIP", "192.168.1.2").toString());
    config.setDSTip(settings.value("dstIP", "192.168.1.62").toString());
    config.setSRCport(settings.value("srcPort", 6201).toUInt());
    config.setDSTport(settings.value("dstPort", 6101).toUInt());
    settings.endGroup();

    // Recording
    settings.beginGroup("record");
    config.setCsvEnabled(settings.value("csv", false).toBool());
    settings.endGroup();
}
