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
//  V0.2.8  2026-01-16  Added reset flag for resetting
//                      the geometry, state and visibility
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
void MainWindow::saveSettings(bool resetRequested)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    // Window geometry and state for Qt windows
    settings.beginGroup("window");
    if(!resetRequested) {
        settings.setValue("geometry", saveGeometry());
        settings.setValue("state", saveState());
    } else {
        settings.setValue("reset",true); // once set true only a restart clears its
    }
    settings.endGroup();

    // Window geometry and state for point cloud window
    if(!resetRequested) {
        if(m_pointCloudWindow) {
            m_pointCloudWindow->saveWindowState();
        }
    }

    // Network
    settings.beginGroup("net");
    settings.setValue("srcIP", config.getSRCip());
    settings.setValue("dstIP", config.getDSTip());
    settings.setValue("srcPort", config.getSRCport());
    settings.setValue("dstPort", config.getDSTport());
    settings.endGroup();

    // throttling
    settings.beginGroup("throttling");
    settings.setValue("NumFramesToSkip", config.getSkipFrame());
    settings.setValue("PacketUpdateRate", config.getPacketUpdateRate());
    settings.setValue("DiagUpdateRate", config.getDiagUpdateRate());
    settings.setValue("PCupdateRate", config.getPCupdateRate());
    settings.setValue("RendererUpdateRate",config.getRenderRate());
    settings.endGroup();

    // windows visibility
    if(!resetRequested) {
        settings.beginGroup("visibility");
        settings.setValue("PCviewer", config.isPCviewerEnabled());
        settings.setValue("ACK", config.isACKenabled());
        settings.setValue("Diag", config.isDiagEnabled());
        settings.setValue("IMU", config.isIMUenabled());
        settings.setValue("PacketRateChart", config.isPacketRateChartEnabled());
        settings.setValue("Stats", config.isStatsEnabled());
        settings.endGroup();
    }

    // point cloud view settings
    settings.beginGroup("PCview");
    settings.setValue("Distance",config.getPCWdistance());
    settings.setValue("Yaw",config.getPCWyaw());
    settings.setValue("Pitch",config.getPCWpitch());
    settings.endGroup();

}

//--------------------------------------------------------
//  GetSettingsReset
//  flag for settings reset on next start of application
//--------------------------------------------------------
bool MainWindow::GetSettingsReset()
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    bool Reset{false};
    // Window geometry

    settings.beginGroup("geometry");
    Reset = settings.value("reset", false).toBool();
    settings.endGroup();

    return Reset;
}

//--------------------------------------------------------
//  SetSettingsReset
//  flag for settings reset on next start of application
//--------------------------------------------------------
void MainWindow::SetSettingsReset(bool Reset)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    // Window geometry
    settings.beginGroup("geometry");
    settings.setValue("reset",Reset);
    settings.endGroup();

    return;
}

//--------------------------------------------------------
//  loadSettings()
//  load user settings from an ini file format
//--------------------------------------------------------
void MainWindow::loadSettings(bool resetRequested)
{
    QString iniPath = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation) + "/L2diagnostic.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    // Window geometry and state for Qt windows

    settings.beginGroup("window");
    if(!resetRequested) {
        if (settings.contains("geometry"))
            restoreGeometry(settings.value("geometry").toByteArray());

        if (settings.contains("state"))
            restoreState(settings.value("state").toByteArray());
    }
    settings.endGroup();

    // Window geometry and state for point cloud window
    if(!resetRequested) {
        m_pointCloudWindow->restoreWindowState();
    }

    // Network
    settings.beginGroup("net");
    config.setSRCip(settings.value("srcIP", "192.168.1.2").toString());
    config.setDSTip(settings.value("dstIP", "192.168.1.62").toString());
    config.setSRCport(settings.value("srcPort", 6201).toUInt());
    config.setDSTport(settings.value("dstPort", 6101).toUInt());
    settings.endGroup();

    // throttling
    settings.beginGroup("throttling");
    config.setSkipFrame(settings.value("NumFramesToSkip", 4).toUInt());
    config.setPacketUpdateRate(settings.value("PacketUpdateRate", 100).toUInt());
    config.setDiagUpdateRate(settings.value("DiagUpdateRate", 250).toUInt());
    config.setPCupdateRate(settings.value("PCupdateRate", 16).toUInt());
    config.setRenderRate(settings.value("RendererUpdateRate", 33).toUInt());
    settings.endGroup();

    // windows visibility
    settings.beginGroup("visibility");
    if(resetRequested){
        // reset settings to initial state
        config.setPCviewerEnabled(true);
        config.setACKenabled(true);
        config.setDiagEnabled(true);
        config.setIMUenabled(true);
        config.setPacketRateChartEnabled(true);
        config.setStatsEnabled(true);
      } else {
        config.setPCviewerEnabled(settings.value("PCviewer", true).toBool());
        config.setACKenabled(settings.value("ACK", true).toBool());
        config.setDiagEnabled(settings.value("Diag", true).toBool());
        config.setIMUenabled(settings.value("IMU", true).toBool());
        config.setPacketRateChartEnabled(settings.value("PacketRateChart", true).toBool());
        config.setStatsEnabled(settings.value("Stats", true).toBool());
    }

    // point cloud view settings
    settings.beginGroup("PCview");
    config.setPCWdistance(settings.value("Distance", 10.0).toDouble());
    config.setPCWyaw(settings.value("Yaw", 145.0).toDouble());
    config.setPCWpitch(settings.value("Pitch", 20.0).toDouble());
    settings.endGroup();

}
