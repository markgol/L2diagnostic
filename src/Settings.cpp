//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Settings.cpp
//
//  //  Purpose:
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
//  of packets.
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//                      Moved Load/Save ini setting into
//                      separate source file: Settings.cpp
//  V0.2.8  2026-01-16  Added reset flag for resetting
//                      the geometry, state and visibility
//  V0.3.2  2026-01-22  Updated defaults for various settings
//                      deleted unused older settings
//                      added settings for saving all window geometry
//                      and state
//                      Completed most display/renderer controlls
// V0.3.4   2026-01-23  Added save and restore for workmode settings
// V0.3.5   2026-01-24  Added scan mode setting (3D or 2D)
//                      Added cloud point size control
// V0.3.6   2026-01-25  correction to load/save settings
//                      moved 2 settings into PCview group
//                      removed PCbuffering group
// V0.3.9   2026-02-01  Added L2 time base corrections parameters
// V0.3.10  2026-02-02  Added enable latency measurements flag
// V0.3.11  2026-02-04  Changed default settings for PC viewer enable
//                      to false one first time execution of app.
//                      User must explicitly set PC viewer to enabled
//                      in config dialog if this is first time run or
//                      PC viewer was closed the at last time run.
// V0.4.1   2026-02-12  Added MAC address
// V0.4.3   2026-02-20  Changed default sync time to host to 30 msec
//                      Added 'n' frame aggregation for point cloud frame
//  V1.0.0  2026-03-28  Offical release
//  V1.1.0  2026-04-20  Added override of range calibration params
//  V1.3.0  2026-06-15  Added flag for IMU adjust just to roll, pitch vs roll, pitch, yaw
//  V1.3.1  2026-06-21  Added config params for settings gatewey IP address and subnet mask
//  V1.3.2  2026-07-08  Added FlattenedScanEnabled, MinScnaAngle, MaxScanAngle
//  V2.0.0 RC1 2026-08-01
//                      Updated save and load settings for range correction calibration
//                      Additional calibration override (6 total)
//  V2.0.0 RC2 2026-08-24
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
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
#include <cstdio>

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
    settings.setValue("srcIP", config.getSRCip()); // L2
    settings.setValue("dstIP", config.getDSTip()); // host
    settings.setValue("srcPort", config.getSRCport()); // l2
    settings.setValue("dstPort", config.getDSTport()); //host
    settings.setValue("MACaddress",config.GetMAC());
    settings.setValue("GatewayIP", config.getGatewayip());
    settings.setValue("SubnetMask",config.getSubnetMask());
    settings.endGroup();

    // L2 setup
    settings.beginGroup("L2");
    uint mode =WorkMode.GetWorkmode();

    settings.setValue("TimeCorrection", config.isL2TimeCorrectionEnabled());
    settings.setValue("TSscaleNum", config.getL2TscaleNum());
    settings.setValue("TSscaleDen", config.getL2TscaleDen());
    settings.setValue("SyncHost", config.isL2TsyncHostEnabled());
    settings.setValue("SyncHostRate", config.getL2syncRate());
    settings.setValue("EnableLatency", config.isLatencyEnabled());
    settings.setValue("UseSystemTimestamp",config.isUseSystemNowEnabled());

    settings.setValue("workmode", mode);

    settings.endGroup();

    // Calibration settings
    settings.beginGroup("Calibration");

        // settings for the override of calibration overrides
        settings.setValue("EnableCalOVR", m_calibrationDock->isCalOVRenabled());
        settings.setValue("RangeScale", m_calibrationDock->getRangeScale());
        settings.setValue("RangeBias", m_calibrationDock->getRangeBias());
        settings.setValue("AlphaBias", m_calibrationDock->getAlphaBias());
        settings.setValue("AlphaStep", m_calibrationDock->getAlphaStep());
        settings.setValue("ThetaBias", m_calibrationDock->getThetaBias());
        settings.setValue("BetaAngle", m_calibrationDock->getBetaAngle());
        settings.setValue("XiAngle", m_calibrationDock->getXiAngle());
        settings.setValue("MinRange", m_calibrationDock->getMinRange_mm());
        settings.setValue("Maxrange", m_calibrationDock->getMaxRange_mm());

        // make sure l2lidar class is updated with current settings
        l2lidar.EnableCalibrationOVR(m_calibrationDock->isCalOVRenabled());
        l2lidar.SetRangeBiasOVR(m_calibrationDock->getRangeBias());
        l2lidar.SetRangeScaleOVR(m_calibrationDock->getRangeScale());
        l2lidar.SetAlphaAngleBiasOVR(m_calibrationDock->getAlphaBias());
        l2lidar.SetAlphaAngleStepOVR(m_calibrationDock->getAlphaStep());
        l2lidar.SetThetaAngleBiasOVR(m_calibrationDock->getThetaBias());
        l2lidar.SetBetaAngleOVR(m_calibrationDock->getBetaAngle());
        l2lidar.SetXiAngleOVR(m_calibrationDock->getXiAngle());
        l2lidar.SetMinRange_mm(m_calibrationDock->getMinRange_mm());
        l2lidar.SetMaxRange_mm(m_calibrationDock->getMaxRange_mm());

        settings.setValue("EnableRangeCorrection",m_calibrationDock->isRangeCorrectionEnabled());
        settings.setValue("RangeCalibrationFilename",m_calibrationDock->GetRangeCalFile());
        l2lidar.EnableRangeCorrection(m_calibrationDock->isRangeCorrectionEnabled());

    settings.endGroup();

    // throttling
    settings.beginGroup("throttling");
    settings.setValue("NumFramesToSkip", config.getSkipFrame());
    settings.setValue("PacketUpdateRate", config.getPacketUpdateRate());
    settings.setValue("DiagUpdateRate", config.getDiagUpdateRate());
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

        // point cloud view settings

        settings.setValue("Distance",defaultPCsettings.Distance);
        settings.setValue("Yaw",defaultPCsettings.Yaw);
        settings.setValue("Pitch",defaultPCsettings.Pitch);
        settings.setValue("PointSize",defaultPCsettings.PointSize);
        settings.setValue("MinDistance",defaultPCsettings.MinDistance);
        settings.setValue("MaxDistance",defaultPCsettings.MaxDistance);

        // point cloud buffering settings

        settings.setValue("MaxPoints",mmaxPoints);
        bool IMUadjust = config.isIMUadjustEnabled();
        settings.setValue("IMUadjust",IMUadjust);
        l2lidar.EnableIMUadjust(IMUadjust);

        bool FlattenScanEnabled = config.isFlattenScanEnabled();
        settings.setValue("FlattenScanEnabled",FlattenScanEnabled);
        l2lidar.EnableFlattenScan(FlattenScanEnabled);

        double StartScanAngle = config.getStartScanAngle();
        settings.setValue("StartScanAngle",StartScanAngle);
        l2lidar.SetStartScanAngle(StartScanAngle);

        double ScanAngleWidth = config.getScanAngleWidth();
        settings.setValue("ScanAngleWidth",ScanAngleWidth);
        l2lidar.SetScanAngleWidth(ScanAngleWidth);

        double IMUPCtimeConstraint = config.getIMUPCtimeConstraint();
        settings.setValue("IMUPCtimeConstraint", IMUPCtimeConstraint);
        l2lidar.SetIMUPCtimeConstraint(IMUPCtimeConstraint);

        mNumFramestoAggregate = config.getAggFrames();
        settings.setValue("NumFramestoAggregate",mNumFramestoAggregate);

    settings.endGroup();
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
        if(m_pointCloudWindow)
            m_pointCloudWindow->restoreWindowState();
    }

    // Network
    settings.beginGroup("net");
        config.setSRCip(settings.value("srcIP", "192.168.1.2").toString()); // factory default
        config.setDSTip(settings.value("dstIP", "192.168.1.62").toString()); // factory default
        config.setSRCport(settings.value("srcPort", 6201).toUInt()); // factory default
        config.setDSTport(settings.value("dstPort", 6101).toUInt()); // factory default
        config.setGatewayip(settings.value("GatewayIP", "0.0.0.0").toString()); // factory default
        config.setSubnetMask(settings.value("SubnetMask", "255.255.255.0").toString()); // factory default
        config.SetMAC(settings.value("MACaddress", "0c:29:ab:7c:00:01").toString());
    settings.endGroup();

    // L2 setup
    settings.beginGroup("L2");
        WorkMode.SetWorkmode(settings.value("workmode",0).toUInt());
        config.SetL2TimeCorrectionEnabled(settings.value("TimeCorrection", true).toBool());
        config.setL2TscaleNum(settings.value("TSscaleNum", 2).toUInt());
        config.setL2TscaleDen(settings.value("TSscaleDen", 1).toUInt());

        config.SetL2TsyncHostEnabled(settings.value("SyncHost", true).toBool());
        // default sync to host 6 times per one horizontal revoultion
        // 30 msec <= 5.55Hz * 6
        config.setL2syncRate(settings.value("SyncHostRate", 30).toUInt());
        config.setEnableLatency(settings.value("EnableLatency", true).toBool());
        config.setUseSystemNow(settings.value("UseSystemTimestamp", false).toBool());
    settings.endGroup();

    // throttling
    settings.beginGroup("throttling");
        config.setSkipFrame(settings.value("NumFramesToSkip", 0).toUInt());
        config.setPacketUpdateRate(settings.value("PacketUpdateRate", 100).toUInt()); // 10Hz
        config.setDiagUpdateRate(settings.value("DiagUpdateRate", 200).toUInt());   // 5Hz
        config.setRenderRate(settings.value("RendererUpdateRate", 33).toUInt());    // 30Hz
    settings.endGroup();

    settings.beginGroup("Calibration");

        // settings for the override of calibration overrides
        m_calibrationDock->EnableCalOVR(settings.value("EnableCalOVR", false).toBool());
        m_calibrationDock->SetRangeScale(settings.value("RangeScale", 0.001).toDouble());
        m_calibrationDock->setRangeBias(settings.value("RangeBias", -535).toDouble());
        m_calibrationDock->setAlphaBias(settings.value("AlphaBias", 1.6).toDouble());
        m_calibrationDock->setAlphaStep(settings.value("AlphaStep", 0.602).toDouble());
        m_calibrationDock->setThetaBias(settings.value("ThetaBias", 120).toDouble());
        m_calibrationDock->setBetaAngle(settings.value("BetaAngle", 0.0).toDouble());
        m_calibrationDock->setXiAngle(settings.value("XiAngle", 0.0).toDouble());
        m_calibrationDock->SetMinRange_mm(settings.value("MinRange", 150.0).toDouble());
        m_calibrationDock->SetMaxRange_mm(settings.value("MaxRange", 40000.0).toDouble());

        // make sure l2lidar class is updated with current settings
        l2lidar.EnableCalibrationOVR(m_calibrationDock->isCalOVRenabled());
        l2lidar.SetRangeBiasOVR(m_calibrationDock->getRangeBias());
        l2lidar.SetRangeScaleOVR(m_calibrationDock->getRangeScale());
        l2lidar.SetAlphaAngleBiasOVR(m_calibrationDock->getAlphaBias());
        l2lidar.SetAlphaAngleStepOVR(m_calibrationDock->getAlphaStep());
        l2lidar.SetThetaAngleBiasOVR(m_calibrationDock->getThetaBias());
        l2lidar.SetBetaAngleOVR(m_calibrationDock->getBetaAngle());
        l2lidar.SetXiAngleOVR(m_calibrationDock->getXiAngle());
        l2lidar.SetMinRange_mm(m_calibrationDock->getMinRange_mm());
        l2lidar.SetMaxRange_mm(m_calibrationDock->getMaxRange_mm());

        m_calibrationDock->EnableRangeCorrection(settings.value("EnableRangeCorrection", false).toBool());
        m_calibrationDock->SetRangeCalFile(settings.value("RangeCalibrationFilename","").toString());
        l2lidar.EnableRangeCorrection(m_calibrationDock->isRangeCorrectionEnabled());

    settings.endGroup();

    // windows visibility
    settings.beginGroup("visibility");
        if(resetRequested){  // check if reset to the windows has been requested
            // reset settings to initial state, ignore past settings
            config.setPCviewerEnabled(false);
            config.setACKenabled(true);
            config.setDiagEnabled(true);
            config.setIMUenabled(true);
            config.setPacketRateChartEnabled(true);
            config.setStatsEnabled(true);
          } else {
            config.setPCviewerEnabled(settings.value("PCviewer", false).toBool());
            config.setACKenabled(settings.value("ACK", true).toBool());
            config.setDiagEnabled(settings.value("Diag", true).toBool());
            config.setIMUenabled(settings.value("IMU", true).toBool());
            config.setPacketRateChartEnabled(settings.value("PacketRateChart", true).toBool());
            config.setStatsEnabled(settings.value("Stats", true).toBool());
        }
    settings.endGroup();

    // point cloud view settings

    settings.beginGroup("PCview");
        defaultPCsettings.Distance =settings.value("Distance", 10.0).toDouble();
        defaultPCsettings.Yaw =settings.value("Yaw", 145.0).toDouble();
        defaultPCsettings.Pitch = settings.value("Pitch", 20.0).toDouble();
        defaultPCsettings.PointSize = settings.value("PointSize", 1.0).toDouble();
        defaultPCsettings.MinDistance = settings.value("MinDistance",0.1).toDouble();
        defaultPCsettings.MaxDistance = settings.value("MaxDistance",10.0).toDouble();

        config.setPCWdistance(defaultPCsettings.Distance);
        config.setPCWyaw(defaultPCsettings.Yaw);
        config.setPCWpitch(defaultPCsettings.Pitch);
        config.setPointSize(defaultPCsettings.PointSize);
        config.setMinDistance(defaultPCsettings.MinDistance);
        config.setMaxDistance(defaultPCsettings.MaxDistance);

        // point cloud buffering settings
        mmaxPoints=settings.value("MaxPoints", 450000).toUInt(); // 3D PC frame is 300 points
        config.setMaxPoints(mmaxPoints);

        bool IMUadjust = settings.value("IMUadjust", false).toBool();
        config.setIMUadjustEnabled(IMUadjust);
        l2lidar.EnableIMUadjust(IMUadjust);

        bool IMUadjustRollPitch = settings.value("IMUadjustRollPitch", false).toBool();
        config.setIMUadjustRollPitch(IMUadjustRollPitch);
        l2lidar.EnableAdjustRollPitchOnly(IMUadjustRollPitch);

        bool FlattenScanEnabled = settings.value("FlattenScanEnabled", false).toBool();
        config.setFlattenScanEnabled(FlattenScanEnabled);
        l2lidar.EnableFlattenScan(FlattenScanEnabled);

        double StartScanAngle = settings.value("StartScanAngle", 0.0).toDouble();
        config.setStartScanAngle(StartScanAngle);
        l2lidar.SetStartScanAngle(StartScanAngle);

        double ScanAngleWidth = settings.value("ScanAngleWidth", 360.0).toDouble();
        config.setScanAngleWidth(ScanAngleWidth);
        l2lidar.SetScanAngleWidth(ScanAngleWidth);

        double IMUPCtimeConstraint = settings.value("IMUPCtimeConstraint", 0.07).toDouble();
        config.setIMUPCtimeConstraint(IMUPCtimeConstraint);
        l2lidar.SetIMUPCtimeConstraint(IMUPCtimeConstraint);

        mNumFramestoAggregate = settings.value("NumFramestoAggregate", 38).toInt();
        config.setAggFrames(mNumFramestoAggregate);

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

//-----------------------------------------------------
// logging functions
// These are for diagnostic logging into a log file
//-----------------------------------------------------
void MainWindow::logParameter(const char *Name, double value)
{
    if(!mLogging) return;
    if(mLogFile ==NULL) {
        QByteArray filename = mLogFilename.toLocal8Bit();
        const char *cfilename = filename.data();
#ifdef _MSC_VER
        if(fopen_s(&mLogFile,cfilename,"w"))
            return;
#else
        mLogFile = fopen(cfilename,"w");
        if(!mLogFile)
            return;
#endif
    }
    fprintf(mLogFile,"%s: %.8g\n", Name, value);
}

void MainWindow::logParameter(const char *Name, bool value)
{
    if(!mLogging) return;
    if(mLogFile ==NULL) {
        QByteArray filename = mLogFilename.toLocal8Bit();
        const char *cfilename = filename.data();
        #ifdef _MSC_VER
            if(fopen_s(&mLogFile,cfilename,"w")) return;
        #else
            mLogFile = fopen(cfilename,"w");
            if(!mLogFile) return;
        #endif
    }
    if(value) {
        fprintf(mLogFile,"%s: true\n", Name);
    } else {
        fprintf(mLogFile,"%s: false\n", Name);
    }
}

void MainWindow::logParameter(const char *Name, float value)
{
    if(!mLogging) return;
    if(mLogFile ==NULL) {
        QByteArray filename = mLogFilename.toLocal8Bit();
        const char *cfilename = filename.data();
        #ifdef _MSC_VER
            if(fopen_s(&mLogFile,cfilename,"w")) return;
        #else
            mLogFile = fopen(cfilename,"w");
            if(!mLogFile) return;
        #endif
    }
    fprintf(mLogFile,"%s: %.8g\n", Name, value);
}

void MainWindow::logParameter(const char *Name, int value)
{
    if(!mLogging) return;
    if(mLogFile ==NULL) {
        QByteArray filename = mLogFilename.toLocal8Bit();
        const char *cfilename = filename.data();
        #ifdef _MSC_VER
            if(fopen_s(&mLogFile,cfilename,"w")) return;
        #else
            mLogFile = fopen(cfilename,"w");
            if(!mLogFile) return;
        #endif
    }
    fprintf(mLogFile,"%s: %d\n", Name, value);
}

void MainWindow::logParameter(const char *Name, QString text)
{
    if(!mLogging) return;
    if(mLogFile ==NULL) {
        QByteArray filename = mLogFilename.toLocal8Bit();
        const char *cfilename = filename.data();
        #ifdef _MSC_VER
            if(fopen_s(&mLogFile,cfilename,"w")) return;
        #else
            mLogFile = fopen(cfilename,"w");
            if(!mLogFile) return;
        #endif
    }
    QByteArray qvalue = text.toLocal8Bit();
    const char *value = qvalue.data();
    fprintf(mLogFile,"%s: %s\n", Name, value);
}
