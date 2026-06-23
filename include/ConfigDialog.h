//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ConfigDialog.h
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
//  V0.2.1  2026-01-05  Added src and dst - ip and ports
//                      Removed the 'divider' which was never used
//                      removed nth packet loading which was never used
//  V0.2.4  2026-01-10  removed CSV checkbox, never fully implemented
//                      Added spinbox for setting skip frame value
//  V0.2.6  2026-01-14  Added window visibility settings, throttling setting
//  V0.3.5  2026-01-24  With change in renderer architecture
//                      removed Max # frame
//                      added buffer size instead
//                      added cloud point size
//  V0.3.7  2026-01-26  Added set UDP configuration in L2
//  V0.3.9  2026-01-31  Added L2 timebase and sync controls
//  V0.3.10 2026-02-02  Added enable latency measurement checkbobx
//                      Adjusted sizing of ControlsDock and ConfigDialog
//                          to adjust for use on Ubuntu x64 and ARM64 platforms
//  V0.4.0  2026-02-11  Added set L2 MAC address
//  V0.4.3  2026-02-20  Added 'n' frame point cloud frame aggegration
//                      0 is no aggregation, 38 matches one hemishpere scan
//  V1.0.0  2026-03-28  Offical release
//  V1.1.0  2026-04-20  Added override of range calibration params
//  V1.2.1  2026-05-24  Added parameter for time constraint for IMU to PC match
//  V1.3.0  2026-06-18  Added flag for IMU adjust just to roll, pitch vs roll, pitch, yaw
//                      Added Numerator/Denominator scaling for time correction
//                      Added Use system time for packet timestamps checkbox
//  V1.3.1  2026-06-21  Added config params for settings gatewey IP address and subnet mask
//                      Added save current view to default view
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

#pragma once

#include <QDialog>
#include <QString>
#include "ui_ConfigDialog.h"

class ConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConfigDialog(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        ui.setupUi(this);

        // OK / Cancel wiring
        connect(ui.buttonBox, &QDialogButtonBox::accepted,
                this, &QDialog::accept);
        connect(ui.buttonBox, &QDialogButtonBox::rejected,
                this, &QDialog::reject);

        // Set View button
        connect(ui.btnSetView, &QPushButton::clicked,
                this, &ConfigDialog::ResetPCview);

        // Set to current View button
        connect(ui.btnUseCurrentView, &QPushButton::clicked,
                this, &ConfigDialog::CurrentPCview);

        // Configure L2 UDP
        connect(ui.btnConfigureUDP, &QPushButton::clicked,
                this, &ConfigDialog::ConfigureUDP);

        // Get L2 UDP config
        connect(ui.btnGetL2config, &QPushButton::clicked,
                this, &ConfigDialog::SetL2MAC);

        // Reasonable defaults / limits
        // UDP ports
        ui.srcPortSpin->setRange(1, 65535);
        ui.dstPortSpin->setRange(1, 65535);

        // timers
        // range is set in .ui file

        // Point Cloud Controls
        // range is set in .ui file
    }

    // =============================
    // Getters
    // =============================

    //--------------------------------------------------------
    // Ehternet Application UDP parameters
    //--------------------------------------------------------
    QString getSRCip() const
    {
        return ui.srcIPedit->text().trimmed();
    }

    QString getDSTip() const
    {
        return ui.dstIPedit->text().trimmed();
    }

    QString getGatewayip() const
    {
        return ui.gatewayIPedit->text().trimmed();
    }

    QString getSubnetMask() const
    {
        return ui.subnetMaskedit->text().trimmed();
    }

    uint16_t getSRCport() const
    {
        return static_cast<uint16_t>(ui.srcPortSpin->value());
    }

    uint32_t getDSTport() const
    {
        return static_cast<uint32_t>(ui.dstPortSpin->value());
    }

    QString GetMAC() const
    {
        return ui.editMAC->text().trimmed();
    }

    //--------------------------------------------------------
    // Application throttling parameters
    //--------------------------------------------------------
    uint32_t getSkipFrame() const
    {
        return static_cast<uint32_t>(ui.spinNframe->value());
    }

    int getDiagUpdateRate() const
    {
        return static_cast<int>(ui.spinDiagrate->value());
    }

     int getPacketUpdateRate() const
    {
        return static_cast<int>(ui.spinPacketrate->value());
    }

    int getRenderRate() const
    {
        return static_cast<int>(ui.spinRenderRate->value());
    }

    //--------------------------------------------------------
    // Application window visibilty
    //--------------------------------------------------------
    bool isPCviewerEnabled()
    {
        return static_cast<bool>(ui.cbPCviewer->isChecked());
    }

    bool isPacketRateChartEnabled()
    {
        return static_cast<bool>(ui.cbPacketRate->isChecked());
    }

    bool isACKenabled()
    {
        return static_cast<bool>(ui.cbACK->isChecked());
    }

    bool isDiagEnabled()
    {
        return static_cast<bool>(ui.cbDiag->isChecked());
    }

    bool isStatsEnabled()
    {
        return static_cast<bool>(ui.cbStats->isChecked());
    }

    bool isIMUenabled()
    {
        return static_cast<bool>(ui.cbIMU->isChecked());
    }

    //--------------------------------------------------------
    // point cloud viewer buffering
    //--------------------------------------------------------
    int getAggFrames()
    {
        return static_cast<int>(ui.spinAggFrame->value());
    }

    int getMaxPoints()
    {
        return static_cast<int>(ui.spinMaxPoints->value());
    }

    bool isIMUadjustEnabled()
    {
        return static_cast<bool>(ui.cbIMUadjust->isChecked());
    }

    bool isIMUadjustRollPitch()
    {
        return static_cast<bool>(ui.cbIMUadjustRP->isChecked());
    }

    float getIMUPCtimeConstraint() const
    {
        return static_cast<float>(ui.spinTM2PC->value());
    }

    //--------------------------------------------------------
    // point cloud viewer settings
    //--------------------------------------------------------

    float getPCWdistance() const
    {
        return static_cast<float>(ui.spinDistance->value());
    }

    float getPCWyaw() const
    {
        return static_cast<float>(ui.spinYaw->value());
    }

    float getPCWpitch() const
    {
        return static_cast<float>(ui.spinPitch->value());
    }

    float getPointSize() const
    {
        return static_cast<float>(ui.spinPointSize->value());
    }

    float getMinDistance() const
    {
        return static_cast<float>(ui.spinColorMin->value());
    }

    float getMaxDistance() const
    {
        return static_cast<float>(ui.spinColorMax->value());
    }

    //--------------------------------------------------------
    // L2 timebase correction controls
    //--------------------------------------------------------
    bool isL2TimeCorrectionEnabled()
    {
        return static_cast<bool>(ui.cbL2TSCorrect->isChecked());
    }

    uint32_t getL2TscaleNum() const
    {
        return static_cast<uint32_t>(ui.spinTSscaleNum->value());
    }

    uint32_t getL2TscaleDen() const
    {
        return static_cast<uint32_t>(ui.spinTSscaleDen->value());
    }

    bool isL2TsyncHostEnabled()
    {
        return static_cast<bool>(ui.cbL2syncHost->isChecked());
    }

    uint32_t getL2syncRate() const
    {
        return static_cast<uint32_t>(ui.spinL2syncRate->value());
    }

    bool isLatencyEnabled() const
    {
        return static_cast<bool>(ui.cbLatency->isChecked());
    }

    bool isUseSystemNowEnabled() const
    {
        return static_cast<bool>(ui.cbUseSytemNow->isChecked());
    }

    //--------------------------------------------------------
    // Calibration override
    //--------------------------------------------------------
    bool isCalOVRenabled()
    {
        return static_cast<bool>(ui.cbCalOVR->isChecked());
    }

    double getCalScale() const
    {
        return static_cast<double>(ui.spinCalScale->value());
    }

    double getCalBias() const
    {
        return static_cast<double>(ui.spinCalBias->value());
    }

    // =============================
    // Setters (for LoadSettings)
    // =============================

    // this set the range on the point size spin button
    void setPointSizeRange(float *PointSizeRange);

    //--------------------------------------------------------
    // Ehternet Application UDP parameters
    //--------------------------------------------------------
    void setSRCip(const QString& ip)
    {
        ui.srcIPedit->setText(ip);
    }

    void setDSTip(const QString& ip)
    {
        ui.dstIPedit->setText(ip);
    }

    void setGatewayip(const QString& ip)
    {
        ui.gatewayIPedit->setText(ip);
    }

    void setSubnetMask(const QString& mask)
    {
        ui.subnetMaskedit->setText(mask);
    }

    void setSRCport(uint16_t p)
    {
        ui.srcPortSpin->setValue(p);
    }

    void setDSTport(uint32_t p)
    {
        ui.dstPortSpin->setValue(p);
    }

    void SetMAC(const QString& l2MAC)
    {
        ui.editMAC->setText(l2MAC);
    }

    //--------------------------------------------------------
    // Application throttling parameters
    //--------------------------------------------------------
    void setSkipFrame(uint16_t p)
    {
        ui.spinNframe->setValue(p);
    }

    void setDiagUpdateRate(uint16_t p) const
    {
        ui.spinDiagrate->setValue(p);
    }

    void setPacketUpdateRate(uint16_t p) const
    {
        ui.spinPacketrate->setValue(p);
    }

    void setRenderRate(uint16_t p) const
    {
        ui.spinRenderRate->setValue(p);
    }

    //--------------------------------------------------------
    // point cloud viewer buffering
    //--------------------------------------------------------

    void setMaxPoints(uint32_t p)
    {
        ui.spinMaxPoints->setValue(p);
    }

    void setAggFrames(int p)
    {
        ui.spinAggFrame->setValue(p);
    }

    //--------------------------------------------------------
    // point cloud viewer settings
    //--------------------------------------------------------

    void setPCWdistance(float p) const
    {
        ui.spinDistance->setValue(p);
    }

    void setPCWyaw(float p) const
    {
        ui.spinYaw->setValue(p);
    }

    void setPCWpitch(float p) const
    {
        ui.spinPitch->setValue(p);
    }

    void setIMUadjustEnabled(bool p)
    {
        ui.cbIMUadjust->setChecked(p);
    }

    void setIMUadjustRollPitch(bool p)
    {
        ui.cbIMUadjustRP->setChecked(p);
    }

    void setIMUPCtimeConstraint(float TimeConstraint)
    {
        ui.spinTM2PC->setValue(TimeConstraint);
    }

    void setUseSystemNow(bool p)
    {
        ui.cbUseSytemNow->setChecked(p);
    }

    void setPointSize(float PointSize)
    {
        ui.spinPointSize->setValue(PointSize);
    }

    void setMinDistance(float MinDistance)
    {
        ui.spinColorMin->setValue(MinDistance);
    }

    void setMaxDistance(float MaxDistance)
    {
        ui.spinColorMax->setValue(MaxDistance);
    }

    //--------------------------------------------------------
    // Application window visibilty
    //--------------------------------------------------------
    void setPCviewerEnabled(bool p)
    {
        ui.cbPCviewer->setChecked(p);
    }

    void setPacketRateChartEnabled(bool p)
    {
        ui.cbPacketRate->setChecked(p);
    }

    void setACKenabled(bool p)
    {
        ui.cbACK->setChecked(p);
    }

    void setDiagEnabled(bool p)
    {
        ui.cbDiag->setChecked(p);
    }

    void setStatsEnabled(bool p)
    {
        ui.cbStats->setChecked(p);
   }

    void setIMUenabled(bool p)
    {
        ui.cbIMU->setChecked(p);
    }

    //--------------------------------------------------------
    // L2 timebase correction controls
    //--------------------------------------------------------
    void SetL2TimeCorrectionEnabled(bool p)
    {
        ui.cbL2TSCorrect->setChecked(p);
    }

    void setL2TscaleNum(uint32_t p)
    {
        ui.spinTSscaleNum->setValue(p);
    }

    void setL2TscaleDen(uint32_t p)
    {
        ui.spinTSscaleDen->setValue(p);
    }

    void SetL2TsyncHostEnabled(bool p)
    {
        ui.cbL2syncHost->setChecked(p);
    }

    void setL2syncRate(uint32_t p) const
    {
        ui.spinL2syncRate->setValue(p);
    }

    void setEnableLatency(bool p) const
    {
        ui.cbLatency->setChecked(p);
    }

    //--------------------------------------------------------
    // Calibration override
    //--------------------------------------------------------
    void SetCalOVRenabled(bool p)
    {
        ui.cbCalOVR->setChecked(p);
    }

    void SetCalScale(double p) const
    {
        ui.spinCalScale->setValue(p);
    }

    void setCalBias(double p) const
    {
        ui.spinCalBias->setValue(p);
    }

signals:
    void requestViewReset();
    void requestCurrentPCview();
    void requestConfigureUDP();
    void requestSetL2MAC();


private slots:
    void ResetPCview();
    void CurrentPCview();
    void ConfigureUDP();
    void SetL2MAC();


private:
    Ui::ConfigDialog ui;
};
