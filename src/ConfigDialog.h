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
//  V.3.7   2026-01-26  Added set UDP configuration in L2
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

        // Reset View button
        connect(ui.btnSetView, &QPushButton::clicked,
                this, &ConfigDialog::ResetPCview);

        // Configure L2 UDP
        connect(ui.btnConfigureUDP, &QPushButton::clicked,
                this, &ConfigDialog::ConfigureUDP);

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


    uint16_t getSRCport() const
    {
        return static_cast<uint16_t>(ui.srcPortSpin->value());
    }

    uint32_t getDSTport() const
    {
        return static_cast<uint32_t>(ui.dstPortSpin->value());
    }

    //--------------------------------------------------------
    // Application throttling parameters
    //--------------------------------------------------------
    uint32_t getSkipFrame() const
    {
        return static_cast<uint32_t>(ui.spinNframe->value());
    }

    uint32_t getDiagUpdateRate() const
    {
        return static_cast<uint32_t>(ui.spinDiagrate->value());
    }

     uint32_t getPacketUpdateRate() const
    {
        return static_cast<uint32_t>(ui.spinPacketrate->value());
    }

    uint32_t getRenderRate() const
    {
        return static_cast<uint32_t>(ui.spinRenderRate->value());
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
    uint32_t getMaxPoints()
    {
        return static_cast<uint32_t>(ui.spinMaxPoints->value());
    }
    bool isIMUadjustEnabled()
    {
        return static_cast<bool>(ui.cbIMUadjust->isChecked());
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

    void setSRCport(uint16_t p)
    {
        ui.srcPortSpin->setValue(p);
    }

    void setDSTport(uint32_t p)
    {
        ui.dstPortSpin->setValue(p);
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

signals:
    void requestViewReset();
    void requestConfigureUDP();


private slots:
    void ResetPCview();
    void ConfigureUDP();


private:
    Ui::ConfigDialog ui;
};
