//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: ConfigDialog.h
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
//  V0.2.1  2026-01-05  Added src and dst - ip and ports
//                      Removed the 'divider' which was never used
//                      removed nth packet loading which was never used
//  V0.2.4  2026-01-10  removed CSV checkbox, never fully implemented
//                      Added spinbox for setting skip frame value
//  V0.2.6  2026-01-14  Added window visibility settings, throttling setting
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

        // Reasonable defaults / limits
        ui.srcPortSpin->setRange(1, 65535);
        ui.dstPortSpin->setRange(1, 65535);

        ui.spinNframe->setRange(0,10);
        ui.spinDiagrate->setRange(200,500);
        ui.spinPCrate->setRange(25,200);
        ui.spinPacketrate->setRange(100,500);
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

    uint32_t getPCupdateRate() const
    {
        return static_cast<uint32_t>(ui.spinPCrate->value());
    }

     uint32_t getPacketUpdateRate() const
    {
        return static_cast<uint32_t>(ui.spinPacketrate->value());
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

    // =============================
    // Setters (for LoadSettings)
    // =============================

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

    void setPCupdateRate(uint16_t p) const
    {
        ui.spinPCrate->setValue(p);
    }

    void setPacketUpdateRate(uint16_t p) const
    {
        ui.spinPacketrate->setValue(p);
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


private:
    Ui::ConfigDialog ui;
};
