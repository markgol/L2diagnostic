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
        ui.dstPortSpin->setRange(1, 1'000'000);
    }

    // =============================
    // Getters
    // =============================
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

    bool csvEnabled() const
    {
        return ui.csvEnable->isChecked();
    }

    // =============================
    // Setters (for LoadSettings)
    // =============================
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

    void setCsvEnabled(bool enabled)
    {
        ui.csvEnable->setChecked(enabled);
    }

private:
    Ui::ConfigDialog ui;
};
