//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage1CalRangeDialog.h
//
//
//  Purpose:
//  Stage 1 range correction calibration for the L2
//  Metadata entry
//
//  V2.0.0 RC1 2026-08-18
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
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
// This is the Stage 1 meta data entry dialog
//--------------------------------------------------------
#pragma once

#include <QDialog>
#include <QString>
#include "ui_Stage1CalRangeDialog.h"

class Stage1CalRangeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Stage1CalRangeDialog(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        ui.setupUi(this);

        // OK / Cancel wiring
        connect(ui.buttonBox, &QDialogButtonBox::accepted,
                this, &QDialog::accept);
        connect(ui.buttonBox, &QDialogButtonBox::rejected,
                this, &QDialog::reject);
    }

    void SetVersion(std::string Version) {
        ui.lblSpecVersion->setText(QString::fromStdString(Version));
    }

    void SetDate(std::string date) {
        ui.lblDate->setText(QString::fromStdString(date));
    }

    void SetSensor(std::string sensor) {
        ui.lblSensor->setText(QString::fromStdString(sensor));
    }

    void SetSensorID(std::string id) {
        ui.EditSensorID->setText(QString::fromStdString(id));
    }

    void SetFirmware(std::string fw) {
        ui.lblFirmware->setText(QString::fromStdString(fw));
    }
    void SetCreatedBy(std::string cb) {
        ui.lblSoftwareID->setText(QString::fromStdString(cb));
    }
    void SetDescription(std::string desc) {
        ui.EditDesc->setText(QString::fromStdString(desc));
    }

    void SetMinRange_mm(int32_t minr) {
        ui.spinMinRange->setValue(minr);
    }

    void SetMaxRange_mm(int32_t maxr) {
        ui.spinMaxRange->setValue(maxr);
    }

    void SetMinTrustedRange_mm(int32_t p) {
        ui.spinMinTrustedRange->setValue(p);
    }

    std::string GetSensorID() {
        QString str = ui.EditSensorID->text();
        return str.toStdString();
    }

    std::string GetDescription() {
        QString str = ui.EditDesc->text();
        return str.toStdString();
    }

    int32_t GetMinRange_mm() {
        return ui.spinMinRange->value();
    }

    int32_t GetMinTrustedRange_mm() {
        return ui.spinMinTrustedRange->value();
    }

    int32_t GetMaxRange_mm() {
        return ui.spinMaxRange->value();
    }



signals:

private slots:

private:
    Ui::Stage1CalRangeDialog ui;
};
