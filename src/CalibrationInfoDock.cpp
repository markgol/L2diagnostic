//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalibrationInfoDock.cpp
//
//
//  Purpose:
//  display information for a calibration file
//
//  V2.0.0 RC1 2026-08-02
//  V2.0.1  2026-08-24  This is the intial V2.x release
//                      Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Some cleanup of the UI and GUI interactions
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection file
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection file
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

#include "CalibrationInfoDock.h"
#include "ui_CalibrationInfoDock.h"

#include <QString>

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
CalibrationInfoDock::CalibrationInfoDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::CalibrationInfoDock)
{
    ui->setupUi(this);
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
CalibrationInfoDock::~CalibrationInfoDock()
{
    delete ui;
}

//--------------------------------------------------------
//  updateStats
//  signal callback to update Stats window
//--------------------------------------------------------
void CalibrationInfoDock::updateInfo(const CalibrationInfo& info, bool valid)
{
    // Calibration
    QString ResultString;

    if(valid) {
        ui->lblLoadStatus->setText("Yes");
    } else {
        ui->lblLoadStatus->setText("No");
    }

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.Version).toUtf8().constData());
    ui->lblVersion->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.Date).toUtf8().constData());
    ui->lblDate->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.Sensor).toUtf8().constData());
    ui->lblSensor->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.SensorID).toUtf8().constData());
    ui->lblSensorID->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.Firmware).toUtf8().constData());
    ui->lblFirmware->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.CreatedBy).toUtf8().constData());
    ui->lblCreatedBy->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.RangeCalMethod).toUtf8().constData());
    ui->lblCalMethod->setText(ResultString);

    ResultString = ResultString.asprintf("%s",QString::fromStdString(info.CalibrationDescription).toUtf8().constData());
    ui->lblCalDesc->setText(ResultString);

    ResultString = ResultString.asprintf("%i mm",info.RangeBias);
    ui->lblRangeBias->setText(ResultString);

    ResultString = ResultString.asprintf("%.6f",info.RangeScale);
    ui->lblRangeScale->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f deg",info.AlphaAngleBias);
    ui->lblAlphaAngleBias->setText(ResultString);

    ResultString = ResultString.asprintf("%.4f deg",info.AlphaAngleStepSize);
    ui->lblAlphaAngleStep->setText(ResultString);

    ResultString = ResultString.asprintf("%.2f deg",info.ThetaAngleBias);
    ui->lblThetaAngleBias->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f deg",info.BetaAngle);
    ui->lblBetaAngle->setText(ResultString);

    ResultString = ResultString.asprintf("%.3f deg",info.XiAngle);
    ui->lblXiAngle->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f mm",info.MinRange);
    ui->lblMinRange->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f mm",info.MaxRange);
    ui->lblMaxRange->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f mm",info.MinTrustedRange);
    ui->lblMinTrustedRange->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f mm",info.MinCalRange);
    ui->lblMinCalRange->setText(ResultString);

    ResultString = ResultString.asprintf("%8.2f mm",info.MaxCalRange);
    ui->lblMaxCalRange->setText(ResultString);

    ResultString = ResultString.asprintf("%g mm",info.RMSResidual);
    ui->lblRMSresidual->setText(ResultString);

    ResultString = ResultString.asprintf("%u",info.NumberOfSegments);
    ui->lblNumSegments->setText(ResultString);

}

//--------------------------------------------------------
//  updateStats
//  signal callback to update Stats window
//--------------------------------------------------------
void CalibrationInfoDock::SetMessage(const std::string& message)
{
    QString ResultString;

    ResultString = ResultString.asprintf("%s",QString::fromStdString(message).toUtf8().constData());
    ui->lblMessages->setText(ResultString);
}
