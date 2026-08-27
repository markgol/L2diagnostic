//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage2CalRangeDialog.h
//
//
//  Purpose:
//  Stage 2 range correction calibration for the L2
//  Metadata entry
//
//  V2.0.0 RC1 2026-07-30
//  V2.0.1  2026-08-24  This is the intial V2.x release
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
// This is the Stage 2 acqusition dialog
//--------------------------------------------------------
#include "Stage2CalRangeDialog.h"
#include "settingINI.h"

//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
Stage2CalRangeDialog::Stage2CalRangeDialog(L2lidar& lidar, QWidget* parent)
    : QDialog(parent),
    ml2lidar(lidar),
    ui(new Ui::Stage2CalRangeDialog)
{
    ui->setupUi(this);

    // View full 3D button
    connect(ui->btnViewFull3D, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::ViewFull3D);

    // View 3D button
    connect(ui->btnView3D, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::View3D);

    // View flat button
    connect(ui->btnViewFlat, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::ViewFlat);

    // Start Acq button
    connect(ui->btnStart, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::StartAcq);

    // Save Acq button
    connect(ui->btnSaveAcq, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::SaveAcq);

    // check all spin buttons for change
    // if changed reset capture sequence
    connect(ui->spinScanAngleWidth, &QDoubleSpinBox::valueChanged,
            this, &Stage2CalRangeDialog::View3D);

    connect(ui->spinStartScanAngle, &QDoubleSpinBox::valueChanged,
            this, &Stage2CalRangeDialog::View3D);

    // EXIT button
    connect(ui->btnExit, &QPushButton::clicked,
            this, &Stage2CalRangeDialog::Exit);

    // create timer to show number of scans collected
    // setup host sync timer
    connect(&ScanAcqUpdateTimer, &QTimer::timeout,
            this, &Stage2CalRangeDialog::UpdateScanCount);
    ScanAcqUpdateTimer.setInterval(100); // 10 times a seconds
    //ScanAcqUpdateTimer.start();
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
Stage2CalRangeDialog::~Stage2CalRangeDialog()
{
    delete ui;
}

//--------------------------------------------------------
// Exit
//--------------------------------------------------------
void Stage2CalRangeDialog::Exit()
{
    ui->btnViewFlat->setEnabled(false);
    ui->btnStart->setEnabled(false);
    ui->btnSaveAcq->setEnabled(false);
    mACQrunning = false;

    // save parameters
    saveINI("RangeCal","StartAngle", ui->spinStartScanAngle->value());
    saveINI("RangeCal","AngleWidth", ui->spinScanAngleWidth->value());
    saveINI("RangeCal","NumPoints", ui->spinNumAcqPoints->value());

    //restore saved pc configuration
    //restore normal operation
    emit Finished();

    mNowFlat = false;
    ScanAcqUpdateTimer.stop();

    // clear PC
    emit ClearPCwindowRequested();

    // hide this window
    this->hide();
}

//--------------------------------------------------------
// Exit
//--------------------------------------------------------
void Stage2CalRangeDialog::UpdateScanCount()
{
    if(mNowFlat) {
        auto NumScan = ml2lidar.NumPointsConverted();
        QString counter = QString().asprintf("%llu", NumScan);
        ui->lblNumPoints->setText(counter);
        if(ml2lidar.NumPointsConverted() < ui->spinNumAcqPoints->value()) {
            ui->btnSaveAcq->setEnabled(false);
        } else {
            ui->btnSaveAcq->setEnabled(true);

        }
    } else {
        ui->lblNumPoints->setText("0");
    }
}

//--------------------------------------------------------
// View3D
//--------------------------------------------------------
void Stage2CalRangeDialog::ViewFull3D()
{
    mACQrunning = false;

    // set scan angle
    double scanangle = ui->spinStartScanAngle->value();
    ml2lidar.SetStartScanAngle(scanangle);

    // set angle width
    double anglewidth = 360.0;
    ml2lidar.SetScanAngleWidth(anglewidth);

    // disable flat
    ml2lidar.EnableFlattenScan(false);

    // enable View View3D, clear SaveACQ, clear Start buttons
    ui->btnViewFlat->setEnabled(true);
    ui->btnView3D->setEnabled(true);
    ui->btnSaveAcq->setEnabled(false);
    ui->btnStart->setEnabled(false);

    // reset scan counter
    ml2lidar.ClearNumPointsConverted();
    ScanAcqUpdateTimer.stop();

    // clear PC
    emit ClearPCwindowRequested();

    mNowFlat = false;
}

//--------------------------------------------------------
// View3D
//--------------------------------------------------------
void Stage2CalRangeDialog::View3D()
{
    mACQrunning = false;

    // set scan angle
    double scanangle = ui->spinStartScanAngle->value();
    ml2lidar.SetStartScanAngle(scanangle);

    // set angle width
    double anglewidth = ui->spinScanAngleWidth->value();
    ml2lidar.SetScanAngleWidth(anglewidth);

    // disable flat
    ml2lidar.EnableFlattenScan(false);

    // enable View Flat
    ui->btnViewFlat->setEnabled(true);
    ui->btnSaveAcq->setEnabled(false);
    ui->btnStart->setEnabled(false);

    // reset scan counter
    ml2lidar.ClearNumPointsConverted();
    ScanAcqUpdateTimer.stop();

    // clear PC
    emit ClearPCwindowRequested();

    mNowFlat = false;
}

//--------------------------------------------------------
// ViewFlat
//--------------------------------------------------------
void Stage2CalRangeDialog::ViewFlat()
{
    mACQrunning = false;

    // set scan angle
    double scanangle = ui->spinStartScanAngle->value();
    ml2lidar.SetStartScanAngle(scanangle);

    // set angle width
    double anglewidth = ui->spinScanAngleWidth->value();
    ml2lidar.SetScanAngleWidth(anglewidth);

    // set RangeScale to 0.001
    ml2lidar.SetRangeScaleOVR(0.001);

    // enable flat
    ml2lidar.EnableFlattenScan(true);

    ui->btnStart->setEnabled(true);

    // reset scan counter
    ml2lidar.ClearNumPointsConverted();
    ScanAcqUpdateTimer.start();
    // clear PC
    emit ClearPCwindowRequested();

    mNowFlat = true;
}

//--------------------------------------------------------
// StartAcq
//--------------------------------------------------------
void Stage2CalRangeDialog::StartAcq()
{
    // set scan angle
    double scanangle = ui->spinStartScanAngle->value();
    ml2lidar.SetStartScanAngle(scanangle);

    // set angle width
    double anglewidth = ui->spinScanAngleWidth->value();
    ml2lidar.SetScanAngleWidth(anglewidth);

    // set RangeScale to 0.001
    ml2lidar.SetRangeScaleOVR(0.001);

    // enable flat
    ml2lidar.EnableFlattenScan(true);

    // update scan acquisition count
    ScanAcqUpdateTimer.start();
    // clear PC
    emit ClearPCwindowRequested();

    mSaved = false;
    mACQrunning = true;

    // reset scan counter
    ml2lidar.ClearNumPointsConverted();
}

//--------------------------------------------------------
// SaveAcq
//--------------------------------------------------------
void Stage2CalRangeDialog::SaveAcq()
{
    // genaralize to capture verify user did not cancel
    emit Stage2SavePC();
}
