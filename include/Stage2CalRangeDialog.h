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
//  V2.0.0 RC1 2026-08-18
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
#pragma once

#include <QDialog>
#include <QString>
#include <QSettings>
#include <QStandardPaths>
#include <QTimer>
#include "L2lidar.h"
#include "ui_Stage2CalRangeDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Stage2CalRangeDialog;
}
QT_END_NAMESPACE

class Stage2CalRangeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Stage2CalRangeDialog(L2lidar& lidar, QWidget* parent = nullptr);
    ~Stage2CalRangeDialog();

    void SetStartAngle(double startangle) {
        ui->spinStartScanAngle->setValue(startangle);
    };

    void SetAngleWidth(double anglewidth) {
        ui->spinScanAngleWidth->setValue(anglewidth);
    };

    void SetNumScans(int numscans) {
        ui->spinNumAcqPoints->setValue(numscans);
    };

    double GetStartAngle() {
        return ui->spinStartScanAngle->value();
    };

    double GetAngleWidth() {
        return ui->spinScanAngleWidth->value();
    };

    uint32_t GetNumScans() {
        int32_t numscans = ui->spinNumAcqPoints->value();
        if(numscans<0) numscans = 0;
        return numscans;
    };

    bool GetACQsaved() {return mSaved;}
    void ClearSaved() {mSaved = false;}
    bool IsACQrunning() {return mACQrunning;}

signals:
    void SavePC();
    void ClearPCwindowRequested();
    void Finished();

private:  // functions
    void View3D();
    void ViewFull3D();
    void ViewFlat();
    void StartAcq();
    void SaveAcq();
    void Exit();
    void UpdateScanCount();
    QTimer ScanAcqUpdateTimer;

private:  // variables
    Ui::Stage2CalRangeDialog* ui;
    L2lidar& ml2lidar;
    bool mSaved {false};
    bool mNowFlat {false};
    bool mACQrunning {false};
};
