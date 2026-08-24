//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage3CalRangeDialog.h
//
//
//  Purpose:
//  Stage 3 data analysis and extraction
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
// This is the Stage 3 data analysis and extraction dialog
//--------------------------------------------------------
#pragma once

#include <QDialog>
#include <QString>
#include <QDialog>
#include <QVector>
#include <QFileDialog>
#include <QMessageBox>
#include "PCfileIO.h"
#include "L2RangeAnalysis.h"
#include "L2RangeExtraction.h"
#include "ui_Stage3CalRangeDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Stage3CalRangeDialog;
}
QT_END_NAMESPACE

class Stage3CalRangeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Stage3CalRangeDialog(QWidget* parent = nullptr);
    ~Stage3CalRangeDialog();

    void SetAlphaAngleBias(double p) {mCal.alphaAngleBias = p;}
    void SetBetaAngle(double p) {mCal.betaAngle = p;}
    void SetXiAngle(double p) {mCal.xiAngle = p;}
    void SetAlphaAngleStep(double p) {mCal.alphaAngleStepSize = p;}

    void SetMinRange_m(double p) {mMinRange_m = p;} // units are meters
    void SetMaxRange_m(double p) {mMaxRange_m = p;} // units are meters

    const std::vector<Stage3BPoint>& GetStage3BPoints() const noexcept
    {
        return mL2RangeExtraction.GetStage3BPoints();
    }

    void Stage3rejected();
    void Stage3accepted();

    void SetStage3CalSegments(const std::vector<CalibrationSegment>& segments) {
        mSegments = segments;
    }
    void SetStage3ExclusionRegions(const std::vector<ExclusionRegion>& regions) {
        mExclusionRegions = regions;
    }

    const std::vector<RangeCalibrationMeasurement>& GetCalMeasurements() {
        return mMeasurements;
    }

signals:
    void CalGUIrequest(bool clear, bool visible); // need to invoke the range calibration GUI

private: // functions
    bool LoadPC();
    void BrowseFile();
    void AnalyzePC();
    void BrowseCSVfile();
    void Exit();

private: // variables
    L2AngularCalibration mCal;
    Ui::Stage3CalRangeDialog* ui;
    QVector<GLPoint> mcloud;
    L2RangeAnalysis mRangeAnalysis;
    RangeAnalysisResult mAnalysisResult;
    L2RangeExtraction mL2RangeExtraction;
    std::vector<CalibrationSegment> mSegments;
    std::vector<ExclusionRegion> mExclusionRegions;
    std::vector<RangeCalibrationMeasurement> mMeasurements;

    double mMinRange_m {0.15};
    double mMaxRange_m {40.0};
};
