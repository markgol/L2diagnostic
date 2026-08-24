//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalGraphDock.cpp
//
//
//  Purpose:
//  This is for the range calibration GUI
//
//  V2.0.0 RC1 2026-08-02
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
#include <QString>
#include "CalGraphDock.h"
#include "RangeCalibrationGraphicsView.h"
#include "ui_CalGraphDock.h"
//--------------------------------------------------------
// DiagnosticsDock constructor
//--------------------------------------------------------
//--------------------------------------------------------
// CalGraphDock constructor
//--------------------------------------------------------
CalGraphDock::CalGraphDock(QWidget* parent)
    : QDockWidget(parent)
    , ui(new Ui::CalGraphDock)
{
    ui->setupUi(this);

    mGraphicsView =
        new RangeCalibrationGraphicsView(
            ui->graphicsContainer);

    ui->graphicsLayout->addWidget(
        mGraphicsView);
    //
    // connect buttons
    //

    // select button
    connect(ui->selectButton, &QToolButton::clicked,
            this, &CalGraphDock::SelectButtonClicked);

    // add line(segment) button
    connect(ui->addSegmentButton, &QToolButton::clicked,
            this, &CalGraphDock::AddLineButtonClicked);

    // add exclusion(box) button
    connect(ui->addExclusionButton, &QToolButton::clicked,
            this, &CalGraphDock::AddBoxButtonClicked);

    // okay button
    connect(ui->buttonBox,&QDialogButtonBox::accepted,
           this, &CalGraphDock::Okay);

    // cancel button
    connect(ui->buttonBox,&QDialogButtonBox::rejected,
            this, &CalGraphDock::Cancel);

    // received action to change edit mode from GUI
    connect(
        mGraphicsView,
        &RangeCalibrationGraphicsView::EditModeChanged,
        this, [this](EditMode mode)
        {
            ui->selectButton->setChecked(mode == EditMode::Select);
            ui->addSegmentButton->setChecked(mode == EditMode::AddSegment);
            ui->addExclusionButton->setChecked(mode == EditMode::AddExclusion);
        });

    // received action SegmentConflictStateChanged from GUI
    connect(mGraphicsView, &RangeCalibrationGraphicsView::SegmentConflictStateChanged,
            this, &CalGraphDock::OnSegmentConflictStateChanged);

    mGraphicsView->SetEditMode(EditMode::AddExclusion);
    UpdateGUIState();
}

//--------------------------------------------------------
//  closeEvent deconstructor
//  ensure window gets close when application closes
//--------------------------------------------------------
void CalGraphDock::closeEvent(QCloseEvent* e)
{
    e->accept();
}

//--------------------------------------------------------
// DiagnosticsDock destructor
//--------------------------------------------------------
CalGraphDock::~CalGraphDock()
{
    delete ui;
}

//--------------------------------------------------------
// SetPoints
//--------------------------------------------------------
void CalGraphDock::SetPoints(
    const std::vector<Stage3BPoint>& points)
{
    mGraphicsView->SetPoints(points);
}

//--------------------------------------------------------
// ClearGUI_SetPoints
//--------------------------------------------------------
void CalGraphDock::ClearGUI_SetPoints(
    const std::vector<Stage3BPoint>& points)
{
    mGraphicsView->ClearGUI_SetPoints(points);
}

//--------------------------------------------------------
// SetCalibrationSegments
//--------------------------------------------------------
void CalGraphDock::SetCalibrationSegments(const std::vector<CalibrationSegment>& segments)
{
    mGraphicsView->SetCalibrationSegments(segments);
}

//--------------------------------------------------------
// SetExclusionRegions
//--------------------------------------------------------
void CalGraphDock::SetExclusionRegions(const std::vector<ExclusionRegion>& regions)
{
    mGraphicsView->SetExclusionRegions(regions);
}

//--------------------------------------------------------
// GetCalibrationSegments
//--------------------------------------------------------
const std::vector<CalibrationSegment>& CalGraphDock::GetCalibrationSegments()
{
    return mGraphicsView->GetCalibrationSegments();
}

//--------------------------------------------------------
// GetExclusionRegions
//--------------------------------------------------------
const std::vector<ExclusionRegion>& CalGraphDock::GetExclusionRegions()
{
    return mGraphicsView->GetExclusionRegions();
}

//--------------------------------------------------------
// SetExclusionRegions
//--------------------------------------------------------
void CalGraphDock::SelectButtonClicked()
{
    mGraphicsView->SetEditMode(EditMode::Select);
}

//--------------------------------------------------------
// SetExclusionRegions
//--------------------------------------------------------
void CalGraphDock::AddLineButtonClicked()
{
    mGraphicsView->SetEditMode(EditMode::AddSegment);
}

//--------------------------------------------------------
// SetExclusionRegions
//--------------------------------------------------------
void CalGraphDock::AddBoxButtonClicked()
{
    mGraphicsView->SetEditMode(EditMode::AddExclusion);
}

//--------------------------------------------------------
// Accepted
//--------------------------------------------------------
void CalGraphDock::Okay()
{
    emit Accepted();
}

//--------------------------------------------------------
// Rejected
//--------------------------------------------------------
void CalGraphDock::Cancel() {
    emit Rejected();
}

//--------------------------------------------------------
// OnSegmentConflictStateChanged
//--------------------------------------------------------
void CalGraphDock::OnSegmentConflictStateChanged(
    bool conflictExists)
{
    mSegmentConflictExists = conflictExists;
    ui->lblWarnings->setVisible(conflictExists);

    UpdateGUIState();
}

//--------------------------------------------------------
// UpdateGUIState
//--------------------------------------------------------
void CalGraphDock::UpdateGUIState()
{
    ui->lblWarnings->setVisible(
        mSegmentConflictExists);

    if (mSegmentConflictExists) {
        ui->lblWarnings->setText(
            "Warning: Segment conflict detected");
    } else {
        ui->lblWarnings->clear();
    }

    QPushButton* okButton = ui->buttonBox->button(QDialogButtonBox::Ok);

    if (okButton) {
        okButton->setEnabled(!mSegmentConflictExists);
    }
}
