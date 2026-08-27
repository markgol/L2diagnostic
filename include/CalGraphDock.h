//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalGraphDock.h
//
//
//  Purpose:
//  This is for the range calibration GUI
//
//  V2.0.0 RC1 2026-08-18
//  V2.0.1  2026-08-24  This is the intial V2.x release
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
#pragma once

#include <QDockWidget>
#include "RangeCalibrationGraphicsView.h"
#include "Stage3CalRangeDialog.h"

class RangeCalibrationGraphicsView;

QT_BEGIN_NAMESPACE
namespace Ui {
class CalGraphDock;
}
QT_END_NAMESPACE

class CalGraphDock : public QDockWidget
{
    Q_OBJECT

public:
    explicit CalGraphDock(QWidget* parent = nullptr);
    ~CalGraphDock();

    void SetPoints(const std::vector<Stage3BPoint>& points);
    void ClearGUI_SetPoints(const std::vector<Stage3BPoint>& points);
    void SetCalibrationSegments(const std::vector<CalibrationSegment>& segments);
    void SetExclusionRegions(const std::vector<ExclusionRegion>& regions);
    const std::vector<CalibrationSegment>& GetCalibrationSegments();
    const std::vector<ExclusionRegion>& GetExclusionRegions();

    RangeCalibrationGraphicsView* GetGraphicsView() const noexcept
    {
        return mGraphicsView;
    }

signals:
    void Accepted();
    void Rejected();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void SelectButtonClicked();
    void AddLineButtonClicked();
    void AddBoxButtonClicked();
    void Okay();
    void Cancel();
    void OnSegmentConflictStateChanged(bool conflictExists);
    void UpdateGUIState();

    bool mSegmentConflictExists {false};

    Ui::CalGraphDock* ui;

    RangeCalibrationGraphicsView* mGraphicsView {nullptr};
};
