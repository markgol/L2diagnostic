//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RangeCalibrationGraphicsView.h
//
//
//  Purpose:
//  Stage 3 user gui
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
//  This is the interactive stage 3 calibration user gui
//--------------------------------------------------------
#pragma once
#include <QGraphicsView>
#include <qevent.h>
#include <unordered_set>
#include <vector>
#include "Stage3CalRangeDialog.h"
#include "CalibrationGUIitems.h"

//========================================================
// enumerated class definitions
//========================================================

//--------------------------------------------------------
// enumerated class EditMode
//--------------------------------------------------------
enum class EditMode
{
    Select,
    AddSegment,
    AddExclusion
};

//--------------------------------------------------------
// enumerated class SegmentGeometryConflict
//--------------------------------------------------------
enum class SegmentGeometryConflict
{
    None,
    Intersection,
    CollinearOverlap
};

//========================================================
//  class RangeCalibrationGraphicsView definition
//========================================================
class RangeCalibrationGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    struct SegmentConflict
    {
        uint32_t firstSegmentId {0};
        uint32_t secondSegmentId {0};

        uint32_t firstElevationIndex {0};
        uint32_t lastElevationIndex {0};

        bool geometricIntersection {false};
        bool rayMappingOverlap {false};
    };

    explicit RangeCalibrationGraphicsView(QWidget* parent = nullptr);

    void SetPoints(const std::vector<Stage3BPoint>& points);
    void ClearGUI_SetPoints(const std::vector<Stage3BPoint>& points);
    void SetCalibrationSegments(const std::vector<CalibrationSegment>& segments);
    void SetExclusionRegions(const std::vector<ExclusionRegion>& regions);

    const std::vector<CalibrationSegment>& GetCalibrationSegments() const noexcept
    {
        return mSegments;
    }

    const std::vector<ExclusionRegion>& GetExclusionRegions() const noexcept
    {
        return mExclusionRegions;
    }

    void SetEditMode(EditMode mode);

    bool HasSegmentConflicts() const noexcept
    {
        return !mConflictingSegmentIds.empty();
    }

signals:
    void EditModeChanged(EditMode mode);
    void SegmentConflictStateChanged(bool conflictExists);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:

    void RebuildScene();
    void DrawPoints();
    void DrawCalibrationSegments();
    void DrawExclusionRegions();
    void FitToData();
    void DeleteSelectedItems();
    void CancelSegmentCreation();
    void CancelExclusionCreation();

    // segment conflict termination
    void UpdateTemporarySegmentAppearance();

    void EvaluateSegmentConflicts();

    // other member variables
    EditMode mEditMode { EditMode::Select};

    uint32_t mNextSegmentId {1};
    uint32_t mNextExclusionRegionId {1};

    QGraphicsScene* mScene {nullptr};
    std::vector<Stage3BPoint> mPoints;
    std::vector<CalibrationSegment> mSegments;
    std::vector<ExclusionRegion> mExclusionRegions;

    bool mCreatingExclusion {false};
    QPointF mExclusionStart;
    ExclusionRegionItem* mTemporaryExclusionItem {nullptr};

    // temporary line while drawing
    bool mCreatingSegment {false};
    QPointF mSegmentStart;

    // temporary conflict resolution members
    std::unordered_set<uint32_t> mConflictingSegmentIds;
    QGraphicsLineItem* mTemporarySegmentItem {nullptr};
    bool mTemporarySegmentConflict {false};
};
