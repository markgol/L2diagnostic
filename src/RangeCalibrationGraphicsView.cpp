//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RangeCalibrationGraphicsView.cpp
//
//
//  Purpose:
//  Stage 3 user gui
//
//  V2.0.0 RC1 2026-08-07
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
#include <algorithm>
#include <cmath>
#include <numbers>
#include "RaySegmentIntersection.h"
#include "RangeCalibrationGraphicsView.h"
#include "CalibrationGUIitems.h"

//--------------------------------------------------------
// class constructor
//--------------------------------------------------------
RangeCalibrationGraphicsView::
    RangeCalibrationGraphicsView(
        QWidget* parent)
    : QGraphicsView(parent)
{
    mScene = new QGraphicsScene(this);

    setScene(mScene);

    const QColor backgroundColor(160, 160, 160);
    mScene->setBackgroundBrush(QBrush(backgroundColor));
    setBackgroundBrush(QBrush(backgroundColor));
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    setRenderHint(QPainter::Antialiasing, true);

    setDragMode(QGraphicsView::ScrollHandDrag);

    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    setFocusPolicy(Qt::StrongFocus);

    setResizeAnchor(QGraphicsView::AnchorViewCenter);
}

//--------------------------------------------------------
// wheelEvent
//--------------------------------------------------------
void RangeCalibrationGraphicsView::wheelEvent(
    QWheelEvent* event)
{
    constexpr double ZOOM_FACTOR = 1.15;

    if (event->angleDelta().y() > 0)
    {
        scale(
            ZOOM_FACTOR,
            ZOOM_FACTOR);
    }
    else
    {
        scale(
            1.0 / ZOOM_FACTOR,
            1.0 / ZOOM_FACTOR);
    }
}

//--------------------------------------------------------
// SetPoints
//--------------------------------------------------------
void RangeCalibrationGraphicsView::SetPoints(
    const std::vector<Stage3BPoint>& points)
{
    mPoints = points;
    RebuildScene();
    EvaluateSegmentConflicts();

    FitToData();
}

//--------------------------------------------------------
// ClearGUI_SetPoints
//--------------------------------------------------------
void RangeCalibrationGraphicsView::ClearGUI_SetPoints(
    const std::vector<Stage3BPoint>& points)
{
    mPoints = points;

    //
    // Clear all user-defined GUI geometry.
    //
    mSegments.clear();
    mExclusionRegions.clear();

    //
    // Reset generated IDs.
    //
    mNextSegmentId = 1;
    mNextExclusionRegionId = 1;

    //
    // Reset any partially completed operation.
    //
    mCreatingSegment = false;

    //
    // A new calibration starts by defining
    // calibration exclsuion regions.
    //
    SetEditMode(EditMode::AddExclusion);

    RebuildScene();
    EvaluateSegmentConflicts();
    FitToData();
}

//--------------------------------------------------------
// SetCalibrationSegments
//--------------------------------------------------------
void RangeCalibrationGraphicsView::SetCalibrationSegments(
    const std::vector<CalibrationSegment>& segments)
{    
    mSegments = segments;

    mNextSegmentId = 1;

    for (const CalibrationSegment& segment : mSegments)
    {
        if (segment.id >= mNextSegmentId) {
            mNextSegmentId = segment.id + 1;
        }
    }

    RebuildScene();
    EvaluateSegmentConflicts();
    FitToData();
}

//--------------------------------------------------------
// SetExclusionRegions
//--------------------------------------------------------
void RangeCalibrationGraphicsView::SetExclusionRegions(
    const std::vector<ExclusionRegion>& regions)
{
    mExclusionRegions = regions;
    mNextExclusionRegionId = 1;

    for (const ExclusionRegion& region : mExclusionRegions)
    {
        if (region.id >= mNextExclusionRegionId){
            mNextExclusionRegionId = region.id + 1;
        }
    }

    RebuildScene();
    EvaluateSegmentConflicts();

    FitToData();
}

//--------------------------------------------------------
// RebuildScene
//--------------------------------------------------------
void RangeCalibrationGraphicsView::RebuildScene()
{
    mScene->clear();

    DrawPoints();
    DrawCalibrationSegments();
    DrawExclusionRegions();

    const QRectF bounds =
        mScene->itemsBoundingRect();

    if (!bounds.isEmpty()) {
        mScene->setSceneRect(bounds);
    }
}

//--------------------------------------------------------
// DrawPoints
//--------------------------------------------------------
void RangeCalibrationGraphicsView::DrawPoints()
{
    constexpr double POINT_SIZE = 0.015;

    for (const Stage3BPoint& point : mPoints)
    {
        QColor color;

        switch (point.quality)
        {
        case PointQuality::Good:
            color = Qt::green;
            break;

        case PointQuality::Suspect:
            color = QColor(255, 230, 80);
            break;

        case PointQuality::Reject:
            color = Qt::red;
            break;
        }

        const QRectF rect(
            point.x_m - POINT_SIZE / 2.0,
            point.y_m - POINT_SIZE / 2.0,
            POINT_SIZE,
            POINT_SIZE);

        mScene->addEllipse(
            rect,
            Qt::NoPen,
            QBrush(color));
    }
}

//--------------------------------------------------------
// DrawPoints
//--------------------------------------------------------
void RangeCalibrationGraphicsView::DrawCalibrationSegments()
{
    for (const CalibrationSegment& segment : mSegments)
    {
        if (!segment.enabled) {
            continue;
        }

        auto* item = new CalibrationSegmentItem(
                segment.id,
                segment.p1,
                segment.p2);

        item->SetGeometryChangedCallback([this]()
            {
                EvaluateSegmentConflicts();
            });

        mScene->addItem(item);

    }
}

//--------------------------------------------------------
// DrawPoints
//--------------------------------------------------------
void RangeCalibrationGraphicsView::DrawExclusionRegions()
{
    for (const ExclusionRegion& region :
         mExclusionRegions)
    {
        if (!region.enabled) {
            continue;
        }

        auto* item = new ExclusionRegionItem(
                region.id,
                region.rect);

        mScene->addItem(item);
    }
}

//--------------------------------------------------------
// FitToData
//--------------------------------------------------------
void RangeCalibrationGraphicsView::FitToData()
{
    const QRectF bounds =
        mScene->itemsBoundingRect();

    if (bounds.isEmpty())
    {
        return;
    }

    resetTransform();

    fitInView(
        bounds,
        Qt::KeepAspectRatio);

    scale(1.0, -1.0);
}

//--------------------------------------------------------
// SetEditMode
//--------------------------------------------------------
void RangeCalibrationGraphicsView::SetEditMode(
    EditMode mode)
{
    mEditMode = mode;

    switch (mEditMode) {
    case EditMode::Select:
        setDragMode(QGraphicsView::NoDrag);
        viewport()->setCursor(Qt::OpenHandCursor);
        break;

    case EditMode::AddSegment:
        setDragMode(QGraphicsView::NoDrag);
        viewport()->setCursor(Qt::CrossCursor);
        break;

    case EditMode::AddExclusion:
        setDragMode(QGraphicsView::NoDrag);
        viewport()->setCursor(Qt::CrossCursor);
        break;
    }
    emit EditModeChanged(mEditMode);
}

//--------------------------------------------------------
// mousePressEvent
//--------------------------------------------------------
void RangeCalibrationGraphicsView::mousePressEvent(
    QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        CancelSegmentCreation();
        CancelExclusionCreation();
        EvaluateSegmentConflicts();

        if (mScene) {
            mScene->clearSelection();
        }

        SetEditMode(EditMode::Select);
        event->accept();
        return;
    }

    // add select mode
    if (mEditMode == EditMode::Select && event->button() == Qt::LeftButton) {
        const QPointF scenePosition = mapToScene(event->position().toPoint());

        QGraphicsItem* item = mScene->itemAt(scenePosition, transform());

        if (!item) {
            mScene->clearSelection();
            event->accept();
            return;
        }
    }

    // add exclusion mode
    if (mEditMode == EditMode::AddExclusion) {
        if (event->button() != Qt::LeftButton){
            event->ignore();
            return;
        }

        mExclusionStart = mapToScene(event->position().toPoint());
        mCreatingExclusion = true;

        // temporary ID 0, not part of model yet
        mTemporaryExclusionItem =
            new ExclusionRegionItem(0, QRectF(mExclusionStart,mExclusionStart));
        mScene->addItem(mTemporaryExclusionItem);

        event->accept();
        return;
    }

    if (mEditMode != EditMode::AddSegment) {
        QGraphicsView::mousePressEvent(event);
        return;
    }

    if (event->button() != Qt::LeftButton) {
        QGraphicsView::mousePressEvent(event);
        return;
    }

    const QPointF scenePosition = mapToScene(event->position().toPoint());

    // on 2nd click remove temporary line
    if (mTemporarySegmentItem) {
        mScene->removeItem(mTemporarySegmentItem);
        delete mTemporarySegmentItem;

        mTemporarySegmentItem = nullptr;
    }

    // on first click create temporary line
    if (!mCreatingSegment) {
        mSegmentStart =scenePosition;
        mCreatingSegment = true;

        QPen pen(Qt::yellow);
        pen.setCosmetic(true);
        pen.setWidth(2);
        pen.setStyle(Qt::DashLine);

        mTemporarySegmentItem =
            mScene->addLine(
                QLineF(mSegmentStart, mSegmentStart), pen);
        mTemporarySegmentItem->setZValue(20.0);

        event->accept();
        return;
    }

    CalibrationSegment segment;

    segment.id = mNextSegmentId++;
    segment.p1 = mSegmentStart;
    segment.p2 = scenePosition;

    segment.enabled = true;

    mSegments.emplace_back(segment);
    mCreatingSegment = false;

    RebuildScene();
    EvaluateSegmentConflicts();

    event->accept();
}

//--------------------------------------------------------
// CancelSegmentCreation
//--------------------------------------------------------
void RangeCalibrationGraphicsView::CancelSegmentCreation()
{
    mCreatingSegment = false;

    if (mTemporarySegmentItem) {
        mScene->removeItem(mTemporarySegmentItem);
        delete mTemporarySegmentItem;
        mTemporarySegmentItem = nullptr;
    }
}

//--------------------------------------------------------
// CancelExclsuionCreation
//--------------------------------------------------------
void RangeCalibrationGraphicsView::CancelExclusionCreation()
{
    mCreatingExclusion = false;

    if (mTemporaryExclusionItem) {
        mScene->removeItem(mTemporaryExclusionItem);
        delete mTemporaryExclusionItem;
        mTemporaryExclusionItem = nullptr;
    }
}

//--------------------------------------------------------
// mouseMoveEvent
//--------------------------------------------------------
void RangeCalibrationGraphicsView::mouseMoveEvent(
    QMouseEvent* event)
{
    if (mEditMode == EditMode::AddSegment &&
                        mCreatingSegment &&
                        mTemporarySegmentItem) {

        const QPointF currentPosition = mapToScene(event->position().toPoint());
        mTemporarySegmentItem->setLine(QLineF(mSegmentStart, currentPosition));

        EvaluateSegmentConflicts();
        event->accept();
        return;
    }

    if (mEditMode == EditMode::AddExclusion &&
                        mCreatingExclusion &&
                        mTemporaryExclusionItem) {

        const QPointF currentPosition = mapToScene(event->position().toPoint());

        const QRectF rect(mExclusionStart, currentPosition);

        mTemporaryExclusionItem->setRect(rect.normalized());

        event->accept();
        return;
    }

    QGraphicsView::mouseMoveEvent(event);
}

//--------------------------------------------------------
// mouseReleaseEvent
//--------------------------------------------------------
void RangeCalibrationGraphicsView::mouseReleaseEvent(
    QMouseEvent* event)
{
    if (mEditMode == EditMode::AddExclusion &&
                mCreatingExclusion &&
                mTemporaryExclusionItem &&
            event->button() == Qt::LeftButton) {

        const QRectF rect = mTemporaryExclusionItem->GetSceneRect().normalized();
        //
        // Remove temporary graphics item.
        //
        CancelExclusionCreation();

        //
        // Avoid creating an effectively zero-sized
        // exclusion region from an accidental click.
        //

        if (rect.width() > 0.0 && rect.height() > 0.0) {
            ExclusionRegion region;

            region.id = mNextExclusionRegionId++;
            region.rect = rect;
            region.enabled = true;

            mExclusionRegions.emplace_back(region);

            RebuildScene();
            EvaluateSegmentConflicts();
        }

        event->accept();
        return;
    }

    QGraphicsView::mouseReleaseEvent(event);
}

//--------------------------------------------------------
// keyPressEvent
//--------------------------------------------------------
void RangeCalibrationGraphicsView::keyPressEvent(
    QKeyEvent* event)
{
    if (!event) {
        return;
    }

    //----------------------------------------------------
    // Escape
    //----------------------------------------------------
    if (event->key() == Qt::Key_Escape) {
        CancelSegmentCreation();
        CancelExclusionCreation();
        EvaluateSegmentConflicts();

        if (mScene) {
            mScene->clearSelection();
        }
        SetEditMode(EditMode::Select);
        event->accept();
        return;
    }

    //----------------------------------------------------
    // Delete selected geometry
    //----------------------------------------------------
    if (event->key() == Qt::Key_Delete) {
        DeleteSelectedItems();
        event->accept();
        return;
    }

    //----------------------------------------------------
    // Ctrl+A: select all editable top-level items
    //----------------------------------------------------
    if (event->matches(QKeySequence::SelectAll)) {
        if (mScene) {
            mScene->clearSelection();

            for (QGraphicsItem* item : mScene->items())
            {
                if (dynamic_cast<CalibrationSegmentItem*>(item) ||
                    dynamic_cast<ExclusionRegionItem*>(item)) {

                    item->setSelected(true);
                }
            }
        }
        event->accept();
        return;
    }

    //----------------------------------------------------
    // Ctrl+D: clear selection
    //----------------------------------------------------
    if ((event->modifiers() &
         Qt::ControlModifier) &&
        event->key() == Qt::Key_D)
    {
        if (mScene)
        {
            mScene->clearSelection();
        }

        event->accept();
        return;
    }

    //----------------------------------------------------
    // Ignore single-key mode shortcuts when Ctrl, Alt,
    // or Meta is being held.
    //----------------------------------------------------
    if (event->modifiers() &
        (Qt::ControlModifier |
         Qt::AltModifier |
         Qt::MetaModifier))
    {
        QGraphicsView::keyPressEvent(event);
        return;
    }

    //----------------------------------------------------
    // S: Select
    //----------------------------------------------------
    if (event->key() == Qt::Key_S)
    {
        CancelSegmentCreation();
        CancelExclusionCreation();
        EvaluateSegmentConflicts();

        SetEditMode(
            EditMode::Select);

        event->accept();
        return;
    }

    //----------------------------------------------------
    // L: Add Segment (line)
    //----------------------------------------------------
    if (event->key() == Qt::Key_L)
    {
        CancelSegmentCreation();
        CancelExclusionCreation();
        EvaluateSegmentConflicts();

        SetEditMode(
            EditMode::AddSegment);

        event->accept();
        return;
    }

    //----------------------------------------------------
    // B,E: Add Exclusion (box)
    //----------------------------------------------------
    if (event->key() == Qt::Key_B || event->key() == Qt::Key_E)
    {
        CancelSegmentCreation();
        CancelExclusionCreation();
        EvaluateSegmentConflicts();

        SetEditMode(
            EditMode::AddExclusion);

        event->accept();
        return;
    }

    QGraphicsView::keyPressEvent(event);
}
//--------------------------------------------------------
// DeleteSelectedItems
//--------------------------------------------------------
void RangeCalibrationGraphicsView::DeleteSelectedItems()
{
    if (!mScene) {
        return;
    }

    const QList<QGraphicsItem*> selectedItems = mScene->selectedItems();
    if (selectedItems.empty()) {
        return;
    }

    std::vector<uint32_t> segmentIds;
    std::vector<uint32_t> regionIds;

    for (QGraphicsItem* item : selectedItems)
    {
        if (auto* segment = dynamic_cast<CalibrationSegmentItem*>(item)) {
            segmentIds.push_back(segment->GetSegmentId());
            continue;
        }

        if (auto* region = dynamic_cast<ExclusionRegionItem*>(item)) {
            regionIds.push_back(region->GetRegionId());
        }
    }

    for (const uint32_t id : segmentIds)
    {
        std::erase_if(mSegments,
            [id](const CalibrationSegment& segment)
            {
                return segment.id == id;
            });
    }

    for (const uint32_t id : regionIds)
    {
        std::erase_if(mExclusionRegions,
            [id](const ExclusionRegion& region)
            {
                return region.id == id;
            });
    }

    RebuildScene();
    EvaluateSegmentConflicts();
}

//--------------------------------------------------------
// UpdateTemporarySegmentAppearance
//--------------------------------------------------------
void RangeCalibrationGraphicsView::
    UpdateTemporarySegmentAppearance()
{
    if (!mTemporarySegmentItem) {
        return;
    }

    QPen pen(mTemporarySegmentConflict
            ? Qt::magenta
            : Qt::yellow);

    pen.setCosmetic(true);
    pen.setStyle(Qt::DashLine);

    pen.setWidth(mTemporarySegmentConflict
            ? 3
            : 2);

    mTemporarySegmentItem->setPen(
        pen);
}

//--------------------------------------------------------
//  EvaluateSegmentConflicts
//--------------------------------------------------------
void RangeCalibrationGraphicsView::EvaluateSegmentConflicts()
{
    mConflictingSegmentIds.clear();
    mTemporarySegmentConflict = false;

    if (!mScene) {
        emit SegmentConflictStateChanged(false);
        return;
    }

    struct DisplaySegment
    {
        uint32_t id {0};
        CalibrationSegment geometry;
        CalibrationSegmentItem* item {nullptr};
        bool temporary {false};
    };

    std::vector<DisplaySegment> segments;

    //----------------------------------------------------
    // Gather committed segments from their current scene
    // geometry. This ensures endpoint movement is seen
    // immediately, before mSegments is synchronized.
    //----------------------------------------------------
    for (QGraphicsItem* graphicsItem : mScene->items())
    {
        auto* segmentItem = dynamic_cast<CalibrationSegmentItem*>(graphicsItem);

        if (!segmentItem) {
            continue;
        }

        CalibrationSegment segment;

        segment.id = segmentItem->GetSegmentId();
        segment.p1 = segmentItem->GetP1();
        segment.p2 = segmentItem->GetP2();
        segment.enabled = true;
        segments.push_back(
            {
                segment.id,
                segment,
                segmentItem,
                false
            });
    }

    //----------------------------------------------------
    // Include the temporary segment currently being
    // drawn, if one exists.
    //----------------------------------------------------
    if (mCreatingSegment && mTemporarySegmentItem) {
        const QLineF line = mTemporarySegmentItem->line();

        CalibrationSegment segment;

        segment.id = 0;
        segment.p1 = line.p1();
        segment.p2 = line.p2();
        segment.enabled = true;

        segments.push_back(
            {
                0,
                segment,
                nullptr,
                true
            });
    }

    //----------------------------------------------------
    // For each actual Stage 3B elevation ray, determine
    // which displayed segments that same ray intersects.
    //----------------------------------------------------
    for (const Stage3BPoint& point : mPoints)
    {
        std::vector<size_t> intersectingSegments;

        for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
        {
            double distance = 0.0;

            if (RaySegmentIntersection(point.elevation, segments[segmentIndex].geometry, distance)) {
                intersectingSegments.push_back(segmentIndex);
            }
        }

        if (intersectingSegments.size() < 2) {
            continue;
        }

        //------------------------------------------------
        // Every segment intersected by this same
        // elevation ray participates in a conflict.
        //------------------------------------------------
        for (const size_t segmentIndex : intersectingSegments)
        {
            DisplaySegment& segment = segments[segmentIndex];

            if (segment.temporary) {
                mTemporarySegmentConflict = true;
            } else {
                mConflictingSegmentIds.insert(segment.id);
            }
        }
    }

    //----------------------------------------------------
    // Update committed segment appearance.
    //----------------------------------------------------
    for (DisplaySegment& segment : segments)
    {
        if (!segment.item) {
            continue;
        }

        segment.item->SetConflict(mConflictingSegmentIds.contains(segment.id));
    }

    //----------------------------------------------------
    // Update the temporary preview line appearance.
    //----------------------------------------------------
    UpdateTemporarySegmentAppearance();

    const bool conflictExists = !mConflictingSegmentIds.empty() || mTemporarySegmentConflict;

    emit SegmentConflictStateChanged(conflictExists);
}
