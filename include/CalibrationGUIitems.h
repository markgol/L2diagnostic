//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalibrationSegment.h
//
//
//  Purpose:
//  Stage 3 user gui
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
//  This is the interactive stage 3 calibration user gui classes and items
//--------------------------------------------------------
#pragma once

#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <functional>
#include <utility>

//--------------------------------------------------------
//  enumerated class ExclusionHandlePosition definition
//--------------------------------------------------------
enum class ExclusionHandlePosition
{
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
};

class CalibrationSegmentItem;
class ExclusionRegionItem;

//--------------------------------------------------------
//  class SegmentEndpointItem definition
//--------------------------------------------------------
class SegmentEndpointItem :
                            public QGraphicsEllipseItem
{
public:
    SegmentEndpointItem(
        CalibrationSegmentItem* owner,
        bool firstEndpoint);    

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

private:
    CalibrationSegmentItem* mOwner {nullptr};
    bool mFirstEndpoint {false};
};

//--------------------------------------------------------
//  class CalibrationSegmentItem definition
//--------------------------------------------------------
class CalibrationSegmentItem :
        public QGraphicsLineItem
{
public:
    using GeometryChangedCallback = std::function<void()>;

    void SetGeometryChangedCallback(GeometryChangedCallback callback)
    {
        mGeometryChangedCallback = std::move(callback);
    }

    CalibrationSegmentItem(uint32_t segmentId, const QPointF& p1, const QPointF& p2);

    uint32_t GetSegmentId() const noexcept { return mSegmentId;}
    QPointF GetP1() const { return mP1Handle->pos(); }
    QPointF GetP2() const { return mP2Handle->pos(); }
    void EndpointMoved(bool firstEndpoint, const QPointF& position);

    void SetConflict(bool conflict);

    bool HasConflict() const noexcept { return mConflict;}

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    QPainterPath shape() const override;

private:
    void UpdateLine();
    void UpdateAppearance();
    bool mConflict {false};

    uint32_t mSegmentId {0};

    SegmentEndpointItem* mP1Handle {nullptr};
    SegmentEndpointItem* mP2Handle {nullptr};

    GeometryChangedCallback mGeometryChangedCallback;
};

//--------------------------------------------------------
//  class ExclusionResizeHandle definition
//--------------------------------------------------------
class ExclusionResizeHandle :
                              public QGraphicsEllipseItem
{
public:
    ExclusionResizeHandle(
        ExclusionRegionItem* owner,
        ExclusionHandlePosition position);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

private:
    ExclusionRegionItem* mOwner {nullptr};

    ExclusionHandlePosition mPosition {
        ExclusionHandlePosition::TopLeft
    };
};

//--------------------------------------------------------
//  class ExclusionRegionItem definition
//--------------------------------------------------------
class ExclusionRegionItem :
                            public QGraphicsRectItem
{
public:
    ExclusionRegionItem(
        uint32_t regionId,
        const QRectF& rect);

    uint32_t GetRegionId() const noexcept;

    QRectF GetSceneRect() const;

    void ResizeHandleMoved(
        ExclusionHandlePosition handle,
        const QPointF& position);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
    void UpdateHandles();

    uint32_t mRegionId {0};

    ExclusionResizeHandle* mTopLeft {nullptr};
    ExclusionResizeHandle* mTopRight {nullptr};
    ExclusionResizeHandle* mBottomLeft {nullptr};
    ExclusionResizeHandle* mBottomRight {nullptr};

    bool mUpdatingHandles {false};
};
