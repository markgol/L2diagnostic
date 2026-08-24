//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: CalibrationSegmentItem.cpp
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
#include "CalibrationGUIitems.h"
#include <QBrush>
#include <QCursor>
#include <QGraphicsSceneMouseEvent>
#include <QPen>
#include <qgraphicsscene.h>
#include <QPainterPath>
#include <QPainterPathStroker>

//--------------------------------------------------------
//  class constructor SegmentEndpointItem
//--------------------------------------------------------
SegmentEndpointItem::SegmentEndpointItem(
    CalibrationSegmentItem* owner,
    bool firstEndpoint)
    : QGraphicsEllipseItem(owner)
    , mOwner(owner)
    , mFirstEndpoint(firstEndpoint)
{
    constexpr double HANDLE_SIZE = 0.04;

    setRect(
        -HANDLE_SIZE / 2.0,
        -HANDLE_SIZE / 2.0,
        HANDLE_SIZE,
        HANDLE_SIZE);

    setBrush(QBrush(Qt::darkYellow));
    QPen pen(Qt::yellow);
    pen.setCosmetic(true);
    pen.setWidth(1);
    setZValue(1.0);
    setPen(pen);

    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setAcceptedMouseButtons(Qt::LeftButton);
    setCursor(Qt::CrossCursor);
}

//--------------------------------------------------------
//  class constructor CalibrationSegmentItem
//--------------------------------------------------------
CalibrationSegmentItem::CalibrationSegmentItem(
    uint32_t segmentId,
    const QPointF& p1,
    const QPointF& p2)
    : mSegmentId(segmentId)
{
    setZValue(20.0);

    mP1Handle = new SegmentEndpointItem(this, true);
    mP2Handle = new SegmentEndpointItem(this, false);

    mP1Handle->setPos(p1);
    mP2Handle->setPos(p2);

    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setHandlesChildEvents(false);
    UpdateLine();
    UpdateAppearance();
}

//--------------------------------------------------------
//  class constructor ExclusionRegionItem
//--------------------------------------------------------
ExclusionRegionItem::ExclusionRegionItem(
    uint32_t regionId,
    const QRectF& rect)
    : QGraphicsRectItem(rect.normalized())
    , mRegionId(regionId)
{
    QPen pen(Qt::red);

    pen.setCosmetic(true);
    pen.setWidth(2);
    setZValue(10.0);
    setPen(pen);
    setBrush(QBrush(QColor(255, 0, 0, 40)));

    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemIsSelectable, true);

    mTopLeft = new ExclusionResizeHandle(this, ExclusionHandlePosition::TopLeft);
    mTopRight = new ExclusionResizeHandle(this, ExclusionHandlePosition::TopRight);

    mBottomLeft = new ExclusionResizeHandle(this, ExclusionHandlePosition::BottomLeft);
    mBottomRight = new ExclusionResizeHandle(this, ExclusionHandlePosition::BottomRight);

    setAcceptedMouseButtons(Qt::LeftButton);
    setHandlesChildEvents(false);

    UpdateHandles();
}

//--------------------------------------------------------
//  class constructor ExclusionResizeHandle
//--------------------------------------------------------
ExclusionResizeHandle::ExclusionResizeHandle(
    ExclusionRegionItem* owner,
    ExclusionHandlePosition position)
    : QGraphicsEllipseItem(owner)
    , mOwner(owner)
    , mPosition(position)
{
    constexpr double HANDLE_SIZE = 0.06;

    setRect(
        -HANDLE_SIZE / 2.0,
        -HANDLE_SIZE / 2.0,
        HANDLE_SIZE,
        HANDLE_SIZE);

    setBrush(QBrush(Qt::darkRed));

    QPen pen(Qt::red);

    pen.setCosmetic(true);
    pen.setWidth(1);
    setZValue(1.0);
    setPen(pen);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setAcceptedMouseButtons(Qt::LeftButton);
    setCursor(Qt::SizeFDiagCursor);
}

//--------------------------------------------------------
//  ExclusionRegionItem::GetRegionId
//--------------------------------------------------------
uint32_t ExclusionRegionItem::GetRegionId() const noexcept
{
    return mRegionId;
}

//--------------------------------------------------------
//  ExclusionRegionItem::GetSceneRect
//--------------------------------------------------------
QRectF ExclusionRegionItem::GetSceneRect() const
{
    return mapRectToScene(rect());
}

//--------------------------------------------------------
//  CalibrationSegmentItem::UpdateLine
//--------------------------------------------------------
void CalibrationSegmentItem::UpdateLine()
{
    setLine(
        QLineF(
            mP1Handle->pos(),
            mP2Handle->pos()));
}

//--------------------------------------------------------
//  CalibrationSegmentItem::EndpointMoved
//--------------------------------------------------------
void CalibrationSegmentItem::EndpointMoved(bool firstEndpoint, const QPointF& position)
{
    if (firstEndpoint) {
        mP1Handle->setPos(position);
    } else {
        mP2Handle->setPos(position);
    }

    UpdateLine();
    if (mGeometryChangedCallback) {
        mGeometryChangedCallback();
    }
}

//--------------------------------------------------------
//  ExclusionRegionItem::UpdateHandles
//--------------------------------------------------------
void ExclusionRegionItem::UpdateHandles()
{
    mUpdatingHandles = true;

    const QRectF r = rect();

    mTopLeft->setPos(r.topLeft());
    mTopRight->setPos(r.topRight());

    mBottomLeft->setPos(r.bottomLeft());
    mBottomRight->setPos(r.bottomRight());

    mUpdatingHandles = false;
}

//--------------------------------------------------------
//  ExclusionRegionItem::ResizeHandleMoved
//--------------------------------------------------------
void ExclusionRegionItem::ResizeHandleMoved(
    ExclusionHandlePosition handle,
    const QPointF& position)
{
    if (mUpdatingHandles){
        return;
    }

    QRectF r = rect();

    switch (handle)
    {
    case ExclusionHandlePosition::TopLeft:
        r.setTopLeft(position);
        break;

    case ExclusionHandlePosition::TopRight:
        r.setTopRight(position);
        break;

    case ExclusionHandlePosition::BottomLeft:
        r.setBottomLeft(position);
        break;

    case ExclusionHandlePosition::BottomRight:
        r.setBottomRight(position);
        break;
    }

    //
    // Keep the rectangle valid if a handle is dragged
    // through the opposite side.
    //
    r = r.normalized();
    setRect(r);

    UpdateHandles();
}

//--------------------------------------------------------
//  ExclusionResizeHandle::mousePressEvent
//--------------------------------------------------------
void ExclusionResizeHandle::mousePressEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (!mOwner || event->button() != Qt::LeftButton) {
        event->ignore();
        return;
    }

    if (scene()) {
        scene()->clearSelection();
    }

    mOwner->setSelected(true);
    mOwner->setZValue(30.0);
    event->accept();
}

//--------------------------------------------------------
//  ExclusionResizeHandle::mouseMoveEvent
//--------------------------------------------------------
void ExclusionResizeHandle::mouseMoveEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (!mOwner)
    {
        return;
    }

    //
    // Convert the mouse position from scene coordinates
    // into the exclusion rectangle's local coordinates.
    //

    const QPointF position =
        mOwner->mapFromScene(
            event->scenePos());

    mOwner->ResizeHandleMoved(
        mPosition,
        position);

    event->accept();
}

//--------------------------------------------------------
//  ExclusionResizeHandle::mouseReleaseEvent
//--------------------------------------------------------
void ExclusionResizeHandle::mouseReleaseEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (mOwner) {
        mOwner->setZValue(10.0);
    }

    event->accept();
}

//--------------------------------------------------------
//  SegmentEndpointItem::mousePressEvent
//--------------------------------------------------------
void SegmentEndpointItem::mousePressEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (!mOwner || event->button() != Qt::LeftButton) {
        event->ignore();
        return;
    }

    if (scene()) {
        scene()->clearSelection();
    }

    mOwner->setSelected(true);
    mOwner->setZValue(30.0);
    event->accept();
}

//--------------------------------------------------------
//  SegmentEndpointItem::mousePressEvent
//--------------------------------------------------------
void SegmentEndpointItem::mouseMoveEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (!mOwner) {
        return;
    }

    const QPointF position = mOwner->mapFromScene(event->scenePos());
    mOwner->EndpointMoved(mFirstEndpoint,position);

    event->accept();
}

//--------------------------------------------------------
//  SegmentEndpointItem::mouseReleaseEvent
//--------------------------------------------------------
void SegmentEndpointItem::mouseReleaseEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (mOwner) {
        mOwner->setZValue(20.0);
    }

    event->accept();
}

//--------------------------------------------------------
//  CalibrationSegmentItem::mousePressEvent
//--------------------------------------------------------
void CalibrationSegmentItem::mousePressEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (event->button() != Qt::LeftButton)
    {
        event->ignore();
        return;
    }

    if (scene())
    {
        scene()->clearSelection();
    }

    setSelected(true);

    event->accept();
}

//--------------------------------------------------------
//  CalibrationSegmentItem::shape
//--------------------------------------------------------
QPainterPath CalibrationSegmentItem::shape() const
{
    QPainterPath centerLine;

    centerLine.moveTo(
        line().p1());

    centerLine.lineTo(
        line().p2());

    QPainterPathStroker stroker;

    //
    // Scene coordinates are meters. This gives the
    // line a roughly 30 mm selection width.
    //
    stroker.setWidth(0.03);
    stroker.setCapStyle(Qt::RoundCap);
    stroker.setJoinStyle(Qt::RoundJoin);

    return stroker.createStroke(
        centerLine);
}

//--------------------------------------------------------
//  ExclusionRegionItem::mousePressEvent
//--------------------------------------------------------
void ExclusionRegionItem::mousePressEvent(
    QGraphicsSceneMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        event->ignore();
        return;
    }

    if (scene()) {
        scene()->clearSelection();
    }

    setSelected(true);

    //
    // Call the base implementation because it manages
    // ItemIsMovable dragging.
    //
    QGraphicsRectItem::mousePressEvent(event);
}

//--------------------------------------------------------
//  ExclusionRegionItem::mousePressEvent
//--------------------------------------------------------
QVariant CalibrationSegmentItem::itemChange(GraphicsItemChange change,
                                            const QVariant& value)
{
    const QVariant result = QGraphicsLineItem::itemChange(change, value);

    if (change == QGraphicsItem::ItemSelectedHasChanged){
        UpdateAppearance();
    }

    return result;
}

//--------------------------------------------------------
// UpdateAppearance
//--------------------------------------------------------
void CalibrationSegmentItem::UpdateAppearance()
{
    QColor color = Qt::yellow;
    int width = 2;

    if (mConflict) {
        color = Qt::magenta;
        width = 3;
    }

    if (isSelected()) {
        color = Qt::white;
        width = 4;
    }

    QPen pen(color);
    pen.setCosmetic(true);
    pen.setWidth(width);

    setPen(pen);
}

//--------------------------------------------------------
// SetConflict
//--------------------------------------------------------
void CalibrationSegmentItem::SetConflict(bool conflict)
{
    if (mConflict == conflict) {
        return;
    }
    mConflict = conflict;
    UpdateAppearance();
}
