//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RaySegmentIntersection.h
//
//
//  Purpose:
//  display information for a calibration file
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

#include <cmath>
#include <cstdint>
#include <numbers>
#include <QPointF>

//--------------------------------------------------------
// struct CalibrationSegment
//--------------------------------------------------------
struct CalibrationSegment
{
    uint32_t id {0};

    QPointF p1;
    QPointF p2;

    bool enabled {true};
};

//--------------------------------------------------------
// non-class function: Cross2D()
//--------------------------------------------------------
inline double Cross2D(
    double ax,
    double ay,
    double bx,
    double by)
{
    return ax * by - ay * bx;
}

//--------------------------------------------------------
// RaySegmentIntersection
//--------------------------------------------------------
inline bool RaySegmentIntersection(double elevation,
                                   const CalibrationSegment& segment,
                                   double& distance)
{
    constexpr double kDegreesToRadians = std::numbers::pi_v<double> / 180.0;

    constexpr double kEpsilon = 1.0e-12;

    const double angle = elevation * kDegreesToRadians;

    //
    // Ray:
    // origin = (0, 0)
    // direction = d
    //
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);

    //
    // Segment:
    // p1 + u * s
    //
    const double x1 = segment.p1.x();
    const double y1 = segment.p1.y();
    const double sx = segment.p2.x() - segment.p1.x();
    const double sy = segment.p2.y() - segment.p1.y();

    const double denominator = Cross2D(dx, dy, sx, sy);

    //
    // Parallel or nearly parallel.
    //
    if (std::abs(denominator) < kEpsilon) {
        return false;
    }

    const double t = Cross2D(x1, y1, sx, sy) / denominator;
    const double u = Cross2D(x1, y1, dx, dy) / denominator;

    //
    // Intersection must lie forward on the ray
    // and inside the finite calibration segment.
    //
    if (t < 0.0) {
        return false;
    }

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    distance = t;

    return true;
}
