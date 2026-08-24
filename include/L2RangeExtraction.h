//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2RangeExtraction.h
//
//
//  Purpose:
//  Stage 3 data anaylsis for stage 3B data extraction
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
//  This is used in the analysis portion of the
//  L2 range correction calibration stage 3
//--------------------------------------------------------
#pragma once

#include <cstdint>
#include <QRect>
#include <qpoint>
#include "RangeAnalysisResult.h"
#include "RaySegmentIntersection.h"

//--------------------------------------------------------
// enumerated classes
//--------------------------------------------------------
enum class PointQuality
{
    Good,
    Suspect,
    Reject
};

enum PointQualityFlag : uint32_t
{
    QualityIssueNone        = 0x0000,

    LowSampleCount          = 0x0001,
    RangeSpread             = 0x0002,
    HighPeakToPeakRange     = 0x0004,
    InvalidRange            = 0x0008,

    // These next 2 are more likely to flag missing
    // scan angle data (ie. there is no valid range return)
    // then an actual deviation in stepsize.
    PreviousStepDeviation   = 0x0010,
    NextStepDeviation       = 0x0020
};

enum class SegmentAssignment
{
    Unassigned,
    Assigned,
    Ambiguous
};

//--------------------------------------------------------
// Stage 3B structs from or for analysis
//--------------------------------------------------------

//--------------------------------------------------------
// struct Stage3BPoint
//  units are meters
//--------------------------------------------------------
struct Stage3BPoint
{
    uint16_t elevationIndex {0};

    double elevation {0.0};          // degrees
    double measuredRange_m {0.0};      // meters

    double x_m {0.0};
    double y_m {0.0};


    uint32_t sampleCount {0};
    double rangeStdDeviation_m {0.0};
    double peakToPeakRange_m {0.0};

    PointQuality quality {PointQuality::Good};
    uint32_t qualityFlags { QualityIssueNone };
    double previousElevationStep {0.0};
    double nextElevationStep {0.0};

    bool excluded {false};

    SegmentAssignment assignment {
        SegmentAssignment::Unassigned
    };

    std::optional<uint32_t> assignedSegmentId {
        std::nullopt
    };

    double referenceRange_m  {0.0};
    double rangeCorrection_m {0.0};
};


//--------------------------------------------------------
// struct RangeCalibrationMeasurement
//--------------------------------------------------------
struct RangeCalibrationMeasurement
{
    uint16_t elevationIndex {0};
    double elevation {0.0};

    double measuredRange_m {0.0};
    double trueRange_m {0.0};
    double correction_m {0.0};

    double rangeStdDeviation_m {0.0};
    double peakToPeakRange_m {0.0};
};

//--------------------------------------------------------
// struct ExclusionRegion
//--------------------------------------------------------
struct ExclusionRegion
{
    uint32_t id {0};
    QRectF rect;
    bool enabled {true};
};

//========================================================
// non class functions
//========================================================

//--------------------------------------------------------
// non class function: ToString(PointQuality quality)
//--------------------------------------------------------
static const char* ToString(PointQuality quality)
{
    switch (quality)
    {
    case PointQuality::Good:
        return "Good";

    case PointQuality::Suspect:
        return "Suspect";

    case PointQuality::Reject:
        return "Reject";
    }

    return "Unknown";
}

//--------------------------------------------------------
// non class function: ToString(SegmentAssignment assignment)
//--------------------------------------------------------
static const char* ToString(SegmentAssignment assignment)
{
    switch (assignment)
    {
    case SegmentAssignment::Unassigned:
        return "Unassigned";

    case SegmentAssignment::Assigned:
        return "Assigned";

    case SegmentAssignment::Ambiguous:
        return "Ambiguous";
    }
    return "Unknown";
}

//--------------------------------------------------------
// class L2RangeExtraction definition
//--------------------------------------------------------
class L2RangeExtraction
{
public:
    // void SetInput(
    //     const RangeAnalysisResult& analysis);
    void SetInput(const RangeAnalysisResult& analysis)
    {
        mInputAnalysis = analysis;
    }

    const std::vector<Stage3BPoint>& GetStage3BPoints() const noexcept
    {
        return mPoints;
    }

    void SetMinRange_m(double p) {mMinRange_m = p;} // units are in meters
    void SetMaxRange_m(double p) {mMaxRange_m = p;} // units are in meters

    void SetCalibrationSegments(
        const std::vector<CalibrationSegment>& segments) {
        mSegments = segments;
    }

    void SetExclusionRegions(
        const std::vector<ExclusionRegion>& regions) {
        mExclusionRegions = regions;
    }

    bool Evaluate();

    const std::vector<Stage3BPoint>& GetPoints() const noexcept{
        return mPoints;
    };

    bool ExportMeasuredPointsCSV(const std::string& filename) const;
    bool EvaluateCalibrationSegments();
    void EvaluateExclusions();
    bool BuildCalibrationMeasurements(
        std::vector<RangeCalibrationMeasurement>& measurements) const;
    bool ExportCalMeasurementsCSV(const std::string& filename,
                                  const std::vector<RangeCalibrationMeasurement> measurements) const;

    const std::vector<CalibrationSegment>& GetCalibrationSegments() const noexcept
    {
        return mSegments;
    }

    const std::vector<ExclusionRegion>& GetExclusionRegions() const noexcept
    {
        return mExclusionRegions;
    }

private:
    void BuildMeasuredPoints();
    void EvaluatePointQuality();

    bool PointInsideExclusion(
        const Stage3BPoint& point) const;

    RangeAnalysisResult mInputAnalysis;

    std::vector<Stage3BPoint> mPoints;
    std::vector<CalibrationSegment> mSegments;
    std::vector<ExclusionRegion> mExclusionRegions;
    std::vector<RangeCalibrationMeasurement> mCalMeasurements;
    double mNominalElevationStep {0.6};
    double mSuspectStepDeviation {0.02};
    double mRejectStepDeviation {0.05};
    uint32_t mMinimumSampleCount {64};
    double mMinRange_m {0.15}; //meters
    double mMaxRange_m {40.0}; //meters
};
