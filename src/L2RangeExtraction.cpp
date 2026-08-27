//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2RangeExtraction.cpp
//
//
//  Purpose:
//  Stage 3 data anaylsis for stage 3B
//      Data extraction
//
//  V2.0.0 RC1 2026-08-06
//  V2.0.1  2026-08-24  This is the intial V2.x release
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
//  This is used in the analysis portion of the
//  L2 range correction calibration stage 3
//--------------------------------------------------------
#include <fstream>
#include <iomanip>
#include "L2RangeExtraction.h"
//#include "RangeCalibrationGraphicsView.h"

//--------------------------------------------------------
//  Evaluate
//--------------------------------------------------------
bool L2RangeExtraction::Evaluate()
{
    if (mInputAnalysis.elevationBins.empty()) {
        return false;
    }

    // compile the measured point, processes all points
    BuildMeasuredPoints();
    if (mPoints.empty()) {
        return false;
    }

    // flag points for quallity issues
    EvaluatePointQuality();

    // mark points for exclusion region
    EvaluateExclusions();

    // mark points in requested calibration line segments
    if (!EvaluateCalibrationSegments()) {
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  BuildMeasuredPoints
//--------------------------------------------------------
void L2RangeExtraction::BuildMeasuredPoints()
{
    mPoints.clear();

    mPoints.reserve(static_cast<size_t>(mInputAnalysis.elevationBins.size()));

    constexpr double PI = 3.1415926535897932384626433832795;
    constexpr double DEG_TO_RAD = (PI / 180.0);

    for (const ElevationBin& bin : mInputAnalysis.elevationBins)
    {
        Stage3BPoint point;

        point.elevationIndex = bin.elevationIndex;
        point.elevation = bin.meanElevation;

        point.measuredRange_m = bin.meanRange_m;

        const double angle = point.elevation * DEG_TO_RAD;

        //
        // 2D fast-scan plane:
        //
        // elevation 0 deg   -> +X
        // elevation 90 deg  -> +Y, x=0
        // elevation 180 deg -> -X
        //

        point.x_m = point.measuredRange_m * std::cos(angle);
        point.y_m = point.measuredRange_m * std::sin(angle);

        point.sampleCount = bin.sampleCount;
        point.rangeStdDeviation_m = bin.standardDeviation_m;
        point.peakToPeakRange_m = bin.peakToPeak_m;

        mPoints.emplace_back(point);
    }
}

//--------------------------------------------------------
//  EvaluateExclusions
//--------------------------------------------------------
void L2RangeExtraction::EvaluateExclusions()
{
    for (Stage3BPoint& point : mPoints)
    {
        point.excluded =
            PointInsideExclusion(point);
    }
}

//--------------------------------------------------------
//  PointInsideExclusion
//--------------------------------------------------------
bool L2RangeExtraction::PointInsideExclusion(
    const Stage3BPoint& point) const
{
    const QPointF measuredPoint(
        point.x_m,
        point.y_m);

    for (const ExclusionRegion& region :
         mExclusionRegions)
    {
        if (!region.enabled){
            continue;
        }

        if (region.rect.contains(measuredPoint)){
            return true;
        }
    }

    return false;
}

//--------------------------------------------------------
//  EvaluateCalibrationSegments
//--------------------------------------------------------
bool L2RangeExtraction::EvaluateCalibrationSegments()
{
    for (Stage3BPoint& point : mPoints)
    {
        point.assignment = SegmentAssignment::Unassigned;
        point.assignedSegmentId.reset();
        point.referenceRange_m = 0.0;
        point.rangeCorrection_m = 0.0;

        uint32_t intersectionCount = 0;

        for (const CalibrationSegment& segment : mSegments)
        {
            if (!segment.enabled) {
                continue;
            }

            double distance_m = 0.0;

            if (!RaySegmentIntersection(point.elevation, segment, distance_m)) {
                continue;
            }

            ++intersectionCount;

            //
            // A single intersection is provisionally valid.
            //
            if (intersectionCount == 1) {
                point.assignment = SegmentAssignment::Assigned;
                point.assignedSegmentId = segment.id;
                point.referenceRange_m = distance_m;
            }
        }

        //
        // More than one segment intersects this ray.
        // The geometry is ambiguous and must eventually
        // be resolved by trimming.
        //

        if (intersectionCount > 1) {
            point.assignment = SegmentAssignment::Ambiguous;
            point.assignedSegmentId.reset();
            point.referenceRange_m = 0.0;
            point.rangeCorrection_m = 0.0;
            continue;
        }

        if (intersectionCount == 1) {
            point.rangeCorrection_m = point.referenceRange_m
                                      - point.measuredRange_m;
        }
    }

    return true;
}

//--------------------------------------------------------
//  BuildCalibrationMeasurements
//--------------------------------------------------------
bool L2RangeExtraction::BuildCalibrationMeasurements(
    std::vector<RangeCalibrationMeasurement>& measurements) const
{
    measurements.clear();
    measurements.reserve(mPoints.size());

    for (const Stage3BPoint& point : mPoints)
    {
        //
        // Must have an unambiguous calibration surface.
        //

        if (point.assignment != SegmentAssignment::Assigned) {
            continue;
        }

        if (!point.assignedSegmentId.has_value()){
            continue;
        }

        //
        // User exclusion always overrides calibration
        // segment membership.
        //
        if (point.excluded) {
            continue;
        }
        // these units are in meters
        if(point.measuredRange_m < mMinRange_m ||
            point.measuredRange_m > mMaxRange_m) {
            continue;
        }

        //
        // Reject-quality measurements are not permitted
        // in the calibration dataset.
        //
        // Suspect-quality measurements remain available for now.
        //
        if (point.quality == PointQuality::Reject) {
            continue;
        }

        RangeCalibrationMeasurement measurement;

        measurement.elevationIndex = point.elevationIndex;
        measurement.elevation = point.elevation;
        measurement.measuredRange_m = point.measuredRange_m;
        measurement.rangeStdDeviation_m = point.rangeStdDeviation_m;
        measurement.peakToPeakRange_m = point.peakToPeakRange_m;

        //
        // At this boundary the reference geometry has
        // survived the Stage 3B selection process and can
        // legitimately be treated as true range.
        //
        measurement.trueRange_m = point.referenceRange_m;
        measurement.correction_m = measurement.trueRange_m - measurement.measuredRange_m;

        measurements.emplace_back(measurement);
    }

    return !measurements.empty();
}

//--------------------------------------------------------
//  ExportMeasuredPointsCSV
//  intermediate analysis report for calculations and point classification
//--------------------------------------------------------
bool L2RangeExtraction::ExportMeasuredPointsCSV(
    const std::string& filename) const
{
    std::ofstream file(filename);

    if (!file) {
        return false;
    }

    file <<
        "ElevationIndex,"
        "Elevation_deg,"
        "MeasuredRange_m,"
        "X_m,"
        "Y_m,"
        "SampleCount,"
        "RangeStdDeviation_m,"
        "PeakToPeakRange_m,"
        "Quality,"
        "QualityFlags,"
        "Excluded,"
        "Assignment,"
        "AssignedSegmentId,"
        "ReferenceRange_m,"
        "RangeCorrection_m"
        "\n";

    file << std::fixed << std::setprecision(8);

    for (const Stage3BPoint& point : mPoints)
    {
        file
            << point.elevationIndex << ","
            << point.elevation << ","
            << point.measuredRange_m << ","
            << point.x_m << ","
            << point.y_m << ","
            << point.sampleCount << ","
            << point.rangeStdDeviation_m << ","
            << point.peakToPeakRange_m << ",";

        //Quality
        file << ToString(point.quality) << ",";
        file << point.qualityFlags << ",";

        //Excluded
        file << point.excluded << ",";

        //Assignment
        file << ToString(point.assignment) << ",";
        if (point.assignedSegmentId.has_value()) {
            file << point.assignedSegmentId.value() << ",";
        } else {
            file << ",";
        }
        //ReferenceRange_m
        file << point.referenceRange_m << ",";

        //RangeCorrection_m
        file << point.rangeCorrection_m << ",";

        file << "\n";

    }

    return true;
}

//--------------------------------------------------------
//  EvaluatePointQuality
//--------------------------------------------------------
void L2RangeExtraction::EvaluatePointQuality()
{
    for (Stage3BPoint& point : mPoints)
    {
        point.quality = PointQuality::Good;

        point.qualityFlags = QualityIssueNone;

        // Invalid range
        if ((point.measuredRange_m < mMinRange_m) ||
            (point.measuredRange_m > mMaxRange_m)) {

            point.qualityFlags |= InvalidRange;
            point.quality = PointQuality::Reject;
        }

        // Sample population size
        if (point.sampleCount < mMinimumSampleCount) {
            point.qualityFlags |= LowSampleCount;
            point.quality = PointQuality::Reject;
        }

        // stddevation of range exceeds 5cm
        if(point.rangeStdDeviation_m > 0.050) {
            point.qualityFlags |= RangeSpread;
            point.quality = PointQuality::Reject;
        }

        // nongaussian distribution
        if(point.rangeStdDeviation_m <= .001) {
            point.qualityFlags |= RangeSpread;
            point.quality = PointQuality::Reject;
        } else if((point.peakToPeakRange_m / point.rangeStdDeviation_m) > 10.0  ||
                   (point.peakToPeakRange_m / point.rangeStdDeviation_m) <= 3.0) {
            point.qualityFlags |= RangeSpread;
            if(point.quality !=PointQuality::Reject) {
                point.quality = PointQuality::Suspect;
            }
        }

        // peak to peak can not exceed 0.5 meters
        if(point.peakToPeakRange_m > 0.5) {
            point.qualityFlags |= HighPeakToPeakRange;
            if(point.quality !=PointQuality::Reject) {
                point.quality = PointQuality::Suspect;
            }
        }
        //
        // Angular consistency.
        //
        // if (point.previousElevationStep > 0.0)
        // {
        //     const double deviation =
        //         std::abs(
        //             point.previousElevationStep -
        //             mNominalElevationStep);

        //     if (deviation >
        //         mRejectStepDeviation)
        //     {
        //         point.qualityFlags |=
        //             PreviousStepDeviation;

        //         point.quality =
        //             PointQuality::Reject;
        //     }
        //     else if (
        //         deviation >
        //             mSuspectStepDeviation &&
        //         point.quality ==
        //             PointQuality::Good)
        //     {
        //         point.qualityFlags |=
        //             PreviousStepDeviation;

        //         point.quality =
        //             PointQuality::Suspect;
        //     }
        // }

        //
        // Same treatment for nextElevationStep.
        //
    }
}

//--------------------------------------------------------
//  ExportMeasuredPointsCSV
//  intermediate analysis report for calculations and point classification
//--------------------------------------------------------
bool L2RangeExtraction::ExportCalMeasurementsCSV(const std::string& filename,
            const std::vector<RangeCalibrationMeasurement> measurements) const
{
    std::ofstream file(filename);

    if (!file) {
        return false;
    }

    file <<
        "ElevationIndex,"
        "Elevation_deg,"
        "MeasuredRange_m,"
        "stdev_m,"
        "peakTopeak_m,"
        "TrueRange_m,"
        "Correction_m"
        "\n";

    file << std::fixed << std::setprecision(8);

    for (const RangeCalibrationMeasurement& point : measurements)
    {
        file
            << point.elevationIndex << ","
            << point.elevation << ","
            << point.measuredRange_m << ","
            << point.rangeStdDeviation_m << ","
            << point.peakToPeakRange_m << ","
            << point.trueRange_m << ","
            << point.correction_m << ""
             << "\n";
    }

    return true;
}
