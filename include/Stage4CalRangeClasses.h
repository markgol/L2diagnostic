//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage4CalRangeClasses.h
//
//
//  Purpose:
//  Stage 4 correction computation
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
// This is the Stage 4 computation for range correction
//--------------------------------------------------------
#pragma once

#include <string>
#include <vector>
#include <Eigen>

#include "L2RangeExtraction.h"
#include "RangeCalSegments.h"

//--------------------------------------------------------
// enumerated class definitions
//--------------------------------------------------------
enum class FitDisposition
{
    Include,
    Suspect,
    Exclude
};

//--------------------------------------------------------
// non classed structures
//--------------------------------------------------------
struct RangeFitDiagnostic
{
    size_t measurementIndex {0};

    uint32_t neighborCount {0};

    bool localStatisticsValid {false};

    double localMeanCorrection_m {0.0};
    double localCorrectionStdDeviation_m {0.0};
    double correctionResidual_m {0.0};

    FitDisposition fitStatus {FitDisposition::Include};
};

//--------------------------------------------------------
// class L2RangeCalibrationFit definition
//--------------------------------------------------------
class L2RangeCalibrationFit
{
public:
    bool ProcessCalibrationMeasurements(
        const std::vector<RangeCalibrationMeasurement>& measurements,
        const int NumSplineSegments,
        const double MinRange_m,
        const double MaxRange_m);

    bool SetMeasurements(
        const std::vector<RangeCalibrationMeasurement>& measurements);

    const std::vector<RangeCalibrationMeasurement>& GetMeasurements() const noexcept
    {
        return mMeasurements;
    }

    bool AnalyzeLocalConsistency();

    bool EvaluateFitDisposition();

    void ExportStage4CSV();

    const std::string& GetLastErrorMessage() const noexcept
    {
        return mLastErrorMessage;
    }

    bool BuildFitDataSet();

    bool BuildSplineSegments(uint32_t segmentCount);

    bool FitCubicSpline();
    bool CalculateSplineResiduals();
    void ExportStage4CSVspline();
    bool BuildRangeCalibrationCandidate(const double MinRange_m, const double MaxRange_m); // in meters

    const RangeCalibrationCandidate& GetCandidate() const noexcept
    {
        return mCandidateStage4;
    }

    static const char* FitToString(
        FitDisposition assignment)
    {
        switch (assignment)
        {
        case FitDisposition::Exclude :
            return "Exclude";

        case FitDisposition::Include :
            return "Include";

        case FitDisposition::Suspect :
            return "Suspect";
        }
        return "Unknown";
    }


private:
    //=====================================================================
    //
    // Cubic Spline Segment
    //
    // f(x) = a + b(x-x0) + c(x-x0)^2 + d(x-x0)^3
    // Valid for x0 <= x <= x1
    //
    //  double x0 = 0.0; // field 0
    //  double x1 = 0.0; // field 1
    //  double a = 0.0; // field 2
    //  double b = 0.0; // field 3
    //  double c = 0.0; // field 4
    //  double d = 0.0; // field 5
    //
    //=====================================================================
    struct CubicSplineSegment {
        double x0 = 0.0; // field 0
        double x1 = 0.0; // field 1
        double a = 0.0; // field 2
        double b = 0.0; // field 3
        double c = 0.0; // field 4
        double d = 0.0; // field 5
        std::vector<size_t> measurementIndices;
        double rmsResidual {0.0};
        uint32_t measurementCount {0};
        double meanCorrection {0.0};
    };

    bool ValidateMeasurements();
    double EvaluateSplineSegment(const CubicSplineSegment& segment, double measuredRange) const;

    std::vector<RangeCalibrationMeasurement> mFitMeasurements;
    std::vector<RangeCalibrationMeasurement> mMeasurements;
    std::vector<RangeFitDiagnostic> mDiagnostics;
    std::vector<CubicSplineSegment> mSplineSegments;

    double mMinMeasurementRange_m {0.0};
    double mMaxMeasurementRange_m {0.0};
    double mCandidateMinRange_m {0.0};
    double mCandidateMaxRange_m {0.0};

    RangeCalibrationCandidate mCandidateStage4;

    double mRMSResidual_m {0.0};

    // local range criteria
    double mLocalRangeWindow_m {0.025};  // meters

    std::string mLastErrorMessage;
};
