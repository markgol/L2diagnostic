//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RangeAnalysisResult.h
//
//
//  Purpose:
//  Stage 3 data anaylsis for stage 3
//
//  V2.0.0 RC1 2026-08-02
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
//  This is used in the analysis portion of the
//  L2 range correction calibration stage 3
//--------------------------------------------------------
#pragma once

#include <cstdint>
#include <vector>

#include <QVector>


//
// Represents one range measurement assigned to an L2 elevation angle.
//
struct RangeSample
{
    double range_m {0.0};     ///< Geometric range in meters.

    // Geometric angles reconstructed from QVector3D position.
    double azimuth {0.0};
    double elevation {0.0};
};

struct ElevationHistogramBin
{
    double centerElevation {0.0};   ///< Degrees.
    uint32_t sampleCount {0};
    double filteredCount {0.0};     ///< Gaussian-filtered response.
};

//
// Contains the measurements and statistical results for
// one nominal L2 elevation angle.
//
struct ElevationBin
{
    uint16_t elevationIndex {0};

    // Initial expected L2 elevation angle.
    //
    // For the initial Stage 3A implementation this is derived from:
    //
    // AlphaAngleBias +
    //     elevationIndex * AlphaAngleStepSize
    //
    // This is only an initial estimate. BetaAngle and XiAngle may
    // alter the actual geometric elevation.

    double previousElevationStep {0.0}; ///< Degrees.
    double nextElevationStep {0.0};     ///< Degrees.

    std::vector<RangeSample> samples;

    //
    // Statistical results.
    //
    // These will be populated after initial elevation assignment
    // has been validated.
    //

    uint32_t sampleCount {0};

    //
    // Elevation statistics.
    //
    double nominalElevation {0.0};      ///< Reference only, degrees.
    double meanElevation {0.0};         //< Mean reconstructed elevation, degrees.
    double elevationResidual {0.0};     //< meanElevation - nominalElevation,
                                        //< degrees.
    double minimumRange_m {0.0};
    double maximumRange_m {0.0};
    double meanRange_m {0.0};
    double standardDeviation_m {0.0};
    double peakToPeak_m {0.0};
};

//---------------------------------------------------------------
// ElevationPeak struct
//---------------------------------------------------------------
struct ElevationPeak
{
    size_t histogramIndex {0};
    double elevation {0.0};
    double filteredCount {0.0};

    double previousStep {0.0};
    double nextStep {0.0};
};

//---------------------------------------------------------------
// class definition
//---------------------------------------------------------------
//
// Contains the output of Stage 3A range data analysis.
//
class RangeAnalysisResult
{
public:
    RangeAnalysisResult() = default;

    void Clear();

    std::vector<RangeSample> samples;

    QVector<ElevationBin> elevationBins;

    std::vector<ElevationHistogramBin> elevationHistogram;

    std::vector<ElevationPeak> elevationPeaks;

    uint32_t totalPointCount {0};
};

