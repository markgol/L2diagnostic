//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2RangeAnalysis.h
//
//  Purpose:
//  Stage 3 data anaylsis for stage 3
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
//  This is used in the analysis portion of the
//  L2 range correction calibration stage 3
//--------------------------------------------------------
#ifndef L2RANGEANALYSIS_H
#define L2RANGEANALYSIS_H

#include <array>
#include <cstdint>
#include <string>
#include <QVector>
#include <algorithm>
#include <utility>

#include "RangeAnalysisResult.h"

#include "PCfileIO.h" // needed for the definition of GLPoint structre
//---------------------------------------------------------------
// class definition
//---------------------------------------------------------------
//
// Used in Stage 3A analysis of captured L2 range data.
//
class L2RangeAnalysis
{
public:
    static constexpr uint16_t NUM_ELEVATION_ANGLES = 300;

    L2RangeAnalysis() = default;

    void SetBinSize(double p) {mElevationHistogramBinWidth  = p;}

    void SetFilterSigma(double p)
    {
        if(p>0.0) {
           mElevationFilterSigma  = p;
        }
    }

    void SetFilterRadiusSigma(double p)
    {
        if(p>0.0) {
            mElevationFilterRadiusSigma  = p;
        }
    }

    bool AnalyzePointCloud(
        const QVector<GLPoint>& cloud,
        RangeAnalysisResult& result);

    bool ExportFilteredElevationHistogramCSV(
        const std::string& filename,
        const RangeAnalysisResult& result) const;

    bool ExportElevationPeaksCSV(
        const std::string& filename,
        const RangeAnalysisResult& result) const;

    bool ExportAnalysisCSV(
        const std::string& filename,
        const RangeAnalysisResult& result) const;

    bool FindElevationPeakCandidates(RangeAnalysisResult& result);

private:

private:

    bool BuildElevationBins(RangeAnalysisResult& result);
    void CalculateElevationBinStatistics(RangeAnalysisResult& result);
    bool BuildElevationHistogram(RangeAnalysisResult& result);
    void SetElevationHistogramBinWidth(double degrees);
    bool FilterElevationHistogram(RangeAnalysisResult& result);

    double mElevationHistogramBinWidth  {0.01};  // Degrees.
    double mElevationFilterSigma {0.04}; // degrees
    double mElevationFilterRadiusSigma {4.0};
};


#endif // L2RANGEANALYSIS_H
