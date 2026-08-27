//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RangeAnalysisResult.cpp
//
//
//  Purpose:
//  Stage 3 data anaylsis for stage 3
//
//  V2.0.0 RC1 2026-08-02
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

#include "RangeAnalysisResult.h"


void RangeAnalysisResult::Clear()
{
    samples.clear();
    elevationBins.clear();
    elevationHistogram.clear();
    elevationPeaks.clear();
    totalPointCount = 0;
}
