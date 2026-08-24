//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage5saveCalibration.h
//
//
//  Purpose:
//  Stage 5 save calibration file
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
// This is the Stage 5 save calibration file
//--------------------------------------------------------
#pragma once
#include "L2RangeCorrection.h"
#include "L2RangeExtraction.h"

class L2RangeCalibrationWriter
{
public:
    bool SaveCalibrationFile(
        const std::string& filename,
        const RangeCalibrationCandidate& candidate,
        const std::vector<RangeCalibrationMeasurement>& measurements,
        const RangeCalibrationInfo& info);

    const std::string&
    GetLastErrorMessage() const noexcept
    {
        return mLastErrorMessage;
    }

private:
    bool ValidateInput(
        const RangeCalibrationCandidate& candidate,
        const std::vector<RangeCalibrationMeasurement>& measurements,
        const RangeCalibrationInfo& info);

    bool WriteMetadata(
        std::ostream& stream,
        const RangeCalibrationInfo& info);

    bool WriteModel(
        std::ostream& stream,
        const RangeCalibrationCandidate& candidate);

    bool WriteCalibrationPoints(
        std::ostream& stream,
        const std::vector<RangeCalibrationMeasurement>& measurements);

    std::string mLastErrorMessage;
};
