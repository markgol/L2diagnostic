//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage5saveCalibration.cpp
//
//
//  Purpose:
//  Stage 5 save calibration file
//
//  V2.0.0 RC1 2026-08-15
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
// This is the Stage 5 save calibration file
//--------------------------------------------------------
#include <fstream>
#include <iomanip>
#include "Stage5saveCalibration.h"

//--------------------------------------------------------
//  SaveCalibrationFile
//--------------------------------------------------------
bool L2RangeCalibrationWriter::SaveCalibrationFile(
        const std::string& filename,
        const RangeCalibrationCandidate& candidate,
        const std::vector<RangeCalibrationMeasurement>& measurements,
        const RangeCalibrationInfo& info,
        const std::vector<double>& AlphaAngleLUT)
{
    mLastErrorMessage.clear();

    if (!ValidateInput(candidate,measurements,info)) {
        return false;
    }

    std::ofstream stream(filename);

    if (!stream.is_open()) {
        mLastErrorMessage =
            "Unable to open calibration file for writing: " +
            filename;
        return false;
    }

    stream << std::setprecision(8) << std::defaultfloat;

    if (!WriteMetadata(stream, info)){
        return false;
    }

    if (!WriteModel(stream, candidate)) {
        return false;
    }

    if (!WriteCalibrationPoints( stream, measurements)) {
        return false;
    }

    if(AlphaAngleLUT.size() == NUM_ALPHA_ANGLES_IN_SCAN) {
        if(!WriteAlphaAngleLUT( stream, AlphaAngleLUT)) {
            return false;
        }
    }

    stream.flush();

    if (!stream.good()) {
        mLastErrorMessage =
            "An error occurred while writing the calibration file.";
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  ValidateInput
//--------------------------------------------------------
bool L2RangeCalibrationWriter::ValidateInput(
    const RangeCalibrationCandidate& candidate,
    const std::vector<RangeCalibrationMeasurement>& measurements,
    const RangeCalibrationInfo& info)
{
    if (!candidate.valid) {
        mLastErrorMessage = "The range calibration candidate is not valid.";
        return false;
    }

    if (candidate.calibrationMethod != "CubicSpline") {
        mLastErrorMessage = "Unsupported correction method: " + candidate.calibrationMethod;
        return false;
    }

    if (candidate.segments.empty()) {
        mLastErrorMessage = "The candidate contains no model segments.";
        return false;
    }

    if (!(candidate.minRange < candidate.maxRange)) {
        mLastErrorMessage = "The physical range limits are invalid.";
        return false;
    }

    if (!(candidate.minCalRange < candidate.maxCalRange)) {
        mLastErrorMessage = "The calibrated range limits are invalid.";
        return false;
    }

    // minCalRange and maxCalRange are in meters, compare using mm
    if ((candidate.minCalRange) < candidate.minRange ||
        (candidate.maxCalRange) > candidate.maxRange) {
        mLastErrorMessage = "The calibrated range is outside the physical measurement range.";
        return false;
    }

    if (info.SensorID.empty()) {
        mLastErrorMessage = "SensorID is required.";
        return false;
    }

    if (info.Date.empty()) {
        mLastErrorMessage = "Calibration date is required.";
        return false;
    }

    if (measurements.empty()) {
        mLastErrorMessage = "No calibration measurements are available.";
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  WriteMetadata
//  note: calculated input distances are in meters and
//  are output in mms
//--------------------------------------------------------
bool L2RangeCalibrationWriter::WriteMetadata(
    std::ostream& stream,
    const RangeCalibrationInfo& info)
{
    stream
        << "# Version," << info.Version << '\n'
        << "# Date," << info.Date << '\n'
        << "# Sensor," << info.Sensor << '\n'
        << "# SensorID," << info.SensorID << '\n'
        << "# Firmware," << info.Firmware << '\n'
        << "# CreatedBy," << info.CreatedBy << '\n'
        << "# RangeCorrectionMethod,"
        << info.RangeCalMethod << '\n'
        << "# NumberRangeSegments,"
        << info.NumberOfSegments << '\n'
        << "# MinRange,"
        << info.MinRange << '\n' // user input already in mm
        << "# MaxRange,"
        << info.MaxRange << '\n' // user input already in mm
        << "# MinTrustedRange,"
        << info.MinTrustedRange << '\n' // user input already in mm
        << "# MinCalRange,"
        << info.MinCalRange << '\n' // already in mm
        << "# MaxCalRange,"
        << info.MaxCalRange << '\n' // already in mm
        << "# RMSResidual,"
        << info.RMSResidual << '\n' // already in mm
        << "# RangeBias,"
        << info.RangeBias << '\n'
        << "# RangeScale,"
        << info.RangeScale << '\n'
        << "# AlphaAngleBias,"
        << info.AlphaAngleBias << '\n'
        << "# AlphaAngleStepSize,"
        << info.AlphaAngleStepSize << '\n'
        << "# ThetaAngleBias,"
        << info.ThetaAngleBias << '\n'
        << "# BetaAngle,"
        << info.BetaAngle << '\n'
        << "# XiAngle,"
        << info.XiAngle << '\n'
        << "# CalibrationDescription,"
        << info.CalibrationDescription << '\n'
        << '\n';

    if (!stream.good())
    {
        mLastErrorMessage =
            "Failed while writing calibration metadata.";

        return false;
    }

    return true;
}

//--------------------------------------------------------
//  WriteModel
//--------------------------------------------------------
bool L2RangeCalibrationWriter::WriteModel(
    std::ostream& stream,
    const RangeCalibrationCandidate& candidate)
{
    stream << "# RANGE MODEL\n" << "x0,x1,a,b,c,d\n";

    for (const RangeModelFields& segment : candidate.segments)
    {
        if (segment.fieldCount != 6) {
            mLastErrorMessage =
                "A cubic spline segment does not contain "
                "exactly six fields.";
            return false;
        }

        stream
            << segment.fields[0] * 1000.0 << ',' // x0 convert m to mmm
            << segment.fields[1] * 1000.0 << ',' // x0 convert m to mmm
            << segment.fields[2] * 1000.0 << ',' // a convert m to mmm
            << segment.fields[3] << ','             // b no correction for m to mm
            << segment.fields[4] / 1000.0 << ','    // c needs /1000 for m to mm
            << segment.fields[5] / 1000000.0 << '\n'; // d needs /1000000 for m to mm
    }

    stream << '\n';

    if (!stream.good()) {
        mLastErrorMessage = "Failed while writing the calibration model.";
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  WriteCalibrationPoints
//--------------------------------------------------------
bool L2RangeCalibrationWriter::WriteCalibrationPoints(std::ostream& stream,
            const std::vector<RangeCalibrationMeasurement>& measurements)
{
    stream
        << "# CALIBRATION POINTS\n"
        << "MeasuredRange,TrueRange,Correction\n";

    for (const RangeCalibrationMeasurement& measurement : measurements)
    {
        stream
            << measurement.measuredRange_m << ','
            << measurement.trueRange_m << ','
            << measurement.correction_m << '\n';
    }

    if (!stream.good()) {
        mLastErrorMessage = "Failed while writing calibration points.";
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  WriteCalibrationPoints
//--------------------------------------------------------
bool L2RangeCalibrationWriter::WriteAlphaAngleLUT(std::ostream& stream,
                                const std::vector<double>& AlphaAngleLUT)
{
    stream
        << "\n# ALPHA ANGLE LUT\n"
        << "FastScanIndex, RelativeAngle\n";

    for (int i=0; i< NUM_ALPHA_ANGLES_IN_SCAN; i++)
    {
        stream
            << i << ','
            << AlphaAngleLUT[i] << '\n';
    }

    if (!stream.good()) {
        mLastErrorMessage = "Failed while writing calibration points.";
        return false;
    }

    return true;
}