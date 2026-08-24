//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: Stage4CalRangeClasses.cpp
//
//
//  Purpose:
//  Stage 4 correction computation
//
//  V2.0.0 RC1 2026-07-31
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
#include "Stage4CalRangeClasses.h"
#include <algorithm>
#include <cmath>
#include <QFileInfo>
#include <QDir>
#include <fstream>
#include <iomanip>
#include "settingINI.h"

//--------------------------------------------------------
//  ProcessCalibrationMeasurements
//--------------------------------------------------------
bool L2RangeCalibrationFit::ProcessCalibrationMeasurements(
    const std::vector<RangeCalibrationMeasurement>& measurements,
    const int NumSplineSegments,
    const double MinRange_m,
    const double MaxRange_m)
{
    // load measurement from stage3, validate, sort by range
    if(!SetMeasurements(measurements))
        return false;

    // check measurements are valid, no missing data
    if (!AnalyzeLocalConsistency()){
        return false;
    }

    // check range fit diagnostics
    if(!EvaluateFitDisposition()){
        return false;
    }

    // fit points
    if(!BuildFitDataSet()) {
        return false;
    }

    if(!BuildSplineSegments(NumSplineSegments)) {
        return false;
    }

    if(!FitCubicSpline()) {
        return false;
    }

    // prep data for generation of calibration file
    // and candidate correction application
    if (!BuildRangeCalibrationCandidate(MinRange_m,MaxRange_m))
    {
        return false;
    }

    // save result in CSV file
    ExportStage4CSV();
    ExportStage4CSVspline();

    return true;
}

//--------------------------------------------------------
//  SetMeasurements
//--------------------------------------------------------
bool L2RangeCalibrationFit::SetMeasurements(
    const std::vector<RangeCalibrationMeasurement>& measurements)
{
    mLastErrorMessage.clear();
    mMeasurements.clear();

    if (measurements.empty()) {
        mLastErrorMessage = "No range calibration measurements provided.";
        return false;
    }

    mMeasurements = measurements;

    if (!ValidateMeasurements()) {
        mMeasurements.clear();
        return false;
    }

    std::sort(
        mMeasurements.begin(),
        mMeasurements.end(),
        [](const RangeCalibrationMeasurement& a,
           const RangeCalibrationMeasurement& b)
        {
            return a.measuredRange_m <
                   b.measuredRange_m;
        });

    mMinMeasurementRange_m = mMeasurements.front().measuredRange_m;
    mMaxMeasurementRange_m = mMeasurements.back().measuredRange_m;
    mCandidateMinRange_m = mMeasurements.front().measuredRange_m;
    mCandidateMaxRange_m = mMeasurements.back().measuredRange_m;

    return true;
}

//--------------------------------------------------------
//  ValidateMeasurements
//--------------------------------------------------------
bool L2RangeCalibrationFit::ValidateMeasurements()
{
    for (const RangeCalibrationMeasurement& measurement :
         mMeasurements)
    {
        if (!std::isfinite(measurement.measuredRange_m) ||
            !std::isfinite(measurement.trueRange_m) ||
            !std::isfinite(measurement.correction_m) ||
            !std::isfinite(measurement.rangeStdDeviation_m) ||
            !std::isfinite(measurement.peakToPeakRange_m)) {

            mLastErrorMessage =
                "Range calibration measurement contains "
                "a non-finite value.";
            return false;
        }

        if (measurement.measuredRange_m <= 0.0) {
            mLastErrorMessage =
                "Range calibration measurement contains "
                "an invalid measured range.";
            return false;
        }

        if (measurement.trueRange_m <= 0.0) {
            mLastErrorMessage =
                "Range calibration measurement contains "
                "an invalid true range.";
            return false;
        }

        if (measurement.rangeStdDeviation_m < 0.0 ||
            measurement.peakToPeakRange_m < 0.0) {
            mLastErrorMessage =
                "Range calibration measurement contains "
                "invalid range statistics.";
            return false;
        }
    }

    return true;
}

//--------------------------------------------------------
//  AnalyzeLocalConsistency
//--------------------------------------------------------
bool L2RangeCalibrationFit::AnalyzeLocalConsistency()
{
    mDiagnostics.clear();

    if (mMeasurements.empty()) {
        mLastErrorMessage = "No measurements available for local consistency analysis.";
        return false;
    }

    mDiagnostics.resize(mMeasurements.size());

    for (size_t i = 0; i < mMeasurements.size(); ++i)
    {
        const double centerRange_m = mMeasurements[i].measuredRange_m;

        double correctionSum_m = 0.0;
        uint32_t count = 0;

        //
        // First pass: local correction mean.
        //
        for (size_t j = 0; j < mMeasurements.size(); ++j)
        {
            const double rangeDifference_m = std::abs(mMeasurements[j].measuredRange_m -centerRange_m);

            if (rangeDifference_m > mLocalRangeWindow_m) {
                continue;

            }

            correctionSum_m += mMeasurements[j].correction_m;
            ++count;
        }

        RangeFitDiagnostic& diagnostic = mDiagnostics[i];

        diagnostic.measurementIndex = i;
        diagnostic.neighborCount = count;

        if (count == 0) {
            continue;
        }
        //This computes the mean of a local group
        diagnostic.localMeanCorrection_m = correctionSum_m / static_cast<double>(count);
        //
        // Second pass: local correction spread.
        //
        double varianceSum_m = 0.0;

        for (size_t j = 0; j < mMeasurements.size(); ++j)
        {
            const double rangeDifference_m = std::abs(mMeasurements[j].measuredRange_m - centerRange_m);

            if (rangeDifference_m > mLocalRangeWindow_m){
                continue;
            }

            const double delta_m = mMeasurements[j].correction_m - diagnostic.localMeanCorrection_m;

            varianceSum_m += delta_m * delta_m;
        }

        if (count > 1) {
            diagnostic.localCorrectionStdDeviation_m =
                std::sqrt(varianceSum_m / static_cast<double>(count - 1));
        }
        diagnostic.correctionResidual_m = mMeasurements[i].correction_m - diagnostic.localMeanCorrection_m;
        if(count >= 3) { // only count a local neoghborhood if there is at least 3 samples in it
            diagnostic.localStatisticsValid = true;
        } else {
            diagnostic.localStatisticsValid = false;
        }
    }

    return true;
}

//--------------------------------------------------------
//  BuildFitDataSet
//--------------------------------------------------------
bool L2RangeCalibrationFit::BuildFitDataSet()
{
    mFitMeasurements.clear();

    if (mMeasurements.size() != mDiagnostics.size()){
        mLastErrorMessage = "Measurement and diagnostic counts differ.";
        return false;
    }

    for (size_t i = 0; i < mMeasurements.size(); ++i)
    {
        if (mDiagnostics[i].fitStatus != FitDisposition::Include) {
            continue;
        }
        mFitMeasurements.push_back(mMeasurements[i]);
    }

    return !mFitMeasurements.empty();
}

//--------------------------------------------------------
//  BuildSplineSegments
//--------------------------------------------------------
bool L2RangeCalibrationFit::BuildSplineSegments(
    uint32_t segmentCount)
{
    mLastErrorMessage.clear();
    mSplineSegments.clear();

    if (mFitMeasurements.empty()) {
        mLastErrorMessage = "No fit measurements are available.";
        return false;
    }

    if (segmentCount == 0) {
        mLastErrorMessage = "Spline segment count must be greater than zero.";
        return false;
    }

    if (segmentCount > mFitMeasurements.size()) {
        mLastErrorMessage = "Spline segment count exceeds the number of fit measurements.";
        return false;
    }

    mSplineSegments.resize(segmentCount);

    const size_t measurementCount = mFitMeasurements.size();
    const size_t baseCount = measurementCount / static_cast<size_t>(segmentCount);
    const size_t remainder = measurementCount % static_cast<size_t>(segmentCount);
    size_t measurementIndex = 0;

    for (size_t segmentIndex = 0; segmentIndex < mSplineSegments.size(); ++segmentIndex)
    {
        CubicSplineSegment& segment = mSplineSegments[segmentIndex];

        const size_t count = baseCount + (segmentIndex < remainder ? 1 : 0);

        segment.measurementIndices.clear();
        segment.measurementIndices.reserve(count);

        for (size_t i = 0; i < count; ++i)
        {
            segment.measurementIndices.emplace_back(measurementIndex);
            ++measurementIndex;
        }

        segment.measurementCount = count;
        const size_t firstIndex = segment.measurementIndices.front();
        const size_t lastIndex = segment.measurementIndices.back();

        segment.x0 = mFitMeasurements[firstIndex].measuredRange_m;
        segment.x1 = mFitMeasurements[lastIndex].measuredRange_m;

        segment.a = 0.0;
        segment.b = 0.0;
        segment.c = 0.0;
        segment.d = 0.0;
        segment.rmsResidual = 0.0;
    }
    //--------------------------------------------------------
    //  Make spline segment domains contiguous
    //--------------------------------------------------------
    for (size_t i = 0; i + 1 < mSplineSegments.size(); ++i)
    {
        CubicSplineSegment& current = mSplineSegments[i];
        CubicSplineSegment& next = mSplineSegments[i + 1];

        const size_t currentLastIndex = current.measurementIndices.back();
        const size_t nextFirstIndex = next.measurementIndices.front();

        const double boundary = 0.5 * (mFitMeasurements[currentLastIndex].measuredRange_m +
                                       mFitMeasurements[nextFirstIndex].measuredRange_m);
        current.x1 = boundary;
        next.x0 = boundary;
    }

    mSplineSegments.front().x0 = mFitMeasurements.front().measuredRange_m;
    mSplineSegments.back().x1 = mFitMeasurements.back().measuredRange_m;

    //--------------------------------------------------------
    //  Validate spline segment construction
    //--------------------------------------------------------
    size_t assignedMeasurementCount = 0;

    for (size_t i = 0; i < mSplineSegments.size(); ++i)
    {
        const CubicSplineSegment& segment = mSplineSegments[i];

        if (segment.measurementIndices.empty()) {
            mLastErrorMessage = "A spline segment contains no measurements.";
            return false;
        }

        if (!(segment.x0 < segment.x1)) {
            mLastErrorMessage = "A spline segment has an invalid range domain.";
            return false;
        }

        if (i > 0) {
            const CubicSplineSegment& previous = mSplineSegments[i - 1];
            if (segment.x0 != previous.x1) {
                mLastErrorMessage = "Spline segment domains are not contiguous.";
                return false;
            }
        }

        assignedMeasurementCount += segment.measurementIndices.size();
    }

    if (assignedMeasurementCount != mFitMeasurements.size()) {
        mLastErrorMessage = "Not all fit measurements were assigned to spline segments.";
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  EvaluateFitDisposition
//--------------------------------------------------------
bool L2RangeCalibrationFit::EvaluateFitDisposition()
{
    if (mMeasurements.size() != mDiagnostics.size()) {
        mLastErrorMessage = "Measurement and diagnostic counts do not match.";
        return false;
    }

    for (size_t i = 0; i < mMeasurements.size(); ++i) {
        const RangeCalibrationMeasurement& measurement = mMeasurements[i];

        RangeFitDiagnostic& diagnostic = mDiagnostics[i];

        diagnostic.fitStatus = FitDisposition::Include;

        if (measurement.measuredRange_m < mCandidateMinRange_m ||
            measurement.measuredRange_m > mCandidateMaxRange_m) {

            diagnostic.fitStatus = FitDisposition::Exclude;
        }
    }

    return true;
}

//--------------------------------------------------------
//  FitCubicSpline
//--------------------------------------------------------
bool L2RangeCalibrationFit::FitCubicSpline()
{
    mLastErrorMessage.clear();

    if (mSplineSegments.empty()) {
        mLastErrorMessage = "No spline segments are available.";
        return false;
    }

    if (mFitMeasurements.empty()) {
        mLastErrorMessage = "No fit measurements are available.";
        return false;
    }

    const size_t segmentCount = mSplineSegments.size();

    const size_t coefficientCount = segmentCount * 4;

    const size_t measurementCount = mFitMeasurements.size();

    //
    // One C0, C1 and C2 continuity constraint at
    // every internal segment boundary.
    //
    const size_t constraintCount = (segmentCount > 1) ? (segmentCount - 1) * 3 : 0;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
            static_cast<Eigen::Index>(measurementCount),
            static_cast<Eigen::Index>(coefficientCount));
    Eigen::VectorXd y = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(measurementCount));

    //----------------------------------------------------
    // Build measurement equations.
    //----------------------------------------------------
    size_t row = 0;

    for (size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
    {
        const CubicSplineSegment& segment = mSplineSegments[segmentIndex];

        const size_t coefficientOffset = segmentIndex * 4;

        for (const size_t measurementIndex : segment.measurementIndices)
        {
            if (measurementIndex >= mFitMeasurements.size()) {
                mLastErrorMessage = "Spline segment contains an invalid measurement index.";
                return false;
            }

            const RangeCalibrationMeasurement& measurement = mFitMeasurements[measurementIndex];

            const double t = measurement.measuredRange_m - segment.x0;

            A(static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(coefficientOffset + 0)) = 1.0;

            A(static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(coefficientOffset + 1)) = t;

            A(static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(coefficientOffset + 2)) = t * t;

            A(static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(coefficientOffset + 3)) = t * t * t;

            y(static_cast<Eigen::Index>(row)) = measurement.correction_m;
            ++row;
        }
    }

    if (row != measurementCount) {
        mLastErrorMessage = "Spline measurement assignment count does not match the fit dataset.";
        return false;
    }

    //----------------------------------------------------
    // Build exact continuity constraints:
    //
    // C0: f_left(xBoundary) = f_right(xBoundary)
    // C1: f'_left           = f'_right
    // C2: f''_left          = f''_right
    //----------------------------------------------------
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(
            static_cast<Eigen::Index>(constraintCount),
            static_cast<Eigen::Index>(coefficientCount));

    size_t constraintRow = 0;

    for (size_t i = 0; i + 1 < segmentCount; ++i)
    {
        const CubicSplineSegment& left = mSplineSegments[i];

        const double h = left.x1 - left.x0;

        if (!(h > 0.0)) {
            mLastErrorMessage = "Spline segment has a non-positive width.";
            return false;
        }

        const size_t leftOffset = i * 4;

        const size_t rightOffset = (i + 1) * 4;

        //
        // C0 continuity.
        //
        C(constraintRow, leftOffset + 0) = 1.0;
        C(constraintRow, leftOffset + 1) = h;
        C(constraintRow, leftOffset + 2) = h * h;
        C(constraintRow, leftOffset + 3) = h * h * h;

        C(constraintRow, rightOffset + 0) = -1.0;

        ++constraintRow;

        //
        // C1 continuity.
        //
        C(constraintRow, leftOffset + 1) = 1.0;
        C(constraintRow, leftOffset + 2) = 2.0 * h;
        C(constraintRow, leftOffset + 3) = 3.0 * h * h;

        C(constraintRow, rightOffset + 1) = -1.0;

        ++constraintRow;

        //
        // C2 continuity.
        //
        C(constraintRow, leftOffset + 2) = 2.0;
        C(constraintRow, leftOffset + 3) = 6.0 * h;

        C(constraintRow, rightOffset + 2) = -2.0;

        ++constraintRow;
    }

    //----------------------------------------------------
    // Solve constrained least squares through the KKT
    // system:
    // ( ᵀ is transpose)
    // [AᵀA  Cᵀ] [coefficients] = [Aᵀy]
    // [ C     0] [multipliers ]   [  0 ]
    //----------------------------------------------------
    Eigen::MatrixXd normal = A.transpose() * A;

    //
    // Very small regularization improves numerical
    // conditioning without materially changing the fit.
    //
    constexpr double REGULARIZATION = 1.0e-12;

    normal.diagonal().array() += REGULARIZATION;

    const Eigen::Index systemSize = static_cast<Eigen::Index>(
            coefficientCount + constraintCount);

    Eigen::MatrixXd system = Eigen::MatrixXd::Zero(
            systemSize, systemSize);

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(systemSize);

    system.block(0, 0, coefficientCount, coefficientCount) = normal;

    rhs.head(static_cast<Eigen::Index>(coefficientCount)) = A.transpose() * y;

    if (constraintCount > 0) {
        system.block(0, coefficientCount, coefficientCount, constraintCount) = C.transpose();

        system.block(coefficientCount, 0, constraintCount, coefficientCount) = C;
    }

    const Eigen::VectorXd solution = system.fullPivLu().solve(rhs);

    if (!solution.allFinite()) {
        mLastErrorMessage = "Cubic spline coefficient solution contains non-finite values.";
        return false;
    }

    const double relativeResidual = (system * solution - rhs).norm() / std::max(1.0, rhs.norm());

    constexpr double SOLUTION_TOLERANCE = 1.0e-8;

    if (relativeResidual > SOLUTION_TOLERANCE) {
        mLastErrorMessage =
            "Cubic spline coefficient system could not be solved accurately.";
        return false;
    }

    //----------------------------------------------------
    // Store coefficients.
    //----------------------------------------------------
    for (size_t i = 0; i < segmentCount; ++i)
    {
        CubicSplineSegment& segment = mSplineSegments[i];

        const size_t offset = i * 4;

        segment.a = solution[static_cast<Eigen::Index>(offset + 0)];

        segment.b = solution[static_cast<Eigen::Index>(offset + 1)];

        segment.c = solution[static_cast<Eigen::Index>(offset + 2)];

        segment.d = solution[static_cast<Eigen::Index>(offset + 3)];
    }

    return CalculateSplineResiduals();
}

//--------------------------------------------------------
//  CalculateSplineResiduals
//--------------------------------------------------------
bool L2RangeCalibrationFit::CalculateSplineResiduals()
{
    double totalSquaredResidual_m = 0.0;
    size_t totalCount = 0;

    for (CubicSplineSegment& segment : mSplineSegments)
    {
        double segmentSquaredResidual_m = 0.0;
        double MeanCorrectionSum_m = 0.0;

        for (const size_t measurementIndex : segment.measurementIndices)
        {
            const RangeCalibrationMeasurement& measurement = mFitMeasurements[measurementIndex];

            const double fittedCorrection_m = EvaluateSplineSegment(segment, measurement.measuredRange_m);
            MeanCorrectionSum_m += measurement.correction_m;
            const double residual_m =  measurement.correction_m - fittedCorrection_m;
            segmentSquaredResidual_m += residual_m * residual_m;
        }

        const size_t count = segment.measurementIndices.size();

        if (count == 0) {
            return false;
        }

        segment.meanCorrection = MeanCorrectionSum_m / static_cast<double>(count);
        segment.rmsResidual = std::sqrt(segmentSquaredResidual_m / static_cast<double>(count));

        totalSquaredResidual_m += segmentSquaredResidual_m;
        totalCount += count;
    }

    if (totalCount == 0)
    {

        return false;
    }

    mRMSResidual_m = std::sqrt( totalSquaredResidual_m / static_cast<double>(totalCount));

    return true;
}

//--------------------------------------------------------
//  EvaluateSplineSegment
//--------------------------------------------------------
double L2RangeCalibrationFit::EvaluateSplineSegment(
    const CubicSplineSegment& segment,
    double measuredRange) const
{
    const double t =
        measuredRange -
        segment.x0;

    return
        segment.a +
        segment.b * t +
        segment.c * t * t +
        segment.d * t * t * t;
}

//--------------------------------------------------------
//  BuildRangeCalibrationCandidate
//--------------------------------------------------------
bool L2RangeCalibrationFit::
    BuildRangeCalibrationCandidate(const double MinRange_m, const double MaxRange_m)
{
    if (mSplineSegments.empty()) {
        mLastErrorMessage = "No fitted spline segments are available.";
        return false;
    }

    mCandidateStage4.segments.clear();
    mCandidateStage4.calibrationMethod = "CubicSpline";
    mCandidateStage4.minRange = MinRange_m;
    mCandidateStage4.maxRange = MaxRange_m;
    mCandidateStage4.minCalRange = mSplineSegments.front().x0;
    mCandidateStage4.maxCalRange = mSplineSegments.back().x1;
    mCandidateStage4.rmsResidual = mRMSResidual_m;
    mCandidateStage4.segments.reserve(mSplineSegments.size());

    for (const CubicSplineSegment& segment : mSplineSegments)
    {
        RangeModelFields fields;

        fields.fieldCount = 6;

        fields.fields[0] = segment.x0;
        fields.fields[1] = segment.x1;
        fields.fields[2] = segment.a;
        fields.fields[3] = segment.b;
        fields.fields[4] = segment.c;
        fields.fields[5] = segment.d;

        mCandidateStage4.segments.emplace_back(fields);
    }

    mCandidateStage4.valid = true;

    return true;
}

//--------------------------------------------------------
//  ExportStage4CSV
//--------------------------------------------------------
void L2RangeCalibrationFit::ExportStage4CSV()
{
    if(mMeasurements.size()!=mDiagnostics.size()) {
        return;
    }
    // get stage3 CSV filename from INI file
    QString filename = loadINI("RangeCalStage3","CSVfilename",QString(""));
    if(!(filename.trimmed()=="")) {
        QFileInfo info(filename);
        // save analysis
        QString newFile = QDir(info.path()).filePath(
            info.completeBaseName() +
            "Stage4Info" +
            "." +
            info.completeSuffix());

        std::ofstream file(newFile.toStdString());

        if (!file) {
            return;
        }

        file <<
            "point#,"
            "ElevationIndex,"
            "Elevation,"
            "MeasuredRange_m,"
            "TrueRange_m,"
            "Correction_m,"
            "RangeStdDeviation_m,"
            "PeakToPeakRange_m,"
            "NeighborCount,"
            "localStatisticsValid,"
            "LocalMeanCorrection_m,"
            "LocalCorrectionStdDeviation_m,"
            "CorrectionResidual_m,"
            "FitStatus\n";

        file << std::fixed << std::setprecision(8);

        for (size_t i = 0; i < mMeasurements.size(); ++i)
        {
            file
                << i << ","
                << mMeasurements[i].elevationIndex << ","
                << mMeasurements[i].elevation << ","
                << mMeasurements[i].measuredRange_m << ","
                << mMeasurements[i].trueRange_m << ","
                << mMeasurements[i].correction_m << ","
                << mMeasurements[i].rangeStdDeviation_m << ","
                << mMeasurements[i].peakToPeakRange_m << ","
                << mDiagnostics[i].neighborCount << ","
                << mDiagnostics[i].localStatisticsValid << ","
                << mDiagnostics[i].localMeanCorrection_m << ","
                << mDiagnostics[i].localCorrectionStdDeviation_m << ","
                << mDiagnostics[i].correctionResidual_m << ",";
            file << FitToString(mDiagnostics[i].fitStatus);
            file << "\n";
        }
    }
}

//--------------------------------------------------------
//  ExportStage4CSV
//--------------------------------------------------------
void L2RangeCalibrationFit::ExportStage4CSVspline()
{
    if(mMeasurements.size()!=mDiagnostics.size()) {
        return;
    }
    // get stage3 CSV filename from INI file
    QString filename = loadINI("RangeCalStage3","CSVfilename",QString(""));
    if(!(filename.trimmed()=="")) {
        QFileInfo info(filename);
        // save analysis
        QString newFile = QDir(info.path()).filePath(
            info.completeBaseName() +
            "Stage4Spline" +
            "." +
            info.completeSuffix());

        std::ofstream file(newFile.toStdString());

        if (!file) {
            return;
        }

        file <<
            "Segment,"
            "x0,"
            "x1,"
            "MeasurementCount,"
            "a,"
            "b,"
            "c,"
            "d,"
            "SegmentRMSresidual_m,"
            "meanCorrection_m"
            "\n";

        file << std::fixed << std::setprecision(8);

        for (size_t i = 0; i < mSplineSegments.size(); ++i)
        {
            file
                << i << ","
                << mSplineSegments[i].x0 << ","
                << mSplineSegments[i].x1 << ","
                << mSplineSegments[i].measurementCount << ","
                << mSplineSegments[i].a << ","
                << mSplineSegments[i].b << ","
                << mSplineSegments[i].c << ","
                << mSplineSegments[i].d << ","
                << mSplineSegments[i].rmsResidual << ","
                << mSplineSegments[i].meanCorrection;
            file << "\n";
        }
        file << "\nOverallRMSresidual,";
        file << mRMSResidual_m;
        file << "\n";

    }
}
