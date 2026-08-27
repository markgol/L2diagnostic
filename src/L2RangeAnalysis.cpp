//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2RangeAnalysis.h
//
//
//  Purpose:
//  Stage 1 range correction calibration for the L2
//  Metadata entry
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
#include "L2RangeAnalysis.h"
#include <fstream>
#include <iomanip>

namespace
{
constexpr double PI = 3.1415926535897932384626433832795;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;
}

//--------------------------------------------------------
//  SetElevationHistogramBinWidth
//--------------------------------------------------------
void L2RangeAnalysis::SetElevationHistogramBinWidth(double degrees)
{
    if (degrees > 0.0) {
        mElevationHistogramBinWidth = degrees;
    }
}

//--------------------------------------------------------
//  AnalyzePointCloud
//--------------------------------------------------------
bool L2RangeAnalysis::AnalyzePointCloud(const QVector<GLPoint>& cloud,
                                        RangeAnalysisResult& result)
{
    result.Clear();

    if (cloud.empty()) {
        return false;
    }

    //
    // Point conversion and elevation assignment
    //
    //  2 pass process
    //
    // Pass 1
    //
    // Determine the horizontal axis of the flattened scan plane.
    //
    // Scans are collected for a narrow azimuth range.
    // These scans are flattened into a 2d LiDAR scan
    // sampling a hemishpere (180 degrees) in the fast scan
    // direction
    // The data will appear to consist of just 1 single azimuth slice
    // that is not necesarily at 0 azimuth
    //
    // First pass determines what the mean azimuth
    //
    double sumCos2Azimuth = 0.0;
    double sumSin2Azimuth = 0.0;

    uint32_t azimuthPointCount = 0;

    for (const GLPoint& point : cloud)
    {
        const double x = static_cast<double>(point.pos.x());
        const double y = static_cast<double>(point.pos.y());

        //
        // Azimuth is undefined for a point directly on the Z axis.
        // Such a point remains perfectly valid for range/elevation,
        // but contributes no information about scan-plane azimuth.
        //

        if (x == 0.0 && y == 0.0) {
            continue;
        }

        //
        // L2 coordinate convention:
        //
        // +X = right
        // +Y = forward
        // positive azimuth = CCW
        //

        const double azimuth = std::atan2(x, y);
        sumCos2Azimuth += std::cos(2.0 * azimuth);
        sumSin2Azimuth += std::sin(2.0 * azimuth);

        ++azimuthPointCount;
    }

    if (azimuthPointCount == 0) {
        return false;
    }

    double scanAzimuth = 0.5 * std::atan2( sumSin2Azimuth, sumCos2Azimuth);

    //
    // atan2() above determines an axis, not a direction.
    // Normalize the representative scan-plane azimuth
    //

    if (scanAzimuth < 0.0) {
        scanAzimuth += PI;
    }

    const double scanX = std::sin(scanAzimuth);
    const double scanY = std::cos(scanAzimuth);

    //
    // Pass 2
    //
    // Convert every GLPoint into geometric range, azimuth,
    // and fast-scan elevation.
    //
    // The goal here is to convert the x,y,z data back into the
    // native L2 scan measurement which is:
    //      azimuth, elevation and range
    // All point for a given scan angle will lie on a straight angular line
    //
    result.samples.reserve(static_cast<size_t>(cloud.size()));

    for (const GLPoint& point : cloud)
    {
        const double x = static_cast<double>(point.pos.x());
        const double y = static_cast<double>(point.pos.y());
        const double z = static_cast<double>(point.pos.z());

        RangeSample sample;

        //
        // Do not use GLPoint::range here.
        // Geometric range is reconstructed from the point position.
        //

        sample.range_m = std::sqrt(x * x + y * y + z * z);

        //
        // Geometric azimuth.
        //

        double azimuth = std::atan2(x, y) * RAD_TO_DEG;

        if (azimuth < 0.0) {
            azimuth += 360.0;
        }
        sample.azimuth = azimuth;

        //
        // Project the horizontal component of the point onto
        // the flattened scan-plane axis.
        //
        // The sign distinguishes the two halves of the fast scan:
        //
        //   positive : nominal 0...90 degree half
        //   negative : nominal 90...180 degree half
        //

        const double signedHorizontal = x * scanX + y * scanY;

        double elevation = std::atan2(z,signedHorizontal) * RAD_TO_DEG;

        //
        // Normally the captured L2 scan should produce elevation
        // values in the 0...180 degree range.  Do not clamp here;
        // preserve unexpected geometry for diagnostic analysis.
        //

        if (elevation < 0.0) {
            elevation += 360.0;
        }

        sample.elevation = elevation;
        result.samples.emplace_back(sample);
    }

    result.totalPointCount = static_cast<uint32_t>(result.samples.size());

    //
    // Build measured elevation populations directly from
    // the reconstructed point geometry.
    //

    // this just build a elevation histogram
    if (!BuildElevationHistogram(result)) {
        return false;
    }

    // This runs a gaussian convolution filter across the histrogram
    // to locate peaks
    if (!FilterElevationHistogram(result)) {
        return false;
    }

    // look for the peaks and identify peak candidates
    if (!FindElevationPeakCandidates(result)) {
        return false;
    }

    // build the Elevation bins (peaks, range points within each elevation)
    if (!BuildElevationBins(result)) {
        return false;
    }

    // calculate the range stats for each elevation angle identified
    CalculateElevationBinStatistics(result);

    //
    // bin stats checkpoint
    //
    //
    // Verify that every converted sample was assigned exactly once.
    //

    uint64_t assignedPointCount = 0;

    for (const ElevationBin& bin : result.elevationBins)
    {
        assignedPointCount += bin.samples.size();
    }

    if (assignedPointCount != result.samples.size()) {
        // if this isn't true something has gone horribly wrong
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  BuildElevationHistogram
//--------------------------------------------------------
bool L2RangeAnalysis::BuildElevationHistogram(
    RangeAnalysisResult& result)
{
    if (result.samples.empty()) {
        return false;
    }

    if (mElevationHistogramBinWidth <= 0.0) {
        return false;
    }

    // adding needed for the histogram ends
    // for the convolution to operate correctly

    double minimumElevation = result.samples.front().elevation;

    double maximumElevation = result.samples.front().elevation;

    for (const RangeSample& sample : result.samples)
    {
        minimumElevation = std::min(minimumElevation, sample.elevation);
        maximumElevation =std::max(maximumElevation, sample.elevation);
    }

    //
    // Align the histogram limits with padding space at each end
    //
    const double padding = mElevationFilterRadiusSigma * mElevationFilterSigma;

    const double histogramMinimum = std::floor((minimumElevation - padding) /
                                               mElevationHistogramBinWidth) * mElevationHistogramBinWidth;

    const double histogramMaximum = std::ceil((maximumElevation + padding) /
                                              mElevationHistogramBinWidth) * mElevationHistogramBinWidth;

    const size_t numberOfBins = static_cast<size_t>(std::ceil((histogramMaximum -histogramMinimum) /
                                                              mElevationHistogramBinWidth)) + 1;

    result.elevationHistogram.clear();
    result.elevationHistogram.resize(numberOfBins);

    for (size_t i = 0; i < numberOfBins; ++i)
    {
        result.elevationHistogram[i].centerElevation =
                        histogramMinimum + (static_cast<double>(i) + 0.5) *
                        mElevationHistogramBinWidth;
    }

    for (const RangeSample& sample : result.samples)
    {
        const double relativeElevation = sample.elevation - histogramMinimum;

        if (relativeElevation < 0.0) {
            continue;
        }

        const size_t binIndex = static_cast<size_t>(relativeElevation / mElevationHistogramBinWidth);

        if (binIndex < result.elevationHistogram.size()) {
            ++result.elevationHistogram[
                        binIndex].sampleCount;
        }
    }

    return true;
}

//--------------------------------------------------------
//  FilterElevationHistogram
//--------------------------------------------------------
bool L2RangeAnalysis::FilterElevationHistogram(
    RangeAnalysisResult& result)
{
    if (result.elevationHistogram.empty()) {
        return false;
    }

    if (mElevationHistogramBinWidth <= 0.0 || mElevationFilterSigma <= 0.0){
        return false;
    }

    const double sigmaBins = mElevationFilterSigma / mElevationHistogramBinWidth;

    //
    // Use +/- 4 sigma.
    //
    const int radius = static_cast<int>(std::ceil(mElevationFilterRadiusSigma * sigmaBins));

    std::vector<double> kernel(static_cast<size_t>(2 * radius + 1));

    double kernelSum = 0.0;

    for (int i = -radius; i <= radius; ++i)
    {
        const double x = static_cast<double>(i);

        const double value = std::exp(-(x * x) / (2.0 * sigmaBins * sigmaBins));

        kernel[static_cast<size_t>(i + radius)] = value;
        kernelSum += value;
    }

    //
    // Normalize the kernel so a flat histogram
    // remains unchanged after filtering.
    //

    for (double& value : kernel)
    {
        value /= kernelSum;
    }

    //
    // Convolution.
    //

    const size_t histogramSize = result.elevationHistogram.size();

    for (size_t i = 0; i < histogramSize; ++i)
    {
        double filtered = 0.0;

        for (int k = -radius; k <= radius; ++k)
        {
            const int sourceIndex = static_cast<int>(i) + k;

            if (sourceIndex < 0 || sourceIndex >= static_cast<int>(histogramSize)) {
                continue;
            }

            filtered += static_cast<double>(
                result.elevationHistogram[ static_cast<size_t>(sourceIndex)].sampleCount) *
                kernel[static_cast<size_t>(k + radius)];
        }

        result.elevationHistogram[i].filteredCount = filtered;
    }

    return true;
}

//--------------------------------------------------------
//  ExportFilteredElevationHistogramCSV
//--------------------------------------------------------
bool L2RangeAnalysis::ExportFilteredElevationHistogramCSV(
    const std::string& filename,
    const RangeAnalysisResult& result) const
{
    std::ofstream file(filename);

    if (!file) {
        return false;
    }

    file <<
        "BinIndex,"
        "ElevationCenter_deg,"
        "RawCount,"
        "FilteredCount\n";

    file << std::fixed << std::setprecision(8);

    for (size_t i = 0; i < result.elevationHistogram.size(); ++i)
    {
        const ElevationHistogramBin& bin = result.elevationHistogram[i];

        file
            << i << ","
            << bin.centerElevation << ","
            << bin.sampleCount << ","
            << bin.filteredCount
            << "\n";
    }

    return true;
}

//--------------------------------------------------------
//  FindElevationPeakCandidates
//--------------------------------------------------------
bool L2RangeAnalysis::FindElevationPeakCandidates(RangeAnalysisResult& result)
{
    result.elevationPeaks.clear();
    const auto& histogram = result.elevationHistogram;

    if (histogram.size() < 3) {
        return false;
    }

    for (size_t i = 1; i + 1 < histogram.size(); ++i)
    {
        const double previous = histogram[i - 1].filteredCount;
        const double current = histogram[i].filteredCount;
        const double next = histogram[i + 1].filteredCount;

        if (current > previous && current >= next) {
            ElevationPeak peak;

            peak.histogramIndex = i;
            peak.elevation = histogram[i].centerElevation;
            peak.filteredCount = current;
            result.elevationPeaks.emplace_back(peak);
        }
    }

    return !result.elevationPeaks.empty();
}

//--------------------------------------------------------
//  BuildElevationBins
//--------------------------------------------------------
bool L2RangeAnalysis::BuildElevationBins(RangeAnalysisResult& result)
{
    result.elevationBins.clear();

    if (result.samples.empty() || result.elevationPeaks.empty()) {
        return false;
    }

    // sort samples by elevation angle
    // This is required by the sequentail peak assignment
    std::sort(
        result.samples.begin(),
        result.samples.end(),
        [](const RangeSample& a,
           const RangeSample& b)
        {
            return a.elevation < b.elevation;
        });

    const size_t peakCount = result.elevationPeaks.size();

    result.elevationBins.resize( static_cast<qsizetype>(peakCount));

    //
    // Initialize one ElevationBin for each detected peak.
    //

    for (size_t i = 0; i < peakCount; ++i)
    {
        ElevationBin& bin = result.elevationBins[static_cast<qsizetype>(i)];

        bin.elevationIndex = static_cast<uint16_t>(i);

        //
        // For now this is the detected filtered histogram
        // peak location rather than a nominal angle.
        //

        bin.nominalElevation = result.elevationPeaks[i].elevation;
    }

    //
    // Assign each RangeSample to the nearest detected
    // elevation population.
    //
    // Boundaries are halfway between adjacent detected peaks.
    //

    size_t peakIndex = 0;

    for (const RangeSample& sample : result.samples)
    {
        //
        // Advance while the sample lies beyond the midpoint
        // between the current peak and the next peak.
        //

        while (peakIndex + 1 < peakCount)
        {
            const double boundary =
                0.5 *
                (result.elevationPeaks[peakIndex].elevation +
                 result.elevationPeaks[peakIndex + 1].elevation);

            if (sample.elevation <= boundary) {
                break;
            }
            ++peakIndex;
        }

        result.elevationBins[ static_cast<qsizetype>(peakIndex)].samples.emplace_back(sample);
    }

    return true;
}

//--------------------------------------------------------
//  CalculateElevationBinStatistics
//--------------------------------------------------------
void L2RangeAnalysis::CalculateElevationBinStatistics(
    RangeAnalysisResult& result)
{
    for (ElevationBin& bin : result.elevationBins)
    {
        bin.sampleCount = static_cast<uint32_t>(bin.samples.size());

        if (bin.samples.empty()) {
            continue;
        }

        double elevationSum = 0.0;
        double rangeSum_m = 0.0;

        double minimumRange_m = bin.samples.front().range_m;

        double maximumRange_m = bin.samples.front().range_m;

        for (const RangeSample& sample : bin.samples)
        {
            elevationSum += sample.elevation;
            rangeSum_m += sample.range_m;

            minimumRange_m = std::min(minimumRange_m, sample.range_m);
            maximumRange_m = std::max(maximumRange_m, sample.range_m);
        }

        const double count = static_cast<double>(bin.samples.size());

        bin.meanElevation = elevationSum / count;
        bin.meanRange_m = rangeSum_m / count;
        bin.minimumRange_m = minimumRange_m;
        bin.maximumRange_m = maximumRange_m;
        bin.peakToPeak_m = maximumRange_m - minimumRange_m;

        //
        // Population standard deviation.
        //

        double varianceSum_m = 0.0;

        for (const RangeSample& sample : bin.samples)
        {
            const double delta_m = sample.range_m - bin.meanRange_m;
            varianceSum_m += delta_m * delta_m;
        }

        bin.standardDeviation_m = std::sqrt(varianceSum_m / count);
    }

    for (size_t i = 0; i < result.elevationPeaks.size(); ++i)
    {
        ElevationPeak& peak = result.elevationPeaks[i];

        if (i > 0) {
            peak.previousStep = peak.elevation - result.elevationPeaks[i - 1].elevation;
        }

        if (i + 1 < result.elevationPeaks.size()) {
            peak.nextStep = result.elevationPeaks[i + 1].elevation - peak.elevation;
        }
    }

    for (qsizetype i = 0; i < result.elevationBins.size(); ++i)
    {
        ElevationBin& bin = result.elevationBins[i];

        if (i > 0) {
            bin.previousElevationStep = bin.meanElevation - result.elevationBins[i - 1].meanElevation;
        }

        if (i + 1 < result.elevationBins.size()) {
            bin.nextElevationStep = result.elevationBins[i + 1].meanElevation - bin.meanElevation;
        }
    }
}

//--------------------------------------------------------
//  ExportElevationPeaksCSV
//--------------------------------------------------------
bool L2RangeAnalysis::ExportElevationPeaksCSV(
    const std::string& filename,
    const RangeAnalysisResult& result) const
{
    std::ofstream file(filename);

    if (!file) {
        return false;
    }

    file <<
        "PeakIndex,"
        "HistogramIndex,"
        "Elevation_deg,"
        "FilteredCount,"
        "PreviousStep_deg,"
        "NextStep_deg\n";

    file << std::fixed << std::setprecision(8);

    for (size_t i = 0; i < result.elevationPeaks.size(); ++i)
    {
        const ElevationPeak& peak = result.elevationPeaks[i];

        file
            << i << ","
            << peak.histogramIndex << ","
            << peak.elevation << ","
            << peak.filteredCount << ","
            << peak.previousStep << ","
            << peak.nextStep
            << "\n";
    }

    return true;
}

//--------------------------------------------------------
//  ExportAnalysisCSV
//--------------------------------------------------------
bool L2RangeAnalysis::ExportAnalysisCSV(
        const std::string& filename,
        const RangeAnalysisResult& result) const
{
    std::ofstream file(filename);

    if (!file) {
        return false;
    }

    file <<
        "ElevationIndex,"
        "PeakElevation_deg,"
        "MeanElevation_deg,"
        "SampleCount,"
        "MeanRange_m,"
        "MinimumRange_m,"
        "MaximumRange_m,"
        "PeakToPeakRange_m,"
        "StandardDeviation_m\n";

    file << std::fixed << std::setprecision(8);

    for (const ElevationBin& bin : result.elevationBins)
    {
        file
            << bin.elevationIndex << ","
            << bin.nominalElevation << ","
            << bin.meanElevation << ","
            << bin.sampleCount << ","
            << bin.meanRange_m << ","
            << bin.minimumRange_m << ","
            << bin.maximumRange_m << ","
            << bin.peakToPeak_m << ","
            << bin.standardDeviation_m
            << "\n";
    }

    return true;
}
