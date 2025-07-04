/**
 * @file calib_utils.cpp
 * @author Tina Tian
 * @brief Utility functions for camera calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "calib_utils.h"
#include <opencv2/calib3d.hpp>

///@todo add range
CoverageChecker::CoverageChecker(
    size_t image_width, size_t image_height,
    size_t num_corner_bins,
    size_t num_size_bins,
    size_t num_skew_bins,
    size_t min_corners_per_cell,
    std::tuple<double, double> x_range,
    std::tuple<double, double> y_range,
    std::tuple<double, double> size_range,
    std::tuple<double, double> skew_range)
    : image_width_(image_width), image_height_(image_height),
      num_corner_bins_(num_corner_bins),
      num_size_bins_(num_size_bins),
      num_skew_bins_(num_skew_bins),
      min_corners_per_cell_(min_corners_per_cell),
      x_range_(x_range), y_range_(y_range),
      size_range_(size_range), skew_range_(skew_range)
{
    // create the bins filled with 0
    x_bins_.resize(num_corner_bins);
    y_bins_.resize(num_corner_bins);
    size_bins_.resize(num_size_bins);
    skew_bins_.resize(num_skew_bins);
}

void CoverageChecker::addObservation(
    const std::vector<cv::Point2f> &corners,
    double size, double skew)
{
    if (corners.empty())
        return; // No corners to process
    // Add to the coverage checker
    addCornersToBin(corners);

    addToBin(size_bins_, size, std::get<0>(size_range_), std::get<1>(size_range_));
    addToBin(skew_bins_, skew, std::get<0>(skew_range_), std::get<1>(skew_range_));
}

void CoverageChecker::addToBin(
    std::vector<int> &bins,
    double value, double min_val, double max_val)
{
    int num_bins = bins.size();
    if (max_val == min_val)
        return; // Avoid division by zero
    int index = static_cast<int>(((value - min_val) / (max_val - min_val)) * num_bins);
    index = clamp(index, 0, num_bins - 1);
    bins[index]++;
}

void CoverageChecker::addCornersToBin(
    const std::vector<cv::Point2f> &corners)
{
    // Convert corner to grid cell indices
    for (const auto &corner : corners)
    {
        addToBin(x_bins_, corner.x / image_width_,
                 std::get<0>(x_range_), std::get<1>(x_range_));
        addToBin(y_bins_, corner.y / image_height_,
                 std::get<0>(y_range_), std::get<1>(y_range_));
    }
}

CoverageChecker::CoverageMetrics CoverageChecker::evaluateCoverage()
{
    CoverageMetrics result{};

    auto computeCoverageScore = [this](const std::vector<int> &bins)
    {
        int filled = std::count_if(bins.begin(), bins.end(), [this](int v)
                                   { return v > min_corners_per_cell_; });
        return (static_cast<double>(filled) / bins.size()) * 100.0;
    };
    result.score_x = computeCoverageScore(x_bins_);
    result.score_y = computeCoverageScore(y_bins_);
    result.score_size = computeCoverageScore(size_bins_);
    result.score_skew = computeCoverageScore(skew_bins_);

    return result;
}

void CoverageChecker::printBins()
{
    // print the bin vectors
    std::cout << "Coverage Bins:\n";
    std::cout << "  X Bins: ";
    for (const auto &bin : x_bins_)
    {
        std::cout << bin << " ";
    }
    std::cout << "\n  Y Bins: ";
    for (const auto &bin : y_bins_)
    {
        std::cout << bin << " ";
    }
    std::cout << "\n  Size Bins: ";
    for (const auto &bin : size_bins_)
    {
        std::cout << bin << " ";
    }
    std::cout << "\n  Skew Bins: ";
    for (const auto &bin : skew_bins_)
    {
        std::cout << bin << " ";
    }
    std::cout << std::endl;
}