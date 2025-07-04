/**
 * @file calib_utils.h
 * @author Tina Tian
 * @brief Utility functions for camera calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <tuple>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

/**
 * @brief Clamp a value between a minimum and maximum.
 *
 * @tparam T
 * @param val
 * @param min_val
 * @param max_val
 * @return
 */
template <typename T>
T clamp(T val, T min_val, T max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

class CoverageChecker
{
public:
    struct CoverageMetrics
    {
        double score_x, score_y;
        double score_size, score_skew;

        /**
         * @brief Print the metrics to the console.
         */
        void print() const
        {
            std::cout << "Coverage Metrics:\n"
                      << "  Score X: " << score_x << "\n"
                      << "  Score Y: " << score_y << "\n"
                      << "  Score SIZE: " << score_size << "\n"
                      << "  Score SKEW: " << score_skew << std::endl;
        }
        CoverageMetrics()
            : score_x(0), score_y(0),
              score_size(0), score_skew(0)
        {
        }
    };

    CoverageChecker() = default;

    /**
     * @brief Construct a new Coverage Checker object. Initialize bins for coverage metrics.
     *
     */
    CoverageChecker(size_t image_width, size_t image_height,
                    size_t num_corner_bins = 10,
                    size_t num_size_bins = 5,
                    size_t num_skew_bins = 5,
                    size_t min_corners_per_cell = 1,
                    std::tuple<double, double> x_range = {0.0, 1.0},
                    std::tuple<double, double> y_range = {0.0, 1.0},
                    std::tuple<double, double> size_range = {0.0, 1.0},
                    std::tuple<double, double> skew_range = {0.0, 1.0});

    /**
     * @brief Add an observation of camera pose and detected corners.
     *
     * @param corners Detected corners in the image.
     * @param size Size of the tag in the image, normalized to [0, 1].
     * @param skew Skew of the tag in the image, normalized to [0, 1].
     */
    void addObservation(const std::vector<cv::Point2f> &corners,
                        double size, double skew);

    /**
     * @brief Evaluate the coverage metrics based on the stored observations.
     */
    CoverageMetrics evaluateCoverage();

    void printBins();

private:
    size_t image_width_ = 1280;
    size_t image_height_ = 720;

    size_t num_corner_bins_ = 10;
    size_t num_size_bins_ = 5;
    size_t num_skew_bins_ = 5;

    size_t min_corners_per_cell_ = 5;

    std::tuple<double, double> x_range_ = {0.0, 1.0};
    std::tuple<double, double> y_range_ = {0.0, 1.0};
    std::tuple<double, double> size_range_ = {0.0, 1.0};
    std::tuple<double, double> skew_range_ = {0.0, 1.0};

    std::vector<int> x_bins_, y_bins_;
    std::vector<int> size_bins_, skew_bins_;

    void addToBin(
        std::vector<int> &bins,
        double value, double min_val, double max_val);

    void addCornersToBin(const std::vector<cv::Point2f> &corners);
};
