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

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

struct PoseCoverageMetrics
{
    double score_x, score_y, score_z;
    double score_roll, score_pitch, score_yaw;
    double image_coverage_score;
    double total_score;

    /**
     * @brief Print the metrics to the console.
     */
    void print() const
    {
        std::cout << "Pose Coverage Metrics:\n"
                  << "  Score X: " << score_x << "\n"
                  << "  Score Y: " << score_y << "\n"
                  << "  Score Z: " << score_z << "\n"
                  << "  Score Roll: " << score_roll << "\n"
                  << "  Score Pitch: " << score_pitch << "\n"
                  << "  Score Yaw: " << score_yaw << "\n"
                  << "  Image Coverage Score: " << image_coverage_score << "\n"
                  << "  Total Score: " << total_score << "\n";
    }
    PoseCoverageMetrics()
        : score_x(0), score_y(0), score_z(0),
          score_roll(0), score_pitch(0), score_yaw(0),
          image_coverage_score(0), total_score(0) {}
};

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

PoseCoverageMetrics evaluatePoseCoverage(
    const std::vector<Eigen::Vector3d> &translations,   // tvecs
    const std::vector<Eigen::Vector3d> &eulerAngles,    // roll, pitch, yaw in degrees
    const std::vector<std::vector<int>> &cornerHeatmap, // grid[rows][cols]
    int minCornersPerCell = 1,
    double expectedX = 0.5, double expectedY = 0.5, double expectedZ = 0.5,
    double expectedRoll = 90.0, double expectedPitch = 90.0, double expectedYaw = 120.0);

/**
 * @brief Can directly evaluate pose coverage from rvecs, tvecs, and allCorners.
 *
 * @param rvecs
 * @param tvecs
 * @param allCorners
 * @param imageWidth
 * @param imageHeight
 * @param gridRows
 * @param gridCols
 * @return
 */
PoseCoverageMetrics evaluatePoseCoverage(
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const std::vector<std::vector<cv::Point2f>> &allCorners,
    int imageWidth,
    int imageHeight,
    int gridRows = 10,
    int gridCols = 10);

void convertCalibrationDataForCoverageCheck(
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const std::vector<std::vector<cv::Point2f>> &allCorners,
    int imageWidth,
    int imageHeight,
    int gridRows,
    int gridCols,
    std::vector<Eigen::Vector3d> &outTranslations,
    std::vector<Eigen::Vector3d> &outEulerAngles,
    std::vector<std::vector<int>> &outHeatmap);