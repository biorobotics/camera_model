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

PoseCoverageMetrics evaluatePoseCoverage(
    const std::vector<Eigen::Vector3d> &translations,
    const std::vector<Eigen::Vector3d> &eulerAngles,
    const std::vector<std::vector<int>> &cornerHeatmap,
    int minCornersPerCell,
    double expectedX, double expectedY, double expectedZ,
    double expectedRoll, double expectedPitch, double expectedYaw)
{
    PoseCoverageMetrics result{};

    // Get min/max for translations
    Eigen::Vector3d t_min = translations[0];
    Eigen::Vector3d t_max = translations[0];
    for (const auto &t : translations)
    {
        t_min = t_min.cwiseMin(t);
        t_max = t_max.cwiseMax(t);
    }
    double rangeX = t_max[0] - t_min[0];
    double rangeY = t_max[1] - t_min[1];
    double rangeZ = t_max[2] - t_min[2];

    result.score_x = std::min(100.0, (rangeX / expectedX) * 100.0);
    result.score_y = std::min(100.0, (rangeY / expectedY) * 100.0);
    result.score_z = std::min(100.0, (rangeZ / expectedZ) * 100.0);

    // Get min/max for roll/pitch/yaw
    double roll_min = eulerAngles[0][0], roll_max = eulerAngles[0][0];
    double pitch_min = eulerAngles[0][1], pitch_max = eulerAngles[0][1];
    double yaw_min = eulerAngles[0][2], yaw_max = eulerAngles[0][2];

    for (const auto &rpy : eulerAngles)
    {
        roll_min = std::min(roll_min, rpy[0]);
        roll_max = std::max(roll_max, rpy[0]);
        pitch_min = std::min(pitch_min, rpy[1]);
        pitch_max = std::max(pitch_max, rpy[1]);
        yaw_min = std::min(yaw_min, rpy[2]);
        yaw_max = std::max(yaw_max, rpy[2]);
    }

    double roll_range = roll_max - roll_min;
    double pitch_range = pitch_max - pitch_min;
    double yaw_range = yaw_max - yaw_min;

    result.score_roll = std::min(100.0, (roll_range / expectedRoll) * 100.0);
    result.score_pitch = std::min(100.0, (pitch_range / expectedPitch) * 100.0);
    result.score_yaw = std::min(100.0, (yaw_range / expectedYaw) * 100.0);

    // Heatmap coverage
    int covered = 0;
    int total = 0;
    for (const auto &row : cornerHeatmap)
    {
        for (int count : row)
        {
            ++total;
            if (count >= minCornersPerCell)
                ++covered;
        }
    }
    result.image_coverage_score = (static_cast<double>(covered) / total) * 100.0;

    // Combine all into total score (weighted)
    result.total_score =
        0.10 * result.score_x +
        0.10 * result.score_y +
        0.10 * result.score_z +
        0.20 * result.score_roll +
        0.20 * result.score_pitch +
        0.20 * result.score_yaw +
        0.10 * result.image_coverage_score;

    return result;
}

PoseCoverageMetrics evaluatePoseCoverage(
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const std::vector<std::vector<cv::Point2f>> &allCorners,
    int imageWidth,
    int imageHeight,
    int gridRows,
    int gridCols)
{
    std::vector<Eigen::Vector3d> outTranslations;
    std::vector<Eigen::Vector3d> outEulerAngles;
    std::vector<std::vector<int>> outHeatmap;
    convertCalibrationDataForCoverageCheck(
        rvecs, tvecs, allCorners, imageWidth, imageHeight,
        gridRows, gridCols, outTranslations, outEulerAngles, outHeatmap);
    return evaluatePoseCoverage(
        outTranslations, outEulerAngles, outHeatmap);
}

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
    std::vector<std::vector<int>> &outHeatmap)
{
    outTranslations.clear();
    outEulerAngles.clear();
    outHeatmap.assign(gridRows, std::vector<int>(gridCols, 0));

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        // --- Convert rvec to Euler angles
        cv::Mat R_cv;
        cv::Rodrigues(rvecs[i], R_cv);
        Eigen::Matrix3d R;
        cv::cv2eigen(R_cv, R);

        // ZYX euler angles (yaw, pitch, roll) --> then reordered
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);    // yaw, pitch, roll
        euler = euler * 180.0 / M_PI;                      // degrees
        Eigen::Vector3d rpy(euler[2], euler[1], euler[0]); // roll, pitch, yaw

        // --- Convert tvec to Eigen
        Eigen::Vector3d t(
            tvecs[i].at<double>(0),
            tvecs[i].at<double>(1),
            tvecs[i].at<double>(2));

        outEulerAngles.push_back(rpy);
        outTranslations.push_back(t);

        // --- Fill heatmap
        for (const auto &pt : allCorners[i])
        {
            int grid_x = static_cast<int>((pt.x / imageWidth) * gridCols);
            int grid_y = static_cast<int>((pt.y / imageHeight) * gridRows);
            grid_x = clamp(grid_x, 0, gridCols - 1);
            grid_y = clamp(grid_y, 0, gridRows - 1);
            outHeatmap[grid_y][grid_x]++;
        }
    }
}