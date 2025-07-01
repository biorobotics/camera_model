/**
 * @file calibration_target_detector.cpp
 * @author Tina Tian
 * @brief A clean wrapper for Aprilgrid detector for camera calibration.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "calibration_target_detector.hpp"

AprilgridDetector::AprilgridDetector(cv::Mat &image, int tagRows, int tagCols, double tagSize, double tagSpacing)
{
    target_ = std::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(image, tagRows, tagCols, tagSize, tagSpacing);
}

bool AprilgridDetector::detect(float corners_scale_factor)
{
    std::vector<cv::Point2f> points2ds;
    std::vector<cv::Point3f> points3ds;
    std::vector<bool> outCornerObserved;
    if (!target_->computeObservation(points2ds, outCornerObserved, false))
        return false;

    points3ds = target_->points3d();

    for (int idx_p = 0; idx_p < outCornerObserved.size(); idx_p++)
    {
        if (outCornerObserved.at(idx_p))
        {
            cv::Point2f corner_scaled = points2ds.at(idx_p);
            if (corners_scale_factor != 1.0 || corners_scale_factor > 0.0)
            {
                corner_scaled.x *= corners_scale_factor;
                corner_scaled.y *= corners_scale_factor;
            }
            corners_.push_back(corner_scaled);
            grid3dPoints_.push_back(points3ds.at(idx_p));
        }
    }
    return true;
}

bool AprilgridDetector::estimatePose(camera_model::CameraPtr cameraPtr, cv::Mat &rvec, cv::Mat &tvec)
{
    if (corners_.empty() || grid3dPoints_.empty())
        return false;

    cameraPtr->estimateExtrinsics(grid3dPoints_, corners_, rvec, tvec);
    if (rvec.empty() || tvec.empty())
    {
        return false;
    }
    return true;
}

void AprilgridDetector::drawCorners(
    cv::Mat &image, float corners_scale_factor)
{
    std::vector<cv::Point2f> corners;
    if (corners_scale_factor != 1.0 || corners_scale_factor > 0.0)
    {
        for (auto &corner : corners_)
        {
            cv::Point2f corner_scaled = corner;
            corner_scaled.x *= corners_scale_factor;
            corner_scaled.y *= corners_scale_factor;
            corners.push_back(corner_scaled);
        }
    }
    else
    {
        corners = corners_;
    }
    for (const auto &corner : corners)
    {
        cv::circle(image, corner, 5, cv::Scalar(0, 255, 0), -1);
    }
}
