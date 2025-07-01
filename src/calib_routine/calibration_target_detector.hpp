/**
 * @file calibration_target_detector.hpp
 * @author Tina Tian
 * @brief A clean wrapper for Aprilgrid detector for camera calibration.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <memory>
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

class AprilgridDetector
{
public:
    /**
     * @brief Construct a new Aprilgrid Detector object.
     * Each image correspond to one detector object.
     *
     * @param image Input - BGR or gray image
     * @param tagRows Input - # of tags per row
     * @param tagCols Input - # of tags per column
     * @param tagSize Input - size of a tag in meters
     * @param tagSpacing Input - spacing between tags as a ratio of tag size
     */
    AprilgridDetector(cv::Mat &image, int tagRows, int tagCols,
                      double tagSize = 0.0275, double tagSpacing = 0.3);

    /**
     * @brief Detect Aprilgrid corners in the image.
     * Update the `corners_` and `grid3dPoints_` vectors with detected points.
     *
     * @param corners_scale_factor Scale factor for the corners.
     */
    bool detect(float corners_scale_factor = 1.0);

    /**
     * @brief Estimate the pose of the target wrt the camera using the detected corners and their corresponding 3D points.
     *
     * @param cameraPtr Input - Pointer to the camera model
     * @param rvec Output - Rotation vector (3x1)
     * @param tvec Output - Translation vector (3x1)
     * @return True if pose estimation is successful, false otherwise.
     */
    bool estimatePose(camera_model::CameraPtr cameraPtr, cv::Mat &rvec, cv::Mat &tvec);

    /**
     * @brief Draw detected corners on the image in place.
     *
     * @param corners_scale_factor Scale factor for the corners.
     */
    void drawCorners(cv::Mat &image, float corners_scale_factor = 1.0);

    std::vector<cv::Point2f> getCorners() const
    {
        return corners_;
    }

    std::vector<cv::Point3f> getGrid3dPoints() const
    {
        return grid3dPoints_;
    }

private:
    std::vector<cv::Point2f> corners_;
    std::vector<cv::Point3f> grid3dPoints_;

    // The gray and BGR images are stored in the target object
    std::shared_ptr<aslam::cameras::GridCalibrationTargetAprilgrid> target_;
};
