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
     * @param rescale_factor Input - Rescale factor for the corners and for the input image (this factor equals 1 / scale_factor during preprocessing)
     */
    AprilgridDetector(cv::Mat &image, int tagRows, int tagCols,
                      double tagSize = 0.0275, double tagSpacing = 0.3,
                      double rescale_factor = 1.0);

    /**
     * @brief Detect Aprilgrid corners in the image.
     * Update the `corners_` and `grid3dPoints_` vectors with detected points.
     */
    bool detect();

    /**
     * @brief Estimate the pose of the target wrt the camera using the detected corners and their corresponding 3D points.
     * @note This function assumes that the camera model already as a calibrated intrinsics. Else, the rvec and tvec will be nan.
     *
     * @param cameraPtr Input - Pointer to the camera model
     * @param rvec Output - Rotation vector (3x1)
     * @param tvec Output - Translation vector (3x1)
     * @return True if pose estimation is successful, false otherwise.
     */
    bool estimatePose(camera_model::CameraPtr cameraPtr, cv::Mat &rvec, cv::Mat &tvec);

    /**
     * @brief Compute the bounding box of the full tag, projecting the 3D grid points into the image plane.
     * If an outer corner point isn't detected, it's extrapolated.
     *
     * @param projected_corners Output - Order: TL(#0), TR(#3), BR(#15), BL(#12)
     * @param H Output - Homography matrix from the grid points to the image corners.
     * @return false if failed to compute the bounding box, true otherwise.
     *
     * @details corner ordering example :
     *        12-----13  14-----15
     *        | TAG 3 |  | TAG 4 |
     *        8-------9  10-----11
     *        4-------5  6-------7
     *  y     | TAG 1 |  | TAG 2 |
     *  ^     0-------1  2-------3
     *  |-->x
     */
    bool computeBoundingBox(
        std::vector<cv::Point2f> &projected_corners, cv::Mat &H) const;

    /**
     * @brief Calculate the skew of the tag seen in the image by computing the angle between the TL->TR and TR->BR sides.
     *
     * @param projected_corners Input. Ordering: see `computeBoundingBox()`.
     * @return Skew value in the range [0, 1]. 0 = no skew, 1 = max skew
     */
    double calculateSkewFromSideAngle(
        const std::vector<cv::Point2f> &projected_corners) const;

    /**
     * @brief Calculate the skew of the tag seen in the image from the homography matrix.
     *
     * @param H Input - Homography matrix from the grid points to the image corners.
     * @return Skew value in the range [0, 1]. 0 = no skew, 1 = max skew
     */
    double calculateSkewFromHomography(const cv::Mat &H) const;

    /**
     * @brief Calculate the size of the tag seen in the image.
     *
     * @param projected_corners Input. Ordering: see `computeBoundingBox()`.
     * @return Size of the tag in [0, 1]. 0: no tag, 1: same or bigger than the full image pixels
     */
    double calculateSize(
        const std::vector<cv::Point2f> &projected_corners) const;

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
    int image_width_ = 720;       // Default image width
    int image_height_ = 1280;     // Default image height
    double rescale_factor_ = 1.0; // Rescale factor for corners and image

    std::vector<cv::Point2f> corners_;
    std::vector<cv::Point3f> grid3dPoints_;
    std::vector<size_t> corner_ids_;

    // The gray and BGR images are stored in the target object
    std::shared_ptr<aslam::cameras::GridCalibrationTargetAprilgrid> target_;
};
