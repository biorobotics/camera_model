/**
 * @file calibration_target_detector.cpp
 * @author Tina Tian
 * @brief A clean wrapper for Aprilgrid detector for camera calibration.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "calibration_target_detector.hpp"
#include "calib_utils.h"
#include <tuple>

AprilgridDetector::AprilgridDetector(
    cv::Mat &image, int tagRows, int tagCols,
    double tagSize, double tagSpacing,
    double rescale_factor)
    : rescale_factor_(rescale_factor)
{
    target_ = std::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(image, tagRows, tagCols, tagSize, tagSpacing);
    image_width_ = static_cast<int>(image.cols * rescale_factor_);
    image_height_ = static_cast<int>(image.rows * rescale_factor_);
}

bool AprilgridDetector::detect()
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
            if (rescale_factor_ != 1.0 || rescale_factor_ > 0.0)
            {
                corner_scaled.x *= rescale_factor_;
                corner_scaled.y *= rescale_factor_;
            }
            corners_.push_back(corner_scaled);
            grid3dPoints_.push_back(points3ds.at(idx_p));
            corner_ids_.push_back(idx_p);
        }
    }
    return true;
}

bool AprilgridDetector::estimatePose(camera_model::CameraPtr cameraPtr, cv::Mat &rvec, cv::Mat &tvec)
{
    // check if corners_ and grid3dPoints_ have nan points
    for (const auto &corner : corners_)
    {
        if (std::isnan(corner.x) || std::isnan(corner.y))
        {
            std::cerr << "Detected corner has NaN values: " << corner << std::endl;
            return false;
        }
    }
    if (corners_.empty() || grid3dPoints_.empty())
        return false;

    cameraPtr->estimateExtrinsics(grid3dPoints_, corners_, rvec, tvec);
    if (rvec.empty() || tvec.empty() || std::isnan(rvec.at<double>(0)) || std::isnan(tvec.at<double>(0)))
    {
        std::cerr << "Pose estimation failed: rvec or tvec empty or contains NaN values." << std::endl;
        return false;
    }
    return true;
}

///@todo
bool AprilgridDetector::computeBoundingBox(
    std::vector<cv::Point2f> &projected_corners, cv::Mat &H) const
{
    if (corners_.size() < 4)
        return false; // Not enough points

    // grid_pts: (x, y) of grid
    // image_pts: (x, y) of corner in image
    std::vector<cv::Point2f> grid_pts;
    for (const auto &corner_id : corner_ids_)
    {
        Eigen::Vector3d grid_pt_3d = target_->point(corner_id);
        cv::Point2f grid_pt(
            static_cast<float>(grid_pt_3d.x()),
            static_cast<float>(grid_pt_3d.y()));
        grid_pts.push_back(grid_pt);
    }

    H = cv::findHomography(grid_pts, corners_, cv::RANSAC);
    if (H.empty())
        return false;

    // Define 4 corners of the full grid
    std::vector<std::tuple<size_t, size_t>> bounding_corner_coords = {
        {0, 0},
        {0, target_->cols() - 1},
        {target_->rows() - 1, target_->cols() - 1},
        {target_->rows() - 1, 0}};
    std::vector<cv::Point2f> bounding_corners;
    for (const auto &corner_coord : bounding_corner_coords)
    {
        size_t r = std::get<0>(corner_coord);
        size_t c = std::get<1>(corner_coord);
        Eigen::Vector3d bounding_corner = target_->gridPoint(r, c);
        // Use only x, y coordinates
        cv::Point2f bounding_corner_2f(
            static_cast<float>(bounding_corner.x()),
            static_cast<float>(bounding_corner.y()));
        bounding_corners.push_back(bounding_corner_2f);
    }

    cv::perspectiveTransform(bounding_corners, projected_corners, H);
    return true;
}

double AprilgridDetector::calculateSkewFromSideAngle(
    const std::vector<cv::Point2f> &projected_corners) const
{
    if (projected_corners.size() < 4)
        return 0.0; // Not enough corners

    const cv::Point2f &up_left = projected_corners[0];
    const cv::Point2f &up_right = projected_corners[1];
    const cv::Point2f &down_right = projected_corners[2];

    auto angle = [](const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c) -> double
    {
        cv::Point2f ab = a - b;
        cv::Point2f cb = c - b;
        double dot = ab.dot(cb);
        double norm_ab = cv::norm(ab);
        double norm_cb = cv::norm(cb);
        double cos_angle = dot / (norm_ab * norm_cb);
        cos_angle = clamp(cos_angle, -1.0, 1.0); // Ensure safe acos
        return std::acos(cos_angle);
    };

    double angle_radians = angle(up_left, up_right, down_right);
    double skew = std::min(1.0, 2.0 * std::abs((M_PI / 2.0) - angle_radians));

    return skew;
}

double AprilgridDetector::calculateSkewFromHomography(const cv::Mat &H) const
{
    if (H.empty() || H.rows != 3 || H.cols != 3)
        return 0.0;

    // Extract the linear part
    cv::Vec2d v1(H.at<double>(0, 0), H.at<double>(1, 0)); // First column
    cv::Vec2d v2(H.at<double>(0, 1), H.at<double>(1, 1)); // Second column

    double dot_product = v1.dot(v2);
    double norm_v1 = cv::norm(v1);
    double norm_v2 = cv::norm(v2);

    if (norm_v1 < 1e-6 || norm_v2 < 1e-6)
        return 0.0;

    double cos_angle = clamp(dot_product / (norm_v1 * norm_v2), -1.0, 1.0);
    double angle_rad = std::acos(cos_angle);

    // deviation from orthogonal
    double angle_error = std::abs(M_PI / 2.0 - angle_rad);
    // Normalize: max allowed deviation is 60 deg
    double max_allowed = M_PI / 3.0;
    // maps 0 -> 1 over [0, max_allowed]
    double skew = std::min(1.0, angle_error / max_allowed);

    return skew;
}

double AprilgridDetector::calculateSize(
    const std::vector<cv::Point2f> &projected_corners) const
{
    if (projected_corners.size() != 4)
        return 0.0;

    const cv::Point2f &up_left = projected_corners[0];
    const cv::Point2f &up_right = projected_corners[1];
    const cv::Point2f &down_right = projected_corners[2];
    const cv::Point2f &down_left = projected_corners[3];

    cv::Point2f a = up_right - up_left;
    cv::Point2f b = down_right - up_right;
    cv::Point2f c = down_left - down_right;

    cv::Point2f p = b + c;
    cv::Point2f q = a + b;

    // Cross product magnitude for area of parallelogram formed by p and q
    // this area has unit: pix^2
    double area = std::abs(p.x * q.y - p.y * q.x) / 2.0;

    // area over the image total #pixels
    return std::min(1.0, area / (image_width_ * image_height_));
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
