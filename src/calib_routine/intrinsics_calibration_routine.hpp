/**
 * @file intrinsics_calibration_routine.hpp
 *
 * @author Tina Tian
 * @brief Intrinsics calibration routine for camera models.
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "calibration_routine_interface.hpp"
#include "calibration_config.hpp"
#include "calib_utils.h"
#include "camera_model/code_utils/cv_utils.h"
#include "calibration_target_detector.hpp"
#include <camera_model/calib/CameraCalibration.h>
#include <vector>
#include <opencv2/opencv.hpp>

class IntrinsicsCalibrationRoutine : public ICalibrationRoutine
{
public:
    IntrinsicsCalibrationRoutine(ros::NodeHandle &nh,
                                 const CalibrationConfig &config);

    void handleImage(const sensor_msgs::CompressedImageConstPtr &msg) override;

    bool coverageGood(PoseCoverageMetrics &pose_coverage);

    void pubDofStatus(const PoseCoverageMetrics &pose_coverage);

    void beginPhaseTwo() override;
    void saveResults(const std::string &output_path) override;

    void setOnFinishCallback(std::function<void()> cb) override;

private:
    ros::NodeHandle &nh_;
    const CalibrationConfig &config_;
    camera_model::CameraCalibration calibration_;

    std::function<void()> on_finish_;

    std::atomic<bool> phase_one_done_{false};
    std::atomic<bool> phase_two_done_{false};
    int good_detections_ = 0;
    int required_detections_ = 15;

    cv_utils::fisheye::PreProcess *preprocess_one_;
    cv_utils::fisheye::PreProcess *preprocess_two_;

    std::vector<cv::Mat> stored_images_;
    std::vector<std::vector<cv::Point2f>> allCorners_;
    // std::vector<std::vector<cv::Point3f>> allObjPoints_;
    std::vector<cv::Mat> rvecs_;
    std::vector<cv::Mat> tvecs_;

    double last_add_time_ = 0;
    double add_interval_s_ = 1.0;

    ros::Publisher debug_pub_;
    ros::Publisher dof_status_pub_;
    ros::Publisher calib_progress_pub_;

    void performCalibration();
};
