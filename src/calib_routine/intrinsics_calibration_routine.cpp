/**
 * @file intrinsics_calibration_routine.cpp
 * @author Tina Tian
 * @brief Implementation of the intrinsics calibration routine.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "intrinsics_calibration_routine.hpp"
#include "calibration_target_detector.hpp"
#include "camera_model/camera_models/Camera.h"
#include <std_msgs/Int32MultiArray.h>
#include "camera_model/CalibrationProgress.h"
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>

IntrinsicsCalibrationRoutine::IntrinsicsCalibrationRoutine(
    ros::NodeHandle &nh,
    const CalibrationConfig &config)
    : nh_(nh), config_(config),
      calibration_(
          config.camera_model, config.camera_name,
          cv::Size(config.image_width, config.image_height),
          cv::Size(config.tag_cols, config.tag_rows),
          config.tag_size)
{
    // Publisher: Debug image stream (e.g. detections)
    debug_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/pipe_sprite/calibration/intrinsics/detection_result/compressed", 1);

    calib_progress_pub_ = nh_.advertise<camera_model::CalibrationProgress>("/pipe_sprite/calibration/intrinsics/progress", 1);

    dof_status_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/pipe_sprite/calibration/intrinsics/dof_status", 1);

    preprocess_one_ = new cv_utils::fisheye::PreProcess(cv::Size(config.image_width, config.image_height), config.roi_size, config.roi_center, config.resize_scale_one);

    preprocess_two_ = new cv_utils::fisheye::PreProcess(cv::Size(config.image_width, config.image_height), config.roi_size, config.roi_center, config.resize_scale_two);
}

void IntrinsicsCalibrationRoutine::handleImage(const sensor_msgs::CompressedImageConstPtr &msg)
{
    if (phase_one_done_.load())
    {
        // passthrough
        debug_pub_.publish(msg);
        return; // do nothing with image in phase two
    }

    try
    {
        cv::Mat image_src = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        cv::Mat image = preprocess_one_->do_preprocess(image_src);

        AprilgridDetector detector(image, config_.tag_rows, config_.tag_cols, config_.tag_size, config_.tag_spacing);

        if (detector.detect(1 / config_.resize_scale_one))
        {
            camera_model::CameraPtr cameraPtr = calibration_.camera();
            cv::Mat rvec, tvec;
            bool estimate_pose_ok = detector.estimatePose(cameraPtr, rvec, tvec);
            cv::Mat debug_image = image_src.clone();
            if (estimate_pose_ok)
            {
                // draw detected corners on a clone of the original image
                // note: stored corners are already scaled during detect()
                detector.drawCorners(debug_image);
            }
            std_msgs::Header header = msg->header;
            cv_bridge::CvImage bridge_image(
                header, sensor_msgs::image_encodings::BGR8, debug_image);
            sensor_msgs::CompressedImage compressed;
            bridge_image.toCompressedImageMsg(compressed);
            debug_pub_.publish(compressed);
            if (!estimate_pose_ok)
            {
                ROS_WARN("Pose estimation failed for image");
                return; // skip this image
            }
            // if some time has passed after last add to good detection list, add to list the image_src
            if (last_add_time_ == 0 ||
                ros::Time::now().toSec() - last_add_time_ > add_interval_s_)
            {
                stored_images_.push_back(image_src);

                allCorners_.push_back(detector.getCorners());
                rvecs_.push_back(rvec);
                tvecs_.push_back(tvec);
                PoseCoverageMetrics pose_coverage = evaluatePoseCoverage(rvecs_, tvecs_, allCorners_, config_.image_width, config_.image_height);
                pose_coverage.print();
                pubDofStatus(pose_coverage);

                last_add_time_ = ros::Time::now().toSec();
                good_detections_++;

                // @todo: publish coverage metric

                // check dof coverage, if good, call phase one done
                // no. it's up to the user when to end phase one
                if (stored_images_.size() >= required_detections_ && coverageGood(pose_coverage))
                {
                    phase_one_done_.store(true);
                }
            }
        }
        else
        {
            // passthrough
            debug_pub_.publish(msg);
        }
    }
    catch (cv::Exception &e)
    {
        ROS_WARN("Image decode or detection failed: %s", e.what());
    }
}

void IntrinsicsCalibrationRoutine::pubDofStatus(const PoseCoverageMetrics &pose_coverage)
{
    std_msgs::Int32MultiArray dof_status_msg;
    dof_status_msg.data.clear();
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_x));
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_y));
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_z));
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_roll));
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_pitch));
    dof_status_msg.data.push_back(static_cast<int>(pose_coverage.score_yaw));
    dof_status_pub_.publish(dof_status_msg);
}

bool IntrinsicsCalibrationRoutine::coverageGood(PoseCoverageMetrics &pose_coverage)
{
    /// @bug the scores are 100 at the very beginning. check math
    double score_x = pose_coverage.score_x;
    double score_y = pose_coverage.score_y;
    double score_z = pose_coverage.score_z;
    double score_roll = pose_coverage.score_roll;
    double score_pitch = pose_coverage.score_pitch;
    double score_yaw = pose_coverage.score_yaw;
    double image_coverage_score = pose_coverage.image_coverage_score;
    // Check if all scores are above a threshold
    double thresh = 80.0;
    if (score_x >= thresh && score_y >= thresh && score_z >= thresh && score_roll >= thresh && score_pitch >= thresh && score_yaw >= thresh && image_coverage_score >= thresh)
        return true;
    return false;
}

void IntrinsicsCalibrationRoutine::beginPhaseTwo()
{
    phase_one_done_.store(true); // mark phase one as done, to enforce the image callback to stop even when the coverage condition is not met
    std::thread([this]()
                { this->performCalibration(); })
        .detach();
}

void IntrinsicsCalibrationRoutine::saveResults(const std::string &output_path)
{
    ROS_INFO("Calibration results saved to: %s", output_path.c_str());
    calibration_.writeParams(output_path);
}

void IntrinsicsCalibrationRoutine::setOnFinishCallback(std::function<void()> cb)
{
    on_finish_ = std::move(cb);
}

void IntrinsicsCalibrationRoutine::performCalibration()
{
    camera_model::CalibrationProgress progress_msg;
    progress_msg.done = false;
    progress_msg.success = false;

    if (stored_images_.empty())
    {
        progress_msg.done = true;
        progress_msg.success = false;
        progress_msg.message = "Error: no images stored for calibration.";
        calib_progress_pub_.publish(progress_msg);
        ROS_ERROR("No images stored for calibration.");
        if (on_finish_)
        {
            on_finish_();
        }
        return;
    }

    // redo preprocess and corner detection on stored images
    size_t image_count = stored_images_.size();
    size_t processed_count = 0;
    int32_t max_process_progress = 70; // 70% of the progress bar for detection, the rest for calibration
    for (auto &image : stored_images_) // original size image
    {
        image = preprocess_two_->do_preprocess(image);
        AprilgridDetector detector(image, config_.tag_rows, config_.tag_cols, config_.tag_size, config_.tag_spacing);
        if (detector.detect())
        {
            calibration_.addChessboardData(detector.getCorners(), detector.getGrid3dPoints());
        }
        processed_count++;

        progress_msg.done = false;
        progress_msg.success = false;
        progress_msg.progress_percentage = clamp(static_cast<int32_t>(max_process_progress * static_cast<float>(processed_count) / image_count), 0, max_process_progress);
        progress_msg.message = "Processing images for calibration...";
        calib_progress_pub_.publish(progress_msg);
    }
    if (calibration_.sampleCount() < required_detections_)
    {
        progress_msg.done = true;
        progress_msg.success = false;
        progress_msg.message = "Error: Insufficient number of detected tags for calibration.";
        calib_progress_pub_.publish(progress_msg);
        ROS_ERROR("Insufficient number of detected tags for calibration.");
        if (on_finish_)
        {
            on_finish_();
        }
        return;
    }
    // one message before calibration calculation starts
    progress_msg.done = false;
    progress_msg.success = false;
    progress_msg.progress_percentage = max_process_progress;
    progress_msg.message = "Calibrating...";
    calib_progress_pub_.publish(progress_msg);

    calibration_.calibrate();

    // final message after calibration calculation, success depends on the error mean
    progress_msg.done = true;
    progress_msg.progress_percentage = 100;
    progress_msg.error_metrics.push_back("Mean Reprojection Error");
    progress_msg.error_values.push_back(calibration_.getErrorMean());

    if (calibration_.getErrorMean() > config_.max_error_threshold)
    {
        ROS_WARN("Calibration error mean is too high: %f", calibration_.getErrorMean());

        progress_msg.success = false;
        progress_msg.message = "Intrinsics calibration failed due to high error.";
        calib_progress_pub_.publish(progress_msg);
        if (on_finish_)
        {
            on_finish_();
        }
        return;
    }
    // write to local (base station), also send to robot
    /// @todo load previous config, and replace just one section
    saveResults(config_.result_fname);

    ROS_INFO("Intrinsics calibration complete.");
    progress_msg.success = true;
    progress_msg.message = "Intrinsics calibration complete.";
    calib_progress_pub_.publish(progress_msg);
    if (on_finish_)
    {
        on_finish_();
    }
}
