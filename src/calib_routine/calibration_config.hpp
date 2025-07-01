/**
 * @file calibration_config.hpp
 * @author Tina Tian
 * @brief Configuration, typedefs, and factory for camera calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

#include "calibration_routine_interface.hpp"
#include "camera_model/camera_models/Camera.h"

enum class CalibrationType
{
    INTRINSICS,
    CAMERA_LASER
};

enum class CalibrationPhase
{
    IDLE,
    PHASE_ONE,
    PHASE_TWO
};

enum class CalibrationControlAction
{
    PROCEED_TO_PHASE_TWO = 1,
    CANCEL = 2
};

/**
 * @brief Configuration structure for camera calibration routines.
 * Should be configured from a YAML file or similar source.
 */
struct CalibrationConfig
{
    // Calibration type (e.g., intrinsics or extrinsics)
    CalibrationType calibration_type = CalibrationType::INTRINSICS; // default type

    // Tag grid configuration
    int tag_rows = 6;
    int tag_cols = 6;
    double tag_size = 0.0275; // meters
    double tag_spacing = 0.3; // ratio: spacing / tag_size

    // Image input configuration
    int image_width = 720;   // default width
    int image_height = 1280; // default height

    // Camera model and name
    camera_model::Camera::ModelType camera_model = camera_model::Camera::PINHOLE; // default model
    std::string camera_name = "camera";

    // Image preprocessing
    float resize_scale_one = 0.5;
    float resize_scale_two = 1.0;
    cv::Size roi_size = {0, 0};    // optional
    cv::Point roi_center = {0, 0}; // optional

    // File paths
    // std::string storage_path = "/tmp/calib_data";
    std::string result_output_path = "/home/tina/Documents/test_ws/src/camera_model/test_data/aprilgrid/my_results";

    // Misc
    bool verbose = true;

    // Optional runtime utilities
    // std::string image_topic = "/camera/image/compressed";

    double max_error_threshold = 20.0; // px

    /**
     * @brief Get the camera model type based on the model name.
     * @param model_name The name of the camera model.
     * @param verbose Whether to print verbose information.
     * @return The camera model type.
     */
    camera_model::Camera::ModelType toCameraModelType(const std::string &model_name, bool verbose = false);

    /**
     * @brief Print camera model information based on the model type.
     * @param modelType The camera model type.
     */
    void printCameraModelInfo(const camera_model::Camera::ModelType &modelType);
};

/**
 * @brief Factory class to create calibration routines based on type.
 */
class CalibrationRoutineFactory
{
public:
    static std::unique_ptr<ICalibrationRoutine> create(ros::NodeHandle &nh, const CalibrationConfig &config);
};