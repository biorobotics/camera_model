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
    INTRINSICS = 0,
    CAMERA_LASER = 1
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

    /// Tag grid configuration
    int tag_rows = 6;
    int tag_cols = 6;
    double tag_size = 0.0275; // meters
    double tag_spacing = 0.3; // ratio: spacing / tag_size

    /// Image input configuration
    int image_width = 720;   // default width
    int image_height = 1280; // default height

    /// Camera model and name
    camera_model::Camera::ModelType camera_model = camera_model::Camera::PINHOLE; // default model
    std::string camera_name = "camera";

    /// Image preprocessing
    float resize_scale_one = 0.5;
    float resize_scale_two = 1.0;
    cv::Size roi_size = {0, 0};    // optional
    cv::Point roi_center = {0, 0}; // optional

    /// File paths
    // default path to sensing frontend
    std::string frontend_path = "~/catkin_ws/src/blaser_mapping/pipe_blaser_ros";
    // default folder to store all output calib params yaml files
    std::string result_output_folder = "~/calib_yaml";
    std::string routine_data_save_folder = "~/data/intrinsics";

    double max_error_threshold = 20.0; // px

    /// Misc
    bool verbose = true;
    bool save_data = false; // whether to save intermediate data to file system

    /**
     * @brief Set the result output file path for calibration results.
     * @note This field doesn't exist in the yaml file, user need to set it manually.
     *
     * @param path The absolute path of the yaml file storing calib params.
     */
    void setResultOutputFolder(const std::string &path)
    {
        result_output_folder = path;
    }

    /**
     * @brief Set the sensing frontend folder.
     *
     * @param path The absolute path of the sensing frontend folder.
     * @note This field doesn't exist in the yaml file, user need to set it manually.
     */
    void setFrontendPath(const std::string &path)
    {
        frontend_path = path;
    }

    /**
     * @brief Set the folder where calibration data will be saved.
     * @note This field doesn't exist in the yaml file, user need to set it manually.
     *
     * @param folder The absolute path of the folder to save calibration data.
     */
    void setRoutineDataSaveFolder(const std::string &folder)
    {
        routine_data_save_folder = folder;
    }

    // print struct
    friend std::ostream &operator<<(std::ostream &os, const CalibrationConfig &config);
};

namespace calibration_config
{
    /**
     * @brief Convert a string to a CalibrationType enum.
     *
     * @param calib_type_str options: "intrinsics", "camera-laser"
     * @note Throws std::invalid_argument if the string does not match any known type.
     *
     */
    CalibrationType toCalibrationType(const std::string &calib_type_str);

    /**
     * @brief Convert a CalibrationType enum to a string representation.
     *
     * @param type
     */
    std::string toString(CalibrationType type);

    /**
     * @brief Load calibration configuration from a YAML file.
     *
     * @param filename Absolute path to the YAML configuration file.
     * @return CalibrationConfig The loaded configuration.
     */
    CalibrationConfig loadConfigFromYaml(const std::string &filename);
}

/**
 * @brief Factory class to create calibration routines based on type.
 */
class CalibrationRoutineFactory
{
public:
    static std::unique_ptr<ICalibrationRoutine> create(ros::NodeHandle &nh, const CalibrationConfig &config);
};