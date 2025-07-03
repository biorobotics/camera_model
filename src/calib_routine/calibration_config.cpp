/**
 * @file calibration_config.cpp
 * @author Tina Tian
 * @brief Calibration configuration and routine factory implementation.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <yaml-cpp/yaml.h>
#include "calibration_config.hpp"
#include "intrinsics_calibration_routine.hpp"
// #include "camera_laser_calibration_routine.hpp" // Future

std::unique_ptr<ICalibrationRoutine> CalibrationRoutineFactory::create(
    ros::NodeHandle &nh, const CalibrationConfig &config)
{
    switch (config.calibration_type)
    {
    case CalibrationType::INTRINSICS:
        std::cout << "Using intrinsics calibration." << std::endl;
        return std::make_unique<IntrinsicsCalibrationRoutine>(nh, config);

    case CalibrationType::CAMERA_LASER:
        std::cout << "Using camera-laser calibration." << std::endl;
        // Placeholder for future implementation
        // return std::make_unique<CameraLaserCalibrationRoutine>(config);
        throw std::runtime_error("Camera-laser calibration not yet implemented");

    default:
        throw std::invalid_argument("Unknown calibration type");
    }
}

CalibrationType calibration_config::toCalibrationType(const std::string &calib_type_str)
{
    if (calib_type_str == "intrinsics")
    {
        return CalibrationType::INTRINSICS;
    }
    else if (calib_type_str == "camera-laser")
    {
        return CalibrationType::CAMERA_LASER;
    }
    else
    {
        throw std::invalid_argument("Invalid CalibrationType: " + calib_type_str);
    }
}

// map calib type to string
std::string calibration_config::toString(CalibrationType type)
{
    switch (type)
    {
    case CalibrationType::INTRINSICS:
        return "intrinsics";
    case CalibrationType::CAMERA_LASER:
        return "camera-laser";
    default:
        throw std::invalid_argument("Unknown CalibrationType");
    }
}

CalibrationConfig calibration_config::loadConfigFromYaml(
    const std::string &filename)
{
    CalibrationConfig config;

    try
    {
        YAML::Node root = YAML::LoadFile(filename);

        if (root["calibration_type"])
        {
            std::string type_str = root["calibration_type"].as<std::string>();
            config.calibration_type = calibration_config::toCalibrationType(type_str);
        }

        config.tag_rows = root["tag_rows"].as<int>(config.tag_rows);
        config.tag_cols = root["tag_cols"].as<int>(config.tag_cols);
        config.tag_size = root["tag_size"].as<double>(config.tag_size);
        config.tag_spacing = root["tag_spacing"].as<double>(config.tag_spacing);

        config.image_width = root["image_width"].as<int>(config.image_width);
        config.image_height = root["image_height"].as<int>(config.image_height);

        if (root["camera_model"])
        {
            std::string model_str = root["camera_model"].as<std::string>();
            config.camera_model = camera_model::toCameraModelType(model_str);
        }

        config.camera_name = root["camera_name"].as<std::string>(config.camera_name);

        config.resize_scale_one = root["resize_scale_one"].as<float>(config.resize_scale_one);
        config.resize_scale_two = root["resize_scale_two"].as<float>(config.resize_scale_two);

        if (root["roi_size"])
        {
            config.roi_size = cv::Size(
                root["roi_size"]["width"].as<int>(),
                root["roi_size"]["height"].as<int>());
        }

        if (root["roi_center"])
        {
            config.roi_center = cv::Point(
                root["roi_center"]["x"].as<int>(),
                root["roi_center"]["y"].as<int>());
        }

        config.max_error_threshold = root["max_error_threshold"].as<double>(config.max_error_threshold);
        config.verbose = root["verbose"].as<bool>(config.verbose);
        config.save_data = root["save_data"].as<bool>(config.save_data);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading calibration config from YAML: " << e.what() << std::endl;
    }

    return config;
}

std::ostream &operator<<(std::ostream &os, const CalibrationConfig &config)
{
    os << "CalibrationConfig:\n"
       << "  calibration_type: " << calibration_config::toString(config.calibration_type) << "\n"
       << "  tag_rows: " << config.tag_rows << "\n"
       << "  tag_cols: " << config.tag_cols << "\n"
       << "  tag_size: " << config.tag_size << "\n"
       << "  tag_spacing: " << config.tag_spacing << "\n"
       << "  image_width: " << config.image_width << "\n"
       << "  image_height: " << config.image_height << "\n"
       << "  camera_model: " << camera_model::modelTypeToString(config.camera_model) << "\n"
       << "  camera_name: " << config.camera_name << "\n"
       << "  resize_scale_one: " << config.resize_scale_one << "\n"
       << "  resize_scale_two: " << config.resize_scale_two << "\n"
       << "  roi_size: {" << config.roi_size.width << ", " << config.roi_size.height << "}\n"
       << "  roi_center: {" << config.roi_center.x << ", "
       << config.roi_center.y << "}\n"
       << "  frontend_path: " << config.frontend_path << "\n"
       << "  result_output_folder: " << config.result_output_folder << "\n"
       << "  routine_data_save_folder: "
       << config.routine_data_save_folder
       << "\n"
       << "  max_error_threshold: "
       << config.max_error_threshold
       << "\n"
       << "  verbose: "
       << config.verbose
       << "\n"
       << "  save_data: "
       << config.save_data
       << "\n"
       << std::endl;

    return os;
}