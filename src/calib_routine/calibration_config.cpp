/**
 * @file calibration_config.cpp
 * @author Tina Tian
 * @brief Calibration configuration and routine factory implementation.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <boost/algorithm/string.hpp>
#include "calibration_config.hpp"
#include "intrinsics_calibration_routine.hpp"
// #include "camera_laser_calibration_routine.hpp" // Future

std::unique_ptr<ICalibrationRoutine> CalibrationRoutineFactory::create(
    ros::NodeHandle &nh, const CalibrationConfig &config)
{
    switch (config.calibration_type)
    {
    case CalibrationType::INTRINSICS:
        return std::make_unique<IntrinsicsCalibrationRoutine>(nh, config);

    case CalibrationType::CAMERA_LASER:
        // Placeholder for future implementation
        // return std::make_unique<CameraLaserCalibrationRoutine>(config);
        throw std::runtime_error("Camera-laser calibration not yet implemented");

    default:
        throw std::invalid_argument("Unknown calibration type");
    }
}

camera_model::Camera::ModelType CalibrationConfig::toCameraModelType(const std::string &model_name, bool verbose)
{
    camera_model::Camera::ModelType modelType = camera_model::Camera::PINHOLE;
    if (boost::iequals(model_name, "kannala-brandt"))
    {
        modelType = camera_model::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(model_name, "mei"))
    {
        modelType = camera_model::Camera::MEI;
    }
    else if (boost::iequals(model_name, "pinhole"))
    {
        modelType = camera_model::Camera::PINHOLE;
    }
    else if (boost::iequals(model_name, "pinhole2"))
    {
        modelType = camera_model::Camera::PINHOLE_FULL;
    }
    else if (boost::iequals(model_name, "scaramuzza"))
    {
        modelType = camera_model::Camera::SCARAMUZZA;
    }
    else if (boost::iequals(model_name, "myfisheye"))
    {
        modelType = camera_model::Camera::POLYFISHEYE;
    }
    else if (boost::iequals(model_name, "spline"))
    {
        modelType = camera_model::Camera::SPLINE;
    }
    else if (boost::iequals(model_name, "fov"))
    {
        modelType = camera_model::Camera::FOV;
    }
    else
    {
        throw std::invalid_argument("Unknown camera model type");
    }
    if (verbose)
    {
        printCameraModelInfo(modelType);
    }
    return modelType;
}

void CalibrationConfig::printCameraModelInfo(
    const camera_model::Camera::ModelType &modelType)
{
    switch (modelType)
    {
    case camera_model::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camera_model::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camera_model::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camera_model::Camera::PINHOLE_FULL:
        std::cout << "# INFO: Camera model: Full Pinhole Model" << std::endl;
        break;
    case camera_model::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    case camera_model::Camera::SPLINE:
        std::cout << "# INFO: Camera model: spline camera model" << std::endl;
        break;
    case camera_model::Camera::FOV:
        std::cout << "# INFO: Camera model: FOV camera model" << std::endl;
        break;
    case camera_model::Camera::POLYFISHEYE:
        std::cout << "# INFO: Camera model: GaoWenliang's polynomial fisheye model" << std::endl;
        break;
    default:
        std::cout << "# INFO: Camera model: Unknown" << std::endl;
    }
}