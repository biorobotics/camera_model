/**
 * @file calibration_routine_interface.hpp
 * @author Tina Tian
 * @brief Interface for calibration routines.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <ros/ros.h>
#include <string>

class ICalibrationRoutine
{
public:
    virtual ~ICalibrationRoutine() = default;

    virtual void handleImage(const sensor_msgs::CompressedImageConstPtr &msg) = 0;

    virtual void beginPhaseTwo() = 0;
    virtual void saveResults(const std::string &output_path) = 0;

    virtual void setOnFinishCallback(std::function<void()> cb) = 0;
};
