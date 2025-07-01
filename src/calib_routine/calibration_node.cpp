/**
 * @file calibration_node.cpp
 * @author Tina Tian
 * @brief Entry point for the unified calibration node.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_srvs/Trigger.h>
#include "calibration_routine_interface.hpp"
#include "calibration_config.hpp"
#include <camera_model/CalibrationControl.h>

using namespace std;

class CalibrationNode
{
public:
    CalibrationNode(ros::NodeHandle &nh, const CalibrationConfig &config)
        : nh_(nh), config_(config), phase_(CalibrationPhase::IDLE)
    {
        // Subscriber: incoming compressed images
        image_sub_ = nh_.subscribe("/ximea/image_profile/throttle/compressed",
                                   10,
                                   &CalibrationNode::imageCallback, this);

        // Service server: controls the state of calibration
        // options: 1 for proceed to phase two, 2 for cancel
        calib_control_service_ = nh_.advertiseService("/pipe_sprite/calibration/control", &CalibrationNode::controlCalibration, this);

        ROS_INFO("Calibration Node initialized.");
        if (!start())
        {
            ROS_WARN("Failed to start calibration manager.");
            requestShutdown();
        }
    }

    void requestShutdown()
    {
        shutdown_requested_ = true;
    }

    bool shutdownRequested() const { return shutdown_requested_; }

private:
    ros::NodeHandle &nh_;
    ros::Subscriber image_sub_;
    ros::ServiceServer calib_control_service_;

    CalibrationPhase phase_;
    std::unique_ptr<ICalibrationRoutine> routine_;
    const CalibrationConfig &config_;

    bool shutdown_requested_ = false;

    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        if (phase_ == CalibrationPhase::PHASE_ONE)
        {
            if (routine_)
            {
                routine_->handleImage(msg);
            }
        }
    }

    bool controlCalibration(camera_model::CalibrationControl::Request &req, camera_model::CalibrationControl::Response &res)
    {
        if (phase_ == CalibrationPhase::PHASE_ONE && req.action == static_cast<int>(CalibrationControlAction::PROCEED_TO_PHASE_TWO))
        {
            ROS_INFO("Proceeding to Phase Two.");
            res.success = true;
            res.message = "Calibration Phase Two started.";
            phase_ = CalibrationPhase::PHASE_TWO;
            // this creates a separate thread, and returns immediately
            routine_->beginPhaseTwo();
        }
        else if (req.action == static_cast<int>(CalibrationControlAction::CANCEL))
        {
            ROS_INFO("Calibration cancelled.");
            res.success = true;
            res.message = "Calibration cancelled.";
            phase_ = CalibrationPhase::IDLE;
            shutdown_requested_ = true;
        }
        else
        {
            res.success = false;
            res.message = "Failed to perform calibration control action";
        }
        return true;
    }

    bool start()
    {
        if (phase_ != CalibrationPhase::IDLE)
        {
            ROS_WARN("Calibration already in progress.");
            return false;
        }

        routine_ = CalibrationRoutineFactory::create(nh_, config_);
        if (!routine_)
        {
            ROS_ERROR("Failed to create calibration routine.");
            return false;
        }
        routine_->setOnFinishCallback([this]()
                                      {
            ROS_INFO("Calibration complete. Shutting down node.");
            this->requestShutdown(); });

        phase_ = CalibrationPhase::PHASE_ONE;
        ROS_INFO("Calibration Phase One started.");
        return true;
    }
};

CalibrationConfig loadConfigFromYaml(const std::string &filename)
{
    CalibrationConfig config;
    // Load from YAML file (not implemented here, just a placeholder)
    // Use cv::FileStorage or similar to read the config
    return config;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    string config_file = "calibration_config.yaml"; // Default config file
    CalibrationConfig config = loadConfigFromYaml(config_file);
    CalibrationNode node(nh, config);
    ros::Rate loop_rate(0.5);

    while (ros::ok())
    {
        ros::spinOnce();

        if (node.shutdownRequested())
        {
            ros::Duration(0.2).sleep(); // Let response flush
            break;
        }

        loop_rate.sleep();
    }
    nh.shutdown();
    return 0;
}
