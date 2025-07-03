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
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

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

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    std::string calib_type_str;
    std::string frontend_path;
    std::string data_save_folder;

    po::options_description desc("Allowed options.");
    desc.add_options()(
        "help,h", "Produce help message")(
        "calibration_type,t", po::value<std::string>(&calib_type_str)->default_value("intrinsics"), "Calibration type options: intrinsics, camera-laser")(
        "frontend_path", po::value<std::string>(&frontend_path)->default_value("~/catkin_ws/src/blaser_mapping/pipe_blaser_ros"), "Folder of the sensing frontend.")(
        "data_save_folder", po::value<std::string>(&data_save_folder)->default_value("~/calib_images"), "Calibration data and detection save folder.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        nh.shutdown();
        return 1;
    }

    CalibrationType calib_type = calibration_config::toCalibrationType(calib_type_str);

    if (!fs::exists(frontend_path) && !fs::is_directory(frontend_path))
    {
        std::cerr << "# ERROR: Cannot find frontend path." << frontend_path
                  << std::endl;
        nh.shutdown();
        return 1;
    }

    // create folder for saving calibration params
    fs::path result_output_folder_path(frontend_path);
    result_output_folder_path /= "calib_data";
    if (!fs::exists(result_output_folder_path))
    {
        fs::create_directories(result_output_folder_path);
    }

    fs::path config_file_path(frontend_path);
    config_file_path = config_file_path / "calibration" / "config" /
                       (calibration_config::toString(calib_type) + ".yaml");
    if (!fs::exists(config_file_path))
    {
        std::cerr << "# ERROR: config file doesn't exist: "
                  << config_file_path.string() << std::endl;
        nh.shutdown();
        return 1;
    }

    // load and assemble the calibration config
    CalibrationConfig config = calibration_config::loadConfigFromYaml(config_file_path.string());
    config.setFrontendPath(frontend_path);
    config.setResultOutputFolder(result_output_folder_path.string());
    if (config.save_data)
    {
        fs::path data_save_folder_path(data_save_folder);
        data_save_folder_path /= calibration_config::toString(calib_type);
        std::string routine_data_save_folder = data_save_folder_path.string();
        if (!fs::exists(routine_data_save_folder))
        {
            fs::create_directories(routine_data_save_folder);
            std::cout << "Calib data will be saved to: "
                      << routine_data_save_folder << std::endl;
        }
        config.setRoutineDataSaveFolder(routine_data_save_folder);
    }

    std::cout << config << std::endl;

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
