//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
// #include "kinova_driver/kinova_tool_pose_action.h"
// #include "kinova_driver/kinova_joint_angles_action.h"
// #include "kinova_driver/kinova_fingers_action.h"
// #include "kinova_driver/kinova_joint_trajectory_controller.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("kinova_arm_driver");
    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    std::string kinova_robotType = "";
    std::string kinova_robotName = "";

    node->declare_parameter("kinova_robotType", "");
    node->declare_parameter("kinova_robotName", "");

    node->get_parameter("kinova_robotType", kinova_robotType);
    node->get_parameter("kinova_robotName", kinova_robotName);

    if (kinova_robotType == "")
        return -1;

    RCLCPP_INFO(node->get_logger(), "kinova_robotType is %s.", kinova_robotType.c_str());
    RCLCPP_INFO(node->get_logger(), "kinova_robotName is %s.", kinova_robotName.c_str());

    while (rclcpp::ok())
    {
        try
        {
            kinova::KinovaComm comm(node, api_mutex, is_first_init, kinova_robotType);
            kinova::KinovaArm kinova_arm(comm, node, kinova_robotType, kinova_robotName);
            // kinova::KinovaPoseActionServer pose_server(comm, nh, kinova_robotType, kinova_robotName);
            // kinova::KinovaAnglesActionServer angles_server(comm, nh);
            // kinova::KinovaFingersActionServer fingers_server(comm, nh);
            // kinova::JointTrajectoryController joint_trajectory_controller(comm, nh);
            rclcpp::spin(node);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
            kinova::KinovaAPI api;
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            api.closeAPI();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        is_first_init = false;
    }
    return 0;
}
