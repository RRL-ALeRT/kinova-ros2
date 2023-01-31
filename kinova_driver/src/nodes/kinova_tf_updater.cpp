/*
 * kinova_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/kinova_tf_updater.h>
#include <kinova_driver/kinova_ros_types.h>

namespace kinova
{

KinovaTFTree::KinovaTFTree(std::shared_ptr<rclcpp::Node> node_handle, std::string& kinova_robotType)
    : kinematics_(node_handle, kinova_robotType), node_handle(node_handle)
{
    std::string kinova_robotName = "";
    if (!node_handle->has_parameter("kinova_robotName"))
        node_handle->declare_parameter("kinova_robotName", kinova_robotName);
    node_handle->get_parameter("kinova_robotName", kinova_robotName);

    joint_angles_subscriber_ = node_handle->create_subscription<kinova_msgs::msg::JointAngles>(kinova_robotName+"_tf_updater/in/joint_angles", 1,
                                                     std::bind(&KinovaTFTree::jointAnglesMsgHandler, this, std::placeholders::_1));
    current_angles_.joint1 = 0;
    current_angles_.joint2 = 0;
    current_angles_.joint3 = 0;
    current_angles_.joint4 = 0;
    current_angles_.joint5 = 0;
    current_angles_.joint6 = 0;
    current_angles_.joint7 = 0;
    last_angle_update_ = node_handle->get_clock()->now();
    tf_update_timer_ = node_handle->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KinovaTFTree::tfUpdateHandler, this));
    flag_tf_update_timer_ = false;
}


void KinovaTFTree::jointAnglesMsgHandler(const kinova_msgs::msg::JointAngles::SharedPtr joint_angles)
{
    current_angles_.joint1 = joint_angles->joint1;
    current_angles_.joint2 = joint_angles->joint2;
    current_angles_.joint3 = joint_angles->joint3;
    current_angles_.joint4 = joint_angles->joint4;
    current_angles_.joint5 = joint_angles->joint5;
    current_angles_.joint6 = joint_angles->joint6;
    current_angles_.joint7 = joint_angles->joint7;
    last_angle_update_ = node_handle->get_clock()->now();
    flag_tf_update_timer_ = true;
}


void KinovaTFTree::calculatePostion(void)
{
    // Update the forward Kinematics
    float Q[7] = {kinematics_.degToRad(current_angles_.joint1),
                 kinematics_.degToRad(current_angles_.joint2),
                 kinematics_.degToRad(current_angles_.joint3),
                 kinematics_.degToRad(current_angles_.joint4),
                 kinematics_.degToRad(current_angles_.joint5),
                 kinematics_.degToRad(current_angles_.joint6),
                 kinematics_.degToRad(current_angles_.joint7)};

    kinematics_.updateForward(Q);
}


void KinovaTFTree::tfUpdateHandler()
{
    if (!flag_tf_update_timer_) return;
    
    this->calculatePostion();  // Update TF Tree

    if ((node_handle->get_clock()->now().seconds() - last_angle_update_.seconds()) > 1)
    {
        flag_tf_update_timer_ = false;
    }
}

}  // namespace kinova


int main(int argc, char **argv)
{
    /* Set up ROS */
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("kinova_tf_updater");

    std::string kinova_robotType = "";
    // Retrieve the (non-option) argument:
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
        return -1;
    }
    else // there is an input...
    {
        kinova_robotType = argv[argc-1];
        RCLCPP_INFO(nh->get_logger(), "kinova_robotType is %s.", kinova_robotType.c_str());
    }

    kinova::KinovaTFTree KinovaTF(nh, kinova_robotType);
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
