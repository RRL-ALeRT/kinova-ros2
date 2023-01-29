/*
 * kinova_tf_updater.h
 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#ifndef KINOVA_DRIVER_KINOVA_TF_UPDATER_H
#define KINOVA_DRIVER_KINOVA_TF_UPDATER_H

#include <time.h>

#include <kinova_driver/kinova_arm_kinematics.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "kinova_msgs/msg/joint_angles.hpp"


namespace kinova
{

class KinovaTFTree
{
 public:
    explicit KinovaTFTree(std::shared_ptr<rclcpp::Node> nh, std::string& kinova_robotType);

 private:
    void jointAnglesMsgHandler(const kinova_msgs::msg::JointAngles::SharedPtr joint_angles);
    void calculatePostion(void);
    void tfUpdateHandler();

    kinova::KinovaKinematics kinematics_;
    kinova_msgs::msg::JointAngles current_angles_;
    std::shared_ptr<rclcpp::Node> node_handle;
    rclcpp::Time last_angle_update_;
    rclcpp::Subscription<kinova_msgs::msg::JointAngles>::SharedPtr joint_angles_subscriber_;
    rclcpp::TimerBase::SharedPtr tf_update_timer_;
    bool flag_tf_update_timer_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_TF_UPDATER_H
