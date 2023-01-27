/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_tool_pose_action.h
 *  Desc: Action server for kinova arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_POSE_ACTION_H_s
#define KINOVA_DRIVER_KINOVA_POSE_ACTION_H_s

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "kinova_msgs/action/arm_pose.hpp"

#include <string>
#include "kinova_driver/kinova_comm.h"


namespace kinova
{
using ArmPose = kinova_msgs::action::ArmPose;
using GoalHandleArmPose = rclcpp_action::ServerGoalHandle<ArmPose>;

class KinovaPoseActionServer
{
 public:
    KinovaPoseActionServer(KinovaComm &, const std::shared_ptr<rclcpp::Node> nh, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaPoseActionServer();

    void handle_accepted(const std::shared_ptr<GoalHandleArmPose>goal_handle);
    void execute(const std::shared_ptr<GoalHandleArmPose>goal_handle);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ArmPose::Goal>goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleArmPose>goal_handle);

 private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    KinovaComm &arm_comm_;
    rclcpp_action::Server<ArmPose>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};

    rclcpp::Time last_nonstall_time_;
    kinova::KinovaPose last_nonstall_pose_;

    std::string link_base_frame_;

    // Parameters
    double stall_interval_seconds_;
    double stall_threshold_;
    double rate_hz_;
    float position_tolerance_;
    float EulerAngle_tolerance_;
    std::string tf_prefix_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_POSE_ACTION_H_s
