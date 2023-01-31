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
 *  File: kinova_fingers_action.h
 *  Desc: Action server for kinova arm fingers.
 *  Auth: Jeff Schmidt
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

#ifndef KINOVA_DRIVER_KINOVA_FINGERS_ACTION_H
#define KINOVA_DRIVER_KINOVA_FINGERS_ACTION_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "kinova_msgs/action/set_fingers_position.hpp"

#include "kinova_driver/kinova_comm.h"


namespace kinova
{
using SetFingersPosition = kinova_msgs::action::SetFingersPosition;
using GoalHandleSetFingersPosition = rclcpp_action::ServerGoalHandle<SetFingersPosition>;

class KinovaFingersActionServer
{
 public:
    KinovaFingersActionServer(KinovaComm &, const std::shared_ptr<rclcpp::Node> n, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaFingersActionServer();

    void handle_accepted(const std::shared_ptr<GoalHandleSetFingersPosition>goal_handle);
    void execute(const std::shared_ptr<GoalHandleSetFingersPosition>goal_handle);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SetFingersPosition::Goal>goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetFingersPosition>goal_handle);

 private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    KinovaComm &arm_comm_;
    rclcpp_action::Server<SetFingersPosition>::SharedPtr action_server_;

    rclcpp::Time last_nonstall_time_;
    kinova::FingerAngles last_nonstall_finger_positions_;

    // Parameters
    double stall_interval_seconds_;
    double stall_threshold_;
    double rate_hz_;
    float tolerance_;

    std::string tf_prefix_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_FINGERS_ACTION_H
