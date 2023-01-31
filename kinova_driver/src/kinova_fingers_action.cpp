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
 *  File: kinova_fingers_action.cpp
 *  Desc: Class for moving/querying kinova arm fingers.
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

#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_ros_types.h"


namespace kinova
{

KinovaFingersActionServer::KinovaFingersActionServer(KinovaComm &arm_comm, const std::shared_ptr<rclcpp::Node> nh, const std::string &kinova_robotType, const std::string &kinova_robotName)
    : arm_comm_(arm_comm),
      node_handle_(nh),
      kinova_robotType_(kinova_robotType),
      kinova_robotName_(kinova_robotName)
{
    double tolerance = 6400.0*0.01;
    if (!node_handle_->has_parameter("stall_interval_seconds"))
        node_handle_->declare_parameter("stall_interval_seconds", 0.5);
    if (!node_handle_->has_parameter("stall_threshold"))
        node_handle_->declare_parameter("stall_threshold", 1.0);
    if (!node_handle_->has_parameter("rate_hz"))
        node_handle_->declare_parameter("rate_hz", 10.0);
    if (!node_handle_->has_parameter("tolerance"))
        node_handle_->declare_parameter("tolerance", tolerance);

    node_handle_->get_parameter("stall_interval_seconds", stall_interval_seconds_);
    node_handle_->get_parameter("stall_threshold", stall_threshold_);
    node_handle_->get_parameter("rate_hz", rate_hz_);
    node_handle_->get_parameter("tolerance", tolerance);

    tolerance_ = static_cast<float>(tolerance);

    tf_prefix_ = kinova_robotType + "_";
    action_server_ = rclcpp_action::create_server<SetFingersPosition>(
        node_handle_,
        "/"+tf_prefix_+"driver/finger_positions",
        std::bind(&KinovaFingersActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&KinovaFingersActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&KinovaFingersActionServer::handle_accepted, this, std::placeholders::_1));
}


KinovaFingersActionServer::~KinovaFingersActionServer()
{
}

rclcpp_action::GoalResponse KinovaFingersActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SetFingersPosition::Goal>goal)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Received goal request with pose %d", goal->fingers);
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KinovaFingersActionServer::handle_cancel(const std::shared_ptr<GoalHandleSetFingersPosition> goal_handle)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Received request to cancel goal");
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void KinovaFingersActionServer::handle_accepted(const std::shared_ptr<GoalHandleSetFingersPosition> goal_handle)
{
 	// this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread
    {
        std::bind(&KinovaFingersActionServer::execute, this, std::placeholders::_1), goal_handle
    }.detach();
}

void KinovaFingersActionServer::execute(const std::shared_ptr<GoalHandleSetFingersPosition> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    if ((arm_comm_.numFingers() < 3) && (goal->fingers.finger3 != 0.0))
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Detected that the third finger command was non-zero even though there "
                 "are only two fingers on the gripper. The goal for the third finger "
                 "should be set to zero or you make experience delays in action results.");
    }

    auto feedback = std::make_shared<SetFingersPosition::Feedback>();
    auto result = std::make_shared<SetFingersPosition::Result>();    
    FingerAngles current_finger_positions;
    rclcpp::Time current_time = node_handle_->get_clock()->now();

    try
    {
        arm_comm_.getFingerPositions(current_finger_positions);
        

        if (arm_comm_.isStopped())
        {
            RCLCPP_INFO(node_handle_->get_logger(), "Could not complete finger action because the arm is stopped");
            result->fingers = current_finger_positions.constructFingersMsg();
            RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            goal_handle->abort(result);
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_finger_positions_ = current_finger_positions;

        FingerAngles target(goal->fingers);

        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "setting finger angles to: " << target.Finger1 << "," << target.Finger2 << "," << target.Finger3);
        arm_comm_.setFingerPositions(target);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (rclcpp::ok())
        {
            // similar behaviour to setCartesianPosition(), maybe?
            // arm_comm_.setFingerPositions(target);

            if (arm_comm_.isStopped())
            {
                result->fingers = current_finger_positions.constructFingersMsg();
                goal_handle->abort(result);
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                return;
            }
            else if (goal_handle->is_canceling() || !rclcpp::ok())
            {
                result->fingers = current_finger_positions.constructFingersMsg();
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                goal_handle->canceled(result);
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            arm_comm_.getFingerPositions(current_finger_positions);
            current_time = node_handle_->get_clock()->now();
            feedback->fingers = current_finger_positions.constructFingersMsg();
//            action_server_.publishFeedback(feedback);
            if (target.isCloseToOther(current_finger_positions, tolerance_))
            {
                // Check if the action has succeeeded
                result->fingers = current_finger_positions.constructFingersMsg();
                goal_handle->succeed(result);
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                return;
            }
            else if (!last_nonstall_finger_positions_.isCloseToOther(current_finger_positions, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_finger_positions_ = current_finger_positions;
            }
            else if ((current_time.seconds() - last_nonstall_time_.seconds()) > stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result->fingers = current_finger_positions.constructFingersMsg();
                if (!arm_comm_.isStopped())
                    {
                        arm_comm_.stopAPI();
                        arm_comm_.startAPI();
                    }
                //why preemted, if the robot is stalled, trajectory/action failed!
                /*
                action_server_.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                */
                goal_handle->abort(result);
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                return;
            }

            rclcpp::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result->fingers = current_finger_positions.constructFingersMsg();
        RCLCPP_ERROR_STREAM(node_handle_->get_logger(), e.what());
        goal_handle->abort(result);
        RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

}  // namespace kinova
