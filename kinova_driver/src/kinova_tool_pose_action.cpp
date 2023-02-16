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
 *  File: kinova_tool_pose_action.cpp
 *  Desc: Class for moving/querying kinova arm.
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

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_ros_types.h"
#include <string>

geometry_msgs::msg::PoseStamped transform_pose(const std::shared_ptr<tf2_ros::Buffer> tf_, const std::shared_ptr<rclcpp::Node> tp_node, const geometry_msgs::msg::PoseStamped in, const std::string target_frame)
{
    geometry_msgs::msg::PoseStamped out;
    auto source_to_target = tf_->lookupTransform(in.header.frame_id, target_frame, in.header.stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform t_source_to_target(
        tf2::Matrix3x3(tf2::Quaternion(
        source_to_target.transform.rotation.x,source_to_target.transform.rotation.y,source_to_target.transform.rotation.z,source_to_target.transform.rotation.w)),
        tf2::Vector3(source_to_target.transform.translation.x, source_to_target.transform.translation.y, source_to_target.transform.translation.z)
    );

    auto t_position = t_source_to_target.inverse()*(tf2::Vector3(in.pose.position.x, in.pose.position.y, in.pose.position.z));
    auto t_orientation = t_source_to_target.inverse()*(tf2::Quaternion(in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w));
    out.pose.position.x = t_position.getX();
    out.pose.position.y = t_position.getY();
    out.pose.position.z = t_position.getZ();
    out.pose.orientation.x = t_orientation.getX();
    out.pose.orientation.y = t_orientation.getY();
    out.pose.orientation.z = t_orientation.getZ();
    out.pose.orientation.w = t_orientation.getW();
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;
    
    return out;
}


namespace kinova
{

KinovaPoseActionServer::KinovaPoseActionServer(KinovaComm &arm_comm, const std::shared_ptr<rclcpp::Node> nh, const std::string &kinova_robotType, const std::string &kinova_robotName)
    : arm_comm_(arm_comm),
      node_handle_(nh),
      kinova_robotType_(kinova_robotType),
      kinova_robotName_(kinova_robotName)
{
    double position_tolerance = 0.01;
    double EulerAngle_tolerance = 2.0*M_PI/180;

    if (!node_handle_->has_parameter("stall_interval_seconds"))
        node_handle_->declare_parameter("stall_interval_seconds", 1.0);
    if (!node_handle_->has_parameter("stall_threshold"))
        node_handle_->declare_parameter("stall_threshold", 0.005);
    if (!node_handle_->has_parameter("rate_hz"))
        node_handle_->declare_parameter("rate_hz", 10.0);
    if (!node_handle_->has_parameter("position_tolerance"))
        node_handle_->declare_parameter("position_tolerance", position_tolerance);
    if (!node_handle_->has_parameter("EulerAngle_tolerance"))
        node_handle_->declare_parameter("EulerAngle_tolerance", EulerAngle_tolerance);
    node_handle_->get_parameter("stall_interval_seconds", stall_interval_seconds_);
    node_handle_->get_parameter("stall_threshold", stall_threshold_);
    node_handle_->get_parameter("rate_hz", rate_hz_);
    node_handle_->get_parameter("position_tolerance", position_tolerance);
    node_handle_->get_parameter("EulerAngle_tolerance", EulerAngle_tolerance);

    //    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType + "_";

    position_tolerance_ = static_cast<float>(position_tolerance);
    EulerAngle_tolerance_ = static_cast<float>(EulerAngle_tolerance);
    std::stringstream ss;
    ss << tf_prefix_ << "link_base";
    link_base_frame_ = ss.str();

    action_server_ = rclcpp_action::create_server<ArmPose>(
        node_handle_,
        "/"+tf_prefix_+"driver/tool_pose",
        std::bind(&KinovaPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&KinovaPoseActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&KinovaPoseActionServer::handle_accepted, this, std::placeholders::_1));

    double tmp_val = 30;
    tf_ = std::make_unique<tf2_ros::Buffer>(node_handle_->get_clock(),
        tf2::durationFromSec(tmp_val));
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp::node_interfaces::get_node_base_interface(node_handle_),
        rclcpp::node_interfaces::get_node_timers_interface(node_handle_));
    tf_->setCreateTimerInterface(timer_interface);
    listener = std::make_shared<tf2_ros::TransformListener>(*tf_);
}


KinovaPoseActionServer::~KinovaPoseActionServer()
{
}

rclcpp_action::GoalResponse KinovaPoseActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ArmPose::Goal>goal)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Received goal request with pose %d", goal->pose);
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KinovaPoseActionServer::handle_cancel(const std::shared_ptr<GoalHandleArmPose> goal_handle)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Received request to cancel goal");
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void KinovaPoseActionServer::handle_accepted(const std::shared_ptr<GoalHandleArmPose> goal_handle)
{
 	// this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread
    {
        std::bind(&KinovaPoseActionServer::execute, this, std::placeholders::_1), goal_handle
    }.detach();
}

void KinovaPoseActionServer::execute(const std::shared_ptr<GoalHandleArmPose> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<ArmPose::Feedback>();
    auto result = std::make_shared<ArmPose::Result>();
    feedback->pose.header.frame_id = goal->pose.header.frame_id;
    result->pose.header.frame_id = goal->pose.header.frame_id;

    rclcpp::Time current_time = node_handle_->get_clock()->now();
    KinovaPose current_pose;
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header.frame_id = link_base_frame_;

    try
    {
        try {
            local_pose = transform_pose(tf_, node_handle_, goal->pose, local_pose.header.frame_id);
        }
        catch (tf2::ConnectivityException& e) {
            RCLCPP_WARN(node_handle_->get_logger(), e.what());
            return;
        }
        catch (tf2::ExtrapolationException& e) {
            RCLCPP_WARN(node_handle_->get_logger(), e.what());
            return;
        }
        catch (tf2::LookupException& e) {
            RCLCPP_WARN(node_handle_->get_logger(), e.what());
            return;
        }
        // Put the goal pose into the frame used by the arm
        if (rclcpp::ok() && !local_pose.header.stamp.sec)
        {
            RCLCPP_ERROR(node_handle_->get_logger(), "Could not get transfrom from %s to %s, aborting cartesian movement",
                      link_base_frame_.c_str(), goal->pose.header.frame_id.c_str());
            goal_handle->abort(result);
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }
        arm_comm_.getCartesianPosition(current_pose);
        if (arm_comm_.isStopped())
        {
            RCLCPP_INFO(node_handle_->get_logger(), "Could not complete cartesian action because the arm is 'stopped'.");
            local_pose.pose = current_pose.constructPoseMsg();
            try {
                result->pose = transform_pose(tf_, node_handle_, result->pose, result->pose.header.frame_id);
            }
            catch (tf2::ConnectivityException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
            catch (tf2::ExtrapolationException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
            catch (tf2::LookupException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
            goal_handle->abort(result);
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_pose_ = current_pose;

        KinovaPose target(local_pose.pose);
        RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), std::endl << std::endl << "***-----------------------***" << std::endl << __PRETTY_FUNCTION__ << ":  target X " << target.X << "; Y "<< target.Y << "; Z "<< target.Z << "; ThetaX " << target.ThetaX << "; ThetaY " << target.ThetaY  << "; ThetaZ " << target.ThetaZ << std::endl << "***-----------------------***" << std::endl );
        arm_comm_.setCartesianPosition(target);

        while (rclcpp::ok())
        {
            // without setCartesianPosition() in while loop, robot stopped in the half way, and the goal won't be reached.
            arm_comm_.setCartesianPosition(target);
            // rclcpp::spin_some(node_handle_);

            if (arm_comm_.isStopped())
            {
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "" << __PRETTY_FUNCTION__ << ": arm_comm_.isStopped()");
                result->pose = feedback->pose;
                goal_handle->abort(result);
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                return;
            }
            else if (goal_handle->is_canceling() || !rclcpp::ok())
            {
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "" << __PRETTY_FUNCTION__ << ": action server isPreemptRequested");
                result->pose = feedback->pose;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                goal_handle->canceled(result);
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            arm_comm_.getCartesianPosition(current_pose);
            current_time = node_handle_->get_clock()->now();
            local_pose.pose = current_pose.constructPoseMsg();

            try {
                feedback->pose = transform_pose(tf_, node_handle_, local_pose, feedback->pose.header.frame_id);
            }
            catch (tf2::ConnectivityException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
            catch (tf2::ExtrapolationException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
            catch (tf2::LookupException& e) {
                RCLCPP_WARN(node_handle_->get_logger(), e.what());
                return;
            }
    //            action_server_.publishFeedback(feedback);

            RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "" << __PRETTY_FUNCTION__ << ": current_pose X " << current_pose.X << "; Y "<< current_pose.Y << "; Z "<< current_pose.Z << "; ThetaX " << current_pose.ThetaX << "; ThetaY " << current_pose.ThetaY  << "; ThetaZ " << current_pose.ThetaZ );

            if (target.isCloseToOther(current_pose, position_tolerance_, EulerAngle_tolerance_))
            {
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "" << __PRETTY_FUNCTION__ << ": arm_comm_.isCloseToOther");
                result->pose = feedback->pose;
                goal_handle->succeed(result);
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                break;
            }
            else if (!last_nonstall_pose_.isCloseToOther(current_pose, stall_threshold_, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_pose_ = current_pose;
            }
            else if ((current_time.seconds() - last_nonstall_time_.seconds()) > stall_interval_seconds_)
            {
                RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "" << __PRETTY_FUNCTION__ << ": stall_interval_seconds_");
                // Check if the full stall condition has been meet
                result->pose = feedback->pose;
                if (!arm_comm_.isStopped())
                {
                    arm_comm_.stopAPI();
                    arm_comm_.startAPI();
                }
                //why preemted, if the robot is stalled, trajectory/action failed!
                /*
                action_server_.setPreempted(result);
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                */
                goal_handle->abort(result);
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                return;
            }
            
            rclcpp::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result->pose = feedback->pose;
        RCLCPP_ERROR_STREAM(node_handle_->get_logger(), e.what());
        goal_handle->abort(result);
        RCLCPP_WARN_STREAM(node_handle_->get_logger(), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

}  // namespace kinova
