#ifndef JOINT_TRAJECTORY_ACTION_SERVER_H
#define JOINT_TRAJECTORY_ACTION_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


namespace kinova
{
using FJTAS = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJTAS = rclcpp_action::ServerGoalHandle<FJTAS>;
    class JointTrajectoryActionController
    {
        typedef std::vector<trajectory_msgs::msg::JointTrajectoryPoint> JTPointVector;

    public:
        JointTrajectoryActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name);
        ~JointTrajectoryActionController();

        void handle_accepted(const std::shared_ptr<GoalHandleFJTAS>gh);
        void execute(const std::shared_ptr<GoalHandleFJTAS>gh);
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal>goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJTAS>gh);

    private:
        std::shared_ptr<rclcpp::Node> nh_;

        rclcpp_action::Server<FJTAS>::SharedPtr action_server_follow_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
        rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr sub_controller_state_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;

        bool has_active_goal_;
        bool first_fb_;
        rclcpp::Time start_time_;
        std::shared_ptr<GoalHandleFJTAS> active_goal_;
        std::shared_ptr<FJTAS::Result> active_result_;
        trajectory_msgs::msg::JointTrajectory current_traj_;
        control_msgs::action::FollowJointTrajectory_Feedback last_controller_state_, empty_controller_state_;

        std::vector<std::string> joint_names_;
        std::map<std::string,double> goal_constraints_;
        std::map<std::string,double> trajectory_constraints_;
        double goal_time_constraint_;
        double stopped_velocity_tolerance_;

        void goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh);
        // void cancelCBFollow(FJTAS::GoalHandle gh);
        void controllerStateCB(const control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg);
        void watchdog();

    };
}


#endif // JOINT_TRAJECTORY_ACTION_SERVER_H
