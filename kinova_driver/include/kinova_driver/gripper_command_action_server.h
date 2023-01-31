#ifndef GRIPPER_COMMAND_ACTION_SERVER_H
#define GRIPPER_COMMAND_ACTION_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "kinova_msgs/action/set_fingers_position.hpp"


namespace kinova
{
using GCAS = control_msgs::action::GripperCommand;
using GoalHandleGCAS = rclcpp_action::ServerGoalHandle<GCAS>;

using SFPAC = kinova_msgs::action::SetFingersPosition;
using GoalHandleSFPAC = rclcpp_action::ClientGoalHandle<SFPAC>;

class GripperCommandActionController
    {
    public:
        GripperCommandActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name);
        ~GripperCommandActionController();

        void handle_accepted(const std::shared_ptr<GoalHandleGCAS>gh);
        void execute(const std::shared_ptr<GoalHandleGCAS>gh);
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GCAS::Goal>goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGCAS>gh);

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp_action::Server<GCAS>::SharedPtr action_server_gripper_command_;
        rclcpp_action::Client<SFPAC>::SharedPtr action_client_set_finger_;

        rclcpp::Subscription<kinova_msgs::msg::FingerPosition>::SharedPtr sub_fingers_state_;

        bool has_active_goal_;
        bool is_client_active = false;
        std::shared_ptr<GoalHandleGCAS> active_goal_;
        std::shared_ptr<GCAS::Result> active_result_;
        kinova_msgs::msg::FingerPosition last_finger_state_, empty_finger_state_;

        std::vector<std::string> gripper_joint_names_;
        double gripper_command_goal_constraint_;
        double gripper_joint_num_;

        double finger_max_turn_ ; // maximum turn (KinovaFinger defalt unit) value
        double finger_conv_ratio_; // finger value convert ratio defined in kinova_driver/kinova_arm

        void goalCBFollow(std::shared_ptr<GoalHandleGCAS> gh);
        // void cancelCBFollow(GCAS::GoalHandle gh);
        void controllerStateCB(const kinova_msgs::msg::FingerPosition::SharedPtr msg);
        void result_callback(const rclcpp_action::ClientGoalHandle<kinova_msgs::action::SetFingersPosition>::WrappedResult & result);
    };
}



#endif // GRIPPER_COMMAND_ACTION_SERVER_H
