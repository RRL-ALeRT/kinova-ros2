
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kinova_msgs/action/arm_joint_angles.hpp"

#include <kinova_driver/joint_trajectory_action_server.h>

template <typename T>
float deg(T rad) {return (rad * 180) / M_PI;}; 

namespace kinova
{
using FJTAS = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJTAS = rclcpp_action::ServerGoalHandle<FJTAS>;

using AJAAC = kinova_msgs::action::ArmJointAngles;
using GoalHandleAJAAC = rclcpp_action::ClientGoalHandle<AJAAC>;

    class PositionalActionController
    {
        typedef std::vector<trajectory_msgs::msg::JointTrajectoryPoint> JTPointVector;

    public:
        PositionalActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name);
        ~PositionalActionController();

        void handle_accepted(const std::shared_ptr<GoalHandleFJTAS>gh);
        void execute(const std::shared_ptr<GoalHandleFJTAS>gh);
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal>goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJTAS>gh);

    private:
        std::shared_ptr<rclcpp::Node> nh_;

        rclcpp_action::Server<FJTAS>::SharedPtr action_server_follow_;
        rclcpp_action::Client<AJAAC>::SharedPtr action_client_arm_angles_;
        rclcpp::Subscription<kinova_msgs::msg::JointAngles>::SharedPtr sub_controller_state_;

        bool has_active_goal_;
        bool first_fb_;
        bool got_a_message_;
        rclcpp::Time start_time_;
        std::shared_ptr<GoalHandleFJTAS> active_goal_;
        trajectory_msgs::msg::JointTrajectory current_traj_;
        kinova_msgs::msg::JointAngles last_controller_state_, empty_controller_state_;

        std::shared_ptr<FJTAS::Result> active_result_;
        rclcpp_action::Client<AJAAC>::SendGoalOptions send_goal_options;

        bool goal_succeeded, goal_canceled, goal_aborted, goal_error;
        bool final_goal_aborted = false;
        bool final_goal_succeeded = false;

        std::vector<std::string> joint_names_;
        std::map<std::string,double> goal_constraints_;
        std::map<std::string,double> trajectory_constraints_;
        double goal_time_constraint_;
        double stopped_velocity_tolerance_;

        void goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh);
        void result_callback(const rclcpp_action::ClientGoalHandle<AJAAC>::WrappedResult & result);
        // void cancelCBFollow(FJTAS::GoalHandle gh);
        void watchdog();

    };
}

using namespace kinova;

PositionalActionController::PositionalActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name):
    nh_(n),
    has_active_goal_(false)
{
    std::string robot_type = robot_name;
    std::string address;

    address = "/" + robot_name + "/follow_joint_trajectory";
    action_server_follow_ = rclcpp_action::create_server<FJTAS>(
        nh_,
        address,
        std::bind(&PositionalActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PositionalActionController::handle_cancel, this, std::placeholders::_1),
        std::bind(&PositionalActionController::handle_accepted, this, std::placeholders::_1));

    address = "/" + robot_name + "_driver/joint_angles";
    action_client_arm_angles_ = rclcpp_action::create_client<AJAAC>(nh_, address);

    int arm_joint_num = robot_type[3]-'0';
    joint_names_.resize(arm_joint_num);

    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_name + "_joint_" + std::to_string(i+1);
    }

    int goal_time_constraint_ = 0;
    if (!nh_->has_parameter("constraints/goal_time"))
        nh_->declare_parameter("constraints/goal_time", goal_time_constraint_);
    nh_->get_parameter("constraints/goal_time", goal_time_constraint_);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::string ns = std::string("constraints/") + joint_names_[i];
        double g = -1.0;
        double t = -1.0;

        if (!nh_->has_parameter(ns + "/goal"))
            nh_->declare_parameter(ns + "/goal", g);
        nh_->get_parameter(ns + "/goal", g);
        if (!nh_->has_parameter(ns + "/trajectory"))
            nh_->declare_parameter(ns + "/trajectory", t);
        nh_->get_parameter(ns + "/trajectory", t);

        goal_constraints_[joint_names_[i]] = g;
        trajectory_constraints_[joint_names_[i]] = t;
    }

    double stopped_velocity_tolerance_ = 0.01;
    if (!nh_->has_parameter("constraints/stopped_velocity_tolerance"))
        nh_->declare_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);
    nh_->get_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);
    

    if (!action_client_arm_angles_->wait_for_action_server()) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Action server not found: /" + robot_name + "_driver/joint_angles");
    }
    RCLCPP_INFO(nh_->get_logger(), "Waiting for an plan execution (goal) from Moveit");
}


PositionalActionController::~PositionalActionController()
{
}


static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (count(b.begin(), b.end(), a[i]) != 1)
            return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
        if (count(a.begin(), a.end(), b[i]) != 1)
            return false;
    }

    return true;
}


void PositionalActionController::watchdog()
{
    rclcpp::Time now = nh_->get_clock()->now();
    has_active_goal_ = false;
}


rclcpp_action::GoalResponse PositionalActionController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal>goal)
{
    RCLCPP_INFO(nh_->get_logger(), "Received goal request");
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionalActionController::handle_cancel(const std::shared_ptr<GoalHandleFJTAS> gh)
{
    RCLCPP_INFO(nh_->get_logger(), "Received request to cancel goal");
    (void) gh;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionalActionController::handle_accepted(const std::shared_ptr<GoalHandleFJTAS> goal_handle)
{
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server accepted goal!");
    std::thread
    {
        std::bind(&PositionalActionController::goalCBFollow, this, std::placeholders::_1), goal_handle
    }.detach();
}

void PositionalActionController::result_callback(const rclcpp_action::ClientGoalHandle<AJAAC>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            goal_succeeded = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            goal_aborted = true;
            RCLCPP_ERROR(nh_->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            goal_canceled = true;
            RCLCPP_ERROR(nh_->get_logger(), "Goal was canceled");
            return;
        default:
            goal_error = true;
            RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
            return;
    }
    if (final_goal_succeeded) active_goal_->succeed(active_result_);
    else if (final_goal_aborted) active_goal_->abort(active_result_);
}

void PositionalActionController::goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh)
{
    const auto goal = gh->get_goal();
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server received goal!");
    
    // Sends the trajectory along to the controller
    current_traj_ = goal->trajectory;

    std::vector<AJAAC::Goal> client_goals;
    send_goal_options = rclcpp_action::Client<AJAAC>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&PositionalActionController::result_callback, this, std::placeholders::_1);
    for (int i = 0; i < current_traj_.points.size(); i++) {
        auto client_goal = AJAAC::Goal();
        client_goal.angles.joint1 = deg<float>(current_traj_.points[i].positions[0]);
        client_goal.angles.joint2 = deg<float>(current_traj_.points[i].positions[1]);
        client_goal.angles.joint3 = deg<float>(current_traj_.points[i].positions[2]);
        client_goal.angles.joint4 = deg<float>(current_traj_.points[i].positions[3]);
        client_goal.angles.joint5 = deg<float>(current_traj_.points[i].positions[4]);
        client_goal.angles.joint6 = deg<float>(current_traj_.points[i].positions[5]);
        client_goal.angles.joint7 = deg<float>(current_traj_.points[i].positions[6]);

        client_goals.push_back(client_goal);
    }

    goal_succeeded = false;
    goal_canceled = false;
    goal_aborted = false;
    goal_error = false;
    bool final_goal_aborted = false;
    bool final_goal_succeeded = false;

    int i = 0;
    action_client_arm_angles_->async_send_goal(client_goals[i], send_goal_options);
    i++;
    while (rclcpp::ok()) {
        if (goal_canceled || goal_aborted || goal_error) {
            RCLCPP_INFO(nh_->get_logger(), "Final goal was not reached successfully");
            // active_goal_->abort(active_result_);
            final_goal_aborted = true;
            break;
        }
        else if (goal_succeeded) {
            if (i == client_goals.size()) {
                RCLCPP_INFO(nh_->get_logger(), "Reached goal");
                // active_goal_->succeed(active_result_);
                final_goal_succeeded = true;
                break;
            }
            else {
                action_client_arm_angles_->async_send_goal(client_goals[i], send_goal_options);
                goal_succeeded = false;
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Sending point: " << i);
            }
            i++;
        }
        rclcpp::Rate(10).sleep();
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("arm_joint_angles_action_server");

    // Retrieve the (non-option) argument:
    std::string robot_name = "";
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robot_name provided in the argument!" << std::endl;
        return -1;
    }
    else
    {
        robot_name = argv[argc-1];
    }
    kinova::PositionalActionController jtac(node, robot_name);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
