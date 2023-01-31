#include <kinova_driver/gripper_command_action_server.h>

using namespace kinova;




GripperCommandActionController::GripperCommandActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name):
    nh_(n),
    has_active_goal_(false)
{    
    std::string address;
    address = "/" + robot_name + "_gripper/gripper_command";

    action_server_gripper_command_ = rclcpp_action::create_server<GCAS>(
        nh_,
        address,
        std::bind(&GripperCommandActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GripperCommandActionController::handle_cancel, this, std::placeholders::_1),
        std::bind(&GripperCommandActionController::handle_accepted, this, std::placeholders::_1));

    address = "/" + robot_name + "_driver/finger_positions";

    action_client_set_finger_ = rclcpp_action::create_client<SFPAC>(
      nh_, address);

    // unit of GripperCommand is meter. Gripper_command_goal from "moveit.rviz" is in unit of radians.
    double gripper_command_goal_constraint_ = 0.01;
    if (!nh_->has_parameter("gripper_command_goal_constraint_"))
        nh_->declare_parameter("gripper_command_goal_constraint_", gripper_command_goal_constraint_);
    nh_->get_parameter("gripper_command_goal_constraint_", gripper_command_goal_constraint_);

    finger_max_turn_ = 6400.0;
    if (!nh_->has_parameter("finger_max_turn_"))
        nh_->declare_parameter("finger_max_turn_", finger_max_turn_);
    nh_->get_parameter("finger_max_turn_", finger_max_turn_);

    finger_conv_ratio_ = 1.4 / 6400.0;
    if (!nh_->has_parameter("finger_conv_ratio_"))
        nh_->declare_parameter("finger_conv_ratio_", finger_conv_ratio_);
    nh_->get_parameter("finger_conv_ratio_", finger_conv_ratio_);

    gripper_joint_num_ = 3;
    gripper_joint_names_.resize(gripper_joint_num_);

    for (uint i = 0; i<gripper_joint_names_.size(); i++)
    {
        gripper_joint_names_[i] = "joint_finger_" + std::to_string(i+1);
    }

    address = "/" + robot_name + "_driver/out/finger_position";
    sub_fingers_state_ = nh_->create_subscription<kinova_msgs::msg::FingerPosition>(address, 1, std::bind(&GripperCommandActionController::controllerStateCB, this, std::placeholders::_1));

    long unsigned int started_waiting_for_controller = nh_->get_clock()->now().seconds();
    while (rclcpp::ok() && last_finger_state_ == empty_finger_state_)
    {
        if (started_waiting_for_controller != nh_->get_clock()->now().seconds() &&
                nh_->get_clock()->now().seconds() > started_waiting_for_controller + 30)
        {
            RCLCPP_WARN(nh_->get_logger(), "No update on finger positions for 30 seconds");
            started_waiting_for_controller = nh_->get_clock()->now().seconds();
        }
        rclcpp::spin_some(nh_);
        rclcpp::Rate(0.1).sleep();
    }

    if (!action_client_set_finger_->wait_for_action_server()) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Action server not found: /" + robot_name + "_driver/finger_positions");
    }

    RCLCPP_INFO(nh_->get_logger(), "Start Gripper_Command_Trajectory_Action server!");
}


GripperCommandActionController::~GripperCommandActionController()
{
    sub_fingers_state_.reset();
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

rclcpp_action::GoalResponse GripperCommandActionController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GCAS::Goal>goal)
{
    RCLCPP_INFO(nh_->get_logger(), "Received goal request");
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperCommandActionController::handle_cancel(const std::shared_ptr<GoalHandleGCAS> gh)
{
    RCLCPP_INFO(nh_->get_logger(), "Received request to cancel goal");
    (void) gh;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperCommandActionController::handle_accepted(const std::shared_ptr<GoalHandleGCAS> goal_handle)
{
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server accepted goal!");
 	// this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread
    {
        std::bind(&GripperCommandActionController::goalCBFollow, this, std::placeholders::_1), goal_handle
    }.detach();
}

void GripperCommandActionController::result_callback(const rclcpp_action::ClientGoalHandle<kinova_msgs::action::SetFingersPosition>::WrappedResult & result)
{
    has_active_goal_ = false;
    RCLCPP_INFO(nh_->get_logger(), "waiting for new action");
    return;
}

void GripperCommandActionController::goalCBFollow(std::shared_ptr<GoalHandleGCAS> gh)
{
    active_goal_ = gh;
    const auto goal = gh->get_goal();
    RCLCPP_INFO(nh_->get_logger(), "Gripper_command_action_server received goal!");

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
        // Marks the current goal as canceled.
        action_client_set_finger_->async_cancel_all_goals();
        RCLCPP_INFO(nh_->get_logger(), "A new gripper command goal is comming, so the previous one is cancelled.");
        has_active_goal_ = false;
    }

    has_active_goal_ = true;
    // gripper_gap 0 is open, 1.4 is close. Here command.position from Moveit.rviz is finger radian value, rather than Cartesian gap of gripper in meter.
    double gripper_gap = goal->command.position * (180 / M_PI);
    RCLCPP_INFO(nh_->get_logger(), "Gripper_command_action_server accepted goal!");

    // send the goal to fingers position action server, Todo: check what's going on mathematically
    auto client_goal = kinova_msgs::action::SetFingersPosition::Goal();
    client_goal.fingers.finger1 = gripper_gap;
    client_goal.fingers.finger2 = gripper_gap;
    if (gripper_joint_num_ == 3)
        client_goal.fingers.finger3 = gripper_gap;
    else
        client_goal.fingers.finger3 = 0.0;

    RCLCPP_INFO(nh_->get_logger(), "Gripper_command_action_server published goal via command publisher!");
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Finger angles target " << client_goal.fingers.finger1 << "," << client_goal.fingers.finger2 << "," << client_goal.fingers.finger3);
    auto send_goal_options = rclcpp_action::Client<kinova_msgs::action::SetFingersPosition>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GripperCommandActionController::result_callback, this, std::placeholders::_1);
    action_client_set_finger_->async_send_goal(client_goal, send_goal_options);
}

void GripperCommandActionController::controllerStateCB(const kinova_msgs::msg::FingerPosition::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(nh_->get_logger(), "Gripper_command__action_server receive feedback of trajectory state from topic: /trajectory_controller/state");

    last_finger_state_ = *msg;

    rclcpp::Time now = nh_->get_clock()->now();

    if (!has_active_goal_)
        return;

    // Checks that we have ended inside the goal constraints, FingerPosition has 3 values
    double abs_error1 = fabs(active_goal_->get_goal()->command.position - msg->finger1 * finger_conv_ratio_);
    double abs_error2 = fabs(active_goal_->get_goal()->command.position - msg->finger2 * finger_conv_ratio_);
    double abs_error3 = fabs(active_goal_->get_goal()->command.position - msg->finger3 * finger_conv_ratio_);

    if (abs_error1<gripper_command_goal_constraint_  && abs_error2<gripper_command_goal_constraint_ && abs_error3<gripper_command_goal_constraint_)
    {
        RCLCPP_INFO(nh_->get_logger(), "Gripper command goal succeeded!");
        active_goal_->succeed(active_result_);
        has_active_goal_ = false;
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("gripper_command_action_server");

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
    kinova::GripperCommandActionController gcac(node,robot_name);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

