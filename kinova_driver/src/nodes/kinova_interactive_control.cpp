#include "rclcpp/rclcpp.hpp"

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <kinova/KinovaTypes.h>
#include "kinova_driver/kinova_ros_types.h"
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"

#include <algorithm>
#include <math.h>

// using namespace visualization_msgs;
// using namespace interactive_markers;

std::shared_ptr<rclcpp::Node> nh;
rclcpp::Subscription<kinova_msgs::msg::JointAngles>::SharedPtr armJoint_sub;
rclcpp::Subscription<kinova_msgs::msg::KinovaPose>::SharedPtr armCartesian_sub;

// Create actionlib client
rclcpp_action::Client<kinova_msgs::action::ArmJointAngles>::SharedPtr ArmJoint_client_ptr_;
rclcpp_action::Client<kinova_msgs::action::ArmPose>::SharedPtr ArmPose_client_ptr_;
rclcpp_action::Client<kinova_msgs::action::SetFingersPosition>::SharedPtr Finger_client_ptr_;

// %Tag(vars)%
interactive_markers::InteractiveMarkerServer* armPose_interMark_server;
interactive_markers::InteractiveMarkerServer* armJoint_interMark_server;
interactive_markers::MenuHandler menu_handler;

kinova::KinovaAngles current_joint_command;
kinova::KinovaPose current_pose_command;

std::string tf_prefix_;
std::string kinova_robotType_;
char robot_category_;
int robot_category_version_;
char wrist_type_;
int arm_joint_number_;
char robot_mode_;
int finger_number_;
int joint_total_number_;

bool mouse_was_up = true;
bool getCurrentCommand = false;
bool is_client_active = false;
// %EndTag(vars)%

// declare processFeedback(), action when interative marker is clicked.
void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback );

// %Tag(Box)%
visualization_msgs::msg::Marker makeBox( visualization_msgs::msg::InteractiveMarker msg )
{
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = msg.scale * 0.25;
    marker.scale.y = msg.scale * 0.25;
    marker.scale.z = msg.scale * 0.25;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::msg::InteractiveMarkerControl makeBoxControl( visualization_msgs::msg::InteractiveMarker msg )
{
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(1DOF)%
void make1DofMarker(const std::string& frame_id, const std::string& axis, unsigned int interaction_mode, const tf2::Vector3& position, const std::string& description, const std::string& name)
{

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    int_marker.pose.position.x = position[0];
    int_marker.pose.position.y = position[1];
    int_marker.pose.position.z = position[2];

    int_marker.scale = 0.08;
    int_marker.name = name;
    int_marker.description = description;

    // insert a box
    makeBoxControl(int_marker);
    visualization_msgs::msg::InteractiveMarkerControl control;

    if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS)
        control.name = "rotate";
    else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS)
        control.name = "move";
    else
        RCLCPP_INFO(nh->get_logger(), "interactive mode should be eigher ROTATE_AXIS or MOVE_AXIS");

    if (axis == "x")
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name += "_x";
    }
    else if (axis == "y")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name += "_y";
    }
    else if (axis == "z")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name += "_z";
    }
    else
        RCLCPP_INFO(nh->get_logger(), "\n The rotation axis must be x, y or z. \n");

    control.interaction_mode = interaction_mode;
    int_marker.controls.push_back(control);

    armJoint_interMark_server->insert(int_marker);
//    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP);
    armJoint_interMark_server->setCallback(int_marker.name,
                                    [&](const auto& feedback) { processFeedback(feedback); });
}
// %EndTag(1DOF)%position


// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const geometry_msgs::msg::Pose pose, bool show_6dof )
{
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = tf_prefix_+"_link_base";
    int_marker.pose = pose;
    int_marker.scale = 0.1;
    int_marker.name = "cartesian_6dof";
    int_marker.description = "6-DOF Cartesian Control";

    // insert a box
    makeBoxControl(int_marker);
    // int_marker.controls[0].interaction_mode = interaction_mode; //Causes segmentation fault

    visualization_msgs::msg::InteractiveMarkerControl control;

    if ( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    }

    if(show_6dof)
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    armPose_interMark_server->insert(int_marker);
    //  armPose_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP);
    armPose_interMark_server->setCallback(int_marker.name,
                                    [&](const auto& feedback) { processFeedback(feedback); });
}
// %EndTag(6DOF)%


// %Tag(send actionlib goals)%
void sendFingerGoal(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    // Finger_client_ptr_ client("/"+tf_prefix_+"_driver/fingers/finger_positions", true);
    Finger_client_ptr_ = rclcpp_action::create_client<kinova_msgs::action::SetFingersPosition>(
      nh,
    //   "/"+tf_prefix_+"_driver/fingers/finger_positions");
      "/fingers/finger_positions");

    if (!Finger_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(nh->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal = kinova_msgs::action::SetFingersPosition::Goal();
    // limit the range of marker to [0 10] before mapping to finger position
    float markerPos;
    float maxMarkerPose = 2.0f;
    markerPos = std::min(maxMarkerPose, std::max(0.0f,float(feedback->pose.position.x)));
    // map marker position to gripper position
    goal.fingers.finger1 = markerPos/maxMarkerPose*5000;
    goal.fingers.finger2 = markerPos/maxMarkerPose*5000;
    goal.fingers.finger3 = 0.0;

    Finger_client_ptr_->async_send_goal(goal);

    RCLCPP_INFO(nh->get_logger(), "client send goal to Finger actionlib: %f \n", goal.fingers.finger1);
}

void result_callback(const rclcpp_action::ClientGoalHandle<kinova_msgs::action::ArmPose>::WrappedResult & result)
{
    is_client_active = false;
    RCLCPP_INFO(nh->get_logger(), "waiting for new action");
    return;
}

void sendArmPoseGoal(geometry_msgs::msg::PoseStamped endeffector_pose)
{
    if (is_client_active) {
        ArmPose_client_ptr_->async_cancel_all_goals();
        RCLCPP_INFO(nh->get_logger(), "canceled previous goals");
    }
    is_client_active = true;

    ArmPose_client_ptr_ = rclcpp_action::create_client<kinova_msgs::action::ArmPose>(
      nh,
      "/"+tf_prefix_+"_driver/tool_pose");

    if (!ArmPose_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(nh->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal = kinova_msgs::action::ArmPose::Goal();

    goal.pose = endeffector_pose;

    RCLCPP_INFO_STREAM(nh->get_logger(), "Goal parent frame is : " << goal.pose.header.frame_id);

    RCLCPP_INFO_STREAM(nh->get_logger(), "Goal to arm pose actionlib: \n"
                    << "  x: " << goal.pose.pose.position.x
                    << ", y: " << goal.pose.pose.position.y
                    << ", z: " << goal.pose.pose.position.z
                    << ", qx: " << goal.pose.pose.orientation.x
                    << ", qy: " << goal.pose.pose.orientation.y
                    << ", qz: " << goal.pose.pose.orientation.z
                    << ", qw: " << goal.pose.pose.orientation.w << std::endl);

    auto send_goal_options = rclcpp_action::Client<kinova_msgs::action::ArmPose>::SendGoalOptions();
    send_goal_options.result_callback = &result_callback;
    ArmPose_client_ptr_->async_send_goal(goal, send_goal_options);

}


void sendArmJointGoal(const std::string marker_name, double joint_offset)
{
    ArmJoint_client_ptr_ = rclcpp_action::create_client<kinova_msgs::action::ArmJointAngles>(
      nh,
      "/"+tf_prefix_+"_driver/joint_angles");

    if (!ArmJoint_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(nh->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal = kinova_msgs::action::ArmJointAngles::Goal();

    goal.angles.joint1 = current_joint_command.Actuator1;
    goal.angles.joint2 = current_joint_command.Actuator2;
    goal.angles.joint3 = current_joint_command.Actuator3;
    goal.angles.joint4 = current_joint_command.Actuator4;
    if(arm_joint_number_==4)
    {
        goal.angles.joint5 = 0.0;
        goal.angles.joint6 = 0.0;
    }
    else if(arm_joint_number_==6)
    {
        goal.angles.joint5 = current_joint_command.Actuator5;
        goal.angles.joint6 = current_joint_command.Actuator6;
    }

    // map marker position to the position of arm joint.
    switch(*marker_name.rbegin()-'0')
    {
    case 1:
        goal.angles.joint1 += joint_offset;
        break;
    case 2:
        goal.angles.joint2 += joint_offset;
        break;
    case 3:
        goal.angles.joint3 += joint_offset;
        break;
    case 4:
        goal.angles.joint4 += joint_offset;
        break;
    case 5:
        if(arm_joint_number_==6)
        {
            goal.angles.joint5 += joint_offset;
        }
        break;
    case 6:
        if(arm_joint_number_==6)
        {
            goal.angles.joint6 += joint_offset;
        }
        break;
    }

    RCLCPP_INFO(nh->get_logger(),  " joint goal is set as : %f, %f, %f, %f, %f, %f (degree)\n",  goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);

    ArmJoint_client_ptr_->async_send_goal(goal);
}
// %EndTag(send actionlib goals)%


// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback )
{
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.stamp = nh->get_clock()->now();
    poseStamped.header.frame_id = tf_prefix_+"_link_base";
    tf2::Quaternion quaternion_mousedown, quaternion_mouseup;

    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    double roll_mousedown, pitch_mousedown, yaw_mousedown;
    double roll_mouseup, pitch_mouseup, yaw_mouseup;
    switch ( feedback->event_type )
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
        break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
        // the moment mouse is clicked down, not the moment keeps down.
        if(mouse_was_up=true)
        {
            quaternion_mousedown = tf2::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
            tf2::Matrix3x3(quaternion_mousedown).getRPY(roll_mousedown, pitch_mousedown, yaw_mousedown);
            if (feedback->marker_name == "cartesian_6dof")
            {
                RCLCPP_INFO_STREAM(nh->get_logger(), "cartesian_6dof control mode.");
            }
            else
            {
                RCLCPP_INFO_STREAM(nh->get_logger(), "Joint control is mode.");
//                RCLCPP_INFO_STREAM(nh->get_logger(),  s.str() << ": mouse DOWN (refers to current status): "
//                                 << feedback->marker_name
//                                 << ": " << yaw_mousedown
//                                 << "\nframe: " << feedback->header.frame_id
//                                 << " time: " << feedback->header.stamp.sec << "sec, "
//                                 << feedback->header.stamp.nsec << " nsec" );
            }
        }

        mouse_was_up = false;
        break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
        quaternion_mouseup = tf2::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
        tf2::Matrix3x3(quaternion_mouseup).getRPY(roll_mouseup, pitch_mouseup, yaw_mouseup);
        if (feedback->marker_name == "cartesian_6dof")
        {
            geometry_msgs::msg::PoseStamped pose_endeffector;
            pose_endeffector.header.frame_id = tf_prefix_+"_link_base";
            pose_endeffector.header.stamp = nh->get_clock()->now();

            pose_endeffector.pose = feedback->pose;
//            RCLCPP_INFO_STREAM(nh->get_logger(),  s.str() << ": mouse UP (refers to command): "
//                             << "\nposition = "
//                             << feedback->pose.position.x
//                             << ", " << feedback->pose.position.y
//                             << ", " << feedback->pose.position.z
//                             << "\norientation = "
//                             << "w: " << feedback->pose.orientation.w
//                             << ", x: " << feedback->pose.orientation.x
//                             << ", y: " << feedback->pose.orientation.y
//                             << ", z: " << feedback->pose.orientation.z
//                             << "\nRollPitchYaw = "
//                             << ": Roll: " << roll_mouseup
//                             << ", Pitch: " << pitch_mouseup
//                             << ", Yaw: " << yaw_mouseup
//                             << "\nframe: " << feedback->header.frame_id);

//            RCLCPP_INFO_STREAM(nh->get_logger(), "Pose_endeffector parent frame is : " << pose_endeffector.header.frame_id);
            sendArmPoseGoal(pose_endeffector);
        }
        else
        {
//            RCLCPP_INFO_STREAM(nh->get_logger(),  s.str() << ": mouse UP (refers to command): "
//                             << feedback->marker_name
//                             << ": " << yaw_mouseup
//                             << "\nframe: " << feedback->header.frame_id
//                             << " time: " << feedback->header.stamp.sec << "sec, "
//                             << feedback->header.stamp.nsec << " nsec");
            sendArmJointGoal(feedback->marker_name, (yaw_mouseup-yaw_mousedown)*180/M_PI);
        }
        //reset flag for MOUSE_DOWN
        mouse_was_up = true;

        // sendFingerGoal(feedback);

        break;
    }


}
// %EndTag(processFeedback)%


// %Tag(CurrentJoint)%
void currentJointsFeedback(const kinova_msgs::msg::JointAngles::SharedPtr joint_command)
{
    current_joint_command.Actuator1 = joint_command->joint1;
    current_joint_command.Actuator2 = joint_command->joint2;
    current_joint_command.Actuator3 = joint_command->joint3;
    current_joint_command.Actuator4 = joint_command->joint4;
    current_joint_command.Actuator5 = joint_command->joint5;
    current_joint_command.Actuator6 = joint_command->joint6;
}
// %EndTag(CurrentJoint)%

// %Tag(CurrentPose)%
void currentPoseFeedback(const kinova_msgs::msg::KinovaPose::SharedPtr pose_command)
{
    current_pose_command.X = pose_command->x;
    current_pose_command.Y = pose_command->y;
    current_pose_command.Z = pose_command->z;
    current_pose_command.ThetaX = pose_command->theta_x;
    current_pose_command.ThetaY = pose_command->theta_y;
    current_pose_command.ThetaZ = pose_command->theta_z;

    if (getCurrentCommand == false)
    {
        geometry_msgs::msg::Pose pose; // home position for j2n6
        pose.position.x = current_pose_command.X;
        pose.position.y = current_pose_command.Y;
        pose.position.z = current_pose_command.Z;
        tf2::Quaternion q = kinova::EulerXYZ2Quaternion(current_pose_command.ThetaX, current_pose_command.ThetaY, current_pose_command.ThetaZ);
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();
        make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::NONE, pose, true );
        armPose_interMark_server->applyChanges();

        getCurrentCommand = true;
    }

}

// %EndTag(CurrentPose)%
////////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///
int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    nh = std::make_shared<rclcpp::Node>("kinova_interactive_control");

    // Retrieve the (non-option) argument:
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
        return -1;
    }
    else // there is an input...
    {
//        if (valid_kinovaRobotType(kinova_robotType_) == false)
//        {
//            ROS_WARN("Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
//            return -1;
//        }
        kinova_robotType_ = argv[argc-1];
        RCLCPP_INFO(nh->get_logger(), "kinova_robotType is %s.", kinova_robotType_.c_str());
    }

//    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_;

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1]-'0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3]-'0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5]-'0';
    joint_total_number_ = arm_joint_number_ + finger_number_;

    armJoint_sub = nh->create_subscription<kinova_msgs::msg::JointAngles>("/"+tf_prefix_+"_driver/out/joint_command", 1, &currentJointsFeedback);
    armCartesian_sub = nh->create_subscription<kinova_msgs::msg::KinovaPose>("/"+tf_prefix_+"_driver/out/cartesian_command", 1, &currentPoseFeedback);

    armJoint_interMark_server = new interactive_markers::InteractiveMarkerServer(tf_prefix_+"_interactive_control_Joint",nh);
    armPose_interMark_server = new interactive_markers::InteractiveMarkerServer(tf_prefix_+"_interactive_control_Cart",nh);

    rclcpp::Rate(0.1).sleep();

    // %Tag(CreatInteractiveMarkers)%
    // Cartesian Control Marker build based on current pose in currentPoseFeedback().

    // Joint Control
    tf2::Vector3 position = tf2::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_+"_link_1", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "1st Axis", "marker_joint1");
    make1DofMarker(tf_prefix_+"_link_2", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "2nd Axis", "marker_joint2");
    make1DofMarker(tf_prefix_+"_link_3", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "3rd Axis", "marker_joint3");
    make1DofMarker(tf_prefix_+"_link_4", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "4th Axis", "marker_joint4");
    if(arm_joint_number_==6)
    {
        make1DofMarker(tf_prefix_+"_link_5", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "5th Axis", "marker_joint5");
        make1DofMarker(tf_prefix_+"_link_6", "z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, position, "6th Axis", "marker_joint6");
    }
    // %EndTag(CreatInteractiveMarkers)%

    armJoint_interMark_server->applyChanges();
    armPose_interMark_server->applyChanges();

    rclcpp::spin(nh);

    delete armJoint_interMark_server;
    delete armPose_interMark_server;
    rclcpp::shutdown();

    return 0;

}


