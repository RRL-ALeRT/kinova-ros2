/*
 * kinova_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_ARM_H
#define KINOVA_DRIVER_KINOVA_ARM_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "kinova_msgs/srv/stop.hpp"
#include "kinova_msgs/srv/start.hpp"
#include "kinova_msgs/srv/home_arm.hpp"
#include "kinova_msgs/msg/joint_velocity.hpp"
#include "kinova_msgs/msg/pose_velocity.hpp"
#include "kinova_msgs/msg/pose_velocity_with_fingers.hpp"
#include "kinova_msgs/msg/pose_velocity_with_finger_velocity.hpp"
#include "kinova_msgs/msg/joint_torque.hpp"
#include "kinova_msgs/msg/finger_position.hpp"
#include "kinova_msgs/msg/joint_angles.hpp"
#include "kinova_msgs/msg/kinova_pose.hpp"
#include "kinova_msgs/srv/set_force_control_params.hpp"
#include "kinova_msgs/srv/set_end_effector_offset.hpp"
#include "kinova_msgs/srv/set_null_space_mode_state.hpp"
#include "kinova_msgs/srv/set_torque_control_mode.hpp"
#include "kinova_msgs/srv/set_torque_control_parameters.hpp"
#include "kinova_msgs/srv/clear_trajectories.hpp"
#include "kinova_msgs/srv/add_pose_to_cartesian_trajectory.hpp"
#include "kinova_msgs/srv/zero_torques.hpp"
#include "kinova_msgs/srv/run_com_parameters_estimation.hpp"
#include "kinova_msgs/msg/cartesian_force.hpp"

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_api.h"


namespace kinova
{

struct robot_info
{
    int id;
    std::string name;
    std::string type;
    std::string serial;
};

class KinovaArm
{
 public:
    KinovaArm(KinovaComm& arm, const std::shared_ptr<rclcpp::Node> node_handle, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaArm();

    //Subscriber callbacks --------------------------------------------------------
    void jointVelocityCallback(const kinova_msgs::msg::JointVelocity::SharedPtr joint_vel);
    void cartesianVelocityCallback(const kinova_msgs::msg::PoseVelocity::SharedPtr cartesian_vel);
    void cartesianVelocityWithFingersCallback(const kinova_msgs::msg::PoseVelocityWithFingers::SharedPtr cartesian_vel_with_fingers);
    void cartesianVelocityWithFingerVelocityCallback(const kinova_msgs::msg::PoseVelocityWithFingerVelocity::SharedPtr cartesian_vel_with_fingers);
    void jointTorqueSubscriberCallback(const kinova_msgs::msg::JointTorque::SharedPtr joint_torque);
    void forceSubscriberCallback(const kinova_msgs::msg::CartesianForce::SharedPtr force);

    // Service callbacks -----------------------------------------------------------
    bool stopServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::Stop::Request> req,
        std::shared_ptr<kinova_msgs::srv::Stop::Response> res);
    bool startServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::Start::Request> req,
        std::shared_ptr<kinova_msgs::srv::Start::Response> res);
    bool homeArmServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::HomeArm::Request> req,
        std::shared_ptr<kinova_msgs::srv::HomeArm::Response> res);
    bool ActivateNullSpaceModeCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::SetNullSpaceModeState::Request> req,
        std::shared_ptr<kinova_msgs::srv::SetNullSpaceModeState::Response> res);
    bool addCartesianPoseToTrajectory(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::AddPoseToCartesianTrajectory::Request> req,
        std::shared_ptr<kinova_msgs::srv::AddPoseToCartesianTrajectory::Response> res);
    bool clearTrajectoriesServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::ClearTrajectories::Request> req,
        std::shared_ptr<kinova_msgs::srv::ClearTrajectories::Response> res);
    bool setEndEffectorOffsetCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::SetEndEffectorOffset::Request> req,
        std::shared_ptr<kinova_msgs::srv::SetEndEffectorOffset::Response> res);

    //Torque control
    bool setForceControlParamsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::SetForceControlParams::Request> req,
        std::shared_ptr<kinova_msgs::srv::SetForceControlParams::Response> res);
    bool startForceControlCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::Start::Request> req,
        std::shared_ptr<kinova_msgs::srv::Start::Response> res);
    bool stopForceControlCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::Stop::Request> req,
        std::shared_ptr<kinova_msgs::srv::Stop::Response> res);
    bool setTorqueControlModeService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::SetTorqueControlMode::Request> req,
        std::shared_ptr<kinova_msgs::srv::SetTorqueControlMode::Response> res);
    bool setTorqueControlParametersService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::SetTorqueControlParameters::Request> req,
        std::shared_ptr<kinova_msgs::srv::SetTorqueControlParameters::Response> res);
    bool setJointTorquesToZeroService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::ZeroTorques::Request> req,
        std::shared_ptr<kinova_msgs::srv::ZeroTorques::Response> res);
    bool runCOMParameterEstimationService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<kinova_msgs::srv::RunCOMParametersEstimation::Request> req,
        std::shared_ptr<kinova_msgs::srv::RunCOMParametersEstimation::Response> res);

 private:
    void positionTimer();
    void cartesianVelocityTimer();
    void jointVelocityTimer();
    void statusTimer();

    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishToolWrench(void);
    void publishFingerPosition(void);

    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<rclcpp::Node> node_handle_;
    KinovaComm &kinova_comm_;

    // Publishers, subscribers, services
    rclcpp::Subscription<kinova_msgs::msg::JointVelocity>::SharedPtr joint_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs::msg::PoseVelocity>::SharedPtr cartesian_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs::msg::PoseVelocityWithFingers>::SharedPtr cartesian_velocity_with_fingers_subscriber_;
    rclcpp::Subscription<kinova_msgs::msg::PoseVelocityWithFingerVelocity>::SharedPtr cartesian_velocity_with_finger_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs::msg::JointTorque>::SharedPtr joint_torque_subscriber_;
    rclcpp::Subscription<kinova_msgs::msg::CartesianForce>::SharedPtr cartesian_force_subscriber_;

    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_angles_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_position_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_torque_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_wrench_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::FingerPosition>::SharedPtr finger_position_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    rclcpp::Publisher<kinova_msgs::msg::JointAngles>::SharedPtr joint_command_publisher_;
    rclcpp::Publisher<kinova_msgs::msg::KinovaPose>::SharedPtr cartesian_command_publisher_;

    rclcpp::Service<kinova_msgs::srv::Stop>::SharedPtr stop_service_;
    rclcpp::Service<kinova_msgs::srv::Start>::SharedPtr start_service_;
    rclcpp::Service<kinova_msgs::srv::HomeArm>::SharedPtr homing_service_;
    rclcpp::Service<kinova_msgs::srv::SetNullSpaceModeState>::SharedPtr start_null_space_service_;
    rclcpp::Service<kinova_msgs::srv::AddPoseToCartesianTrajectory>::SharedPtr add_trajectory_;
    rclcpp::Service<kinova_msgs::srv::ClearTrajectories>::SharedPtr clear_trajectories_;

    rclcpp::Service<kinova_msgs::srv::SetTorqueControlMode>::SharedPtr set_torque_control_mode_service_;
    rclcpp::Service<kinova_msgs::srv::SetTorqueControlParameters>::SharedPtr set_torque_control_parameters_service_;
    rclcpp::Service<kinova_msgs::srv::ZeroTorques>::SharedPtr set_actuator_torques_to_zero_;
    rclcpp::Service<kinova_msgs::srv::SetForceControlParams>::SharedPtr set_force_control_params_service_;
    rclcpp::Service<kinova_msgs::srv::Start>::SharedPtr start_force_control_service_;
    rclcpp::Service<kinova_msgs::srv::Stop>::SharedPtr stop_force_control_service_;
    rclcpp::Service<kinova_msgs::srv::RunCOMParametersEstimation>::SharedPtr run_COM_parameter_estimation_service_;

    rclcpp::Service<kinova_msgs::srv::SetEndEffectorOffset>::SharedPtr set_end_effector_offset_service_;

    // Timers for control loops
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Parameters
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    std::string tf_prefix_;

    char robot_category_;
    int robot_category_version_;
    char wrist_type_;
    int arm_joint_number_;
    char robot_mode_;
    int finger_number_;
    int joint_total_number_;
    ROBOT_TYPE robot_type_;


    double status_interval_seconds_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    AngularInfo joint_velocities_;
    float l_joint_torque_[COMMAND_SIZE];
    float l_force_cmd_[COMMAND_SIZE];
    CartesianInfo cartesian_velocities_;

    std::vector< std::string > joint_names_;

    //multiple robots
    int active_robot_id_;
    std::vector<robot_info> robots_;

};


}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_H
