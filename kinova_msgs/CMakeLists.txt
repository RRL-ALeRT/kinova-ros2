cmake_minimum_required(VERSION 3.5)
project(kinova_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerPosition.msg"
  "msg/JointAngles.msg"
  "msg/JointVelocity.msg"
  "msg/JointTorque.msg"
  "msg/KinovaPose.msg"
  "msg/PoseVelocity.msg"
  "msg/PoseVelocityWithFingers.msg"
  "msg/PoseVelocityWithFingerVelocity.msg"
  "msg/CartesianForce.msg"
  "srv/Start.srv"
  "srv/Stop.srv"
  "srv/HomeArm.srv"
  "srv/SetForceControlParams.srv"
  "srv/SetEndEffectorOffset.srv"
  "srv/SetNullSpaceModeState.srv"
  "srv/SetTorqueControlMode.srv"
  "srv/SetTorqueControlParameters.srv"
  "srv/ClearTrajectories.srv"
  "srv/ZeroTorques.srv"
  "srv/AddPoseToCartesianTrajectory.srv"
  "srv/RunCOMParametersEstimation.srv"
  "action/ArmJointAngles.action"
  "action/ArmKinovaPose.action"
  "action/ArmPose.action"
  "action/SetFingersPosition.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
