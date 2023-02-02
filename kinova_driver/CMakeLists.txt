cmake_minimum_required(VERSION 3.5)
project(kinova_driver)

set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kinova_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# generate_dynamic_reconfigure_options(
#   #cfg/KinovaPositionConfig.cfg
# )

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

###########
## Build ##
###########

include_directories(
  include
)

#Testers
#add_executable(test_kinova_arm_control src/testers/test_kinova_arm_controller.cpp )
#add_dependencies(test_kinova_arm_control ${PROJECT_NAME}_gencfg)
#add_dependencies(test_kinova_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_executable(test_kinova_arm_vel src/testers/test_kinova_arm_vel.cpp )
#add_executable(test_kinova_arm_car_vel src/testers/test_kinova_arm_car_vel.cpp )

# Nodes
# Note, the headers are shown here so they show up in the project view when using
# Qt Creator. There must be a better way...

add_library(kinova_driver
  src/kinova_api.cpp
  src/kinova_arm.cpp
  src/kinova_comm.cpp
  src/kinova_tool_pose_action.cpp
  src/kinova_joint_angles_action.cpp
  src/kinova_fingers_action.cpp
  src/kinova_ros_types.cpp
  src/kinova_joint_trajectory_controller.cpp
  include/kinova_driver/kinova_joint_trajectory_controller.h
  include/kinova_driver/kinova_joint_angles_action.h
  include/kinova_driver/kinova_api.h
  include/kinova_driver/kinova_arm.h
  include/kinova_driver/kinova_comm.h
  include/kinova_driver/kinova_fingers_action.h
  include/kinova_driver/kinova_tool_pose_action.h
  include/kinova_driver/kinova_ros_types.h
  include/kinova/Kinova.API.USBCommLayerUbuntu.h
  include/kinova/Kinova.API.USBCommandLayerUbuntu.h
  include/kinova/KinovaTypes.h
)
ament_target_dependencies(kinova_driver rclcpp rclcpp_action geometry_msgs tf2 tf2_ros sensor_msgs kinova_msgs angles control_msgs builtin_interfaces)

add_executable(kinova_arm_driver src/nodes/kinova_arm_driver.cpp
)
target_link_libraries(kinova_arm_driver kinova_driver)
ament_target_dependencies(kinova_arm_driver rclcpp rclcpp_action geometry_msgs tf2 tf2_ros sensor_msgs kinova_msgs angles control_msgs builtin_interfaces)

add_executable(joint_trajectory_action_server 
  src/joint_trajectory_action/joint_trajectory_action_server.cpp
  include/kinova_driver/joint_trajectory_action_server.h)
target_link_libraries(joint_trajectory_action_server kinova_driver)

add_executable(gripper_command_action_server 
  src/joint_trajectory_action/gripper_command_action_server.cpp 
  include/kinova_driver/gripper_command_action_server.h)
target_link_libraries(gripper_command_action_server kinova_driver)

add_executable(positional_action_server 
  src/joint_trajectory_action/positional_action_server.cpp)
target_link_libraries(positional_action_server kinova_driver)

add_executable(kinova_tf_updater
  src/kinova_ros_types.cpp
  src/nodes/kinova_tf_updater.cpp
  src/kinova_arm_kinematics.cpp
  include/kinova_driver/kinova_ros_types.h
  include/kinova_driver/kinova_tf_updater.h
  include/kinova_driver/kinova_arm_kinematics.h
)
ament_target_dependencies(kinova_tf_updater rclcpp angles geometry_msgs tf2 tf2_ros kinova_msgs)
# target_link_libraries(kinova_driver dl)
# #target_link_libraries(kinova_arm_driver dl)
# target_link_libraries(kinova_arm_driver kinova_driver)

add_executable(kinova_interactive_control src/nodes/kinova_interactive_control.cpp)
target_link_libraries(kinova_interactive_control kinova_driver)
ament_target_dependencies(kinova_interactive_control interactive_markers)
###---
#############
## Install ##
#############

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Mark executables and/or libraries for installation
install(TARGETS kinova_arm_driver kinova_tf_updater
  kinova_interactive_control kinova_driver
  joint_trajectory_action_server gripper_command_action_server
  positional_action_server
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#  install(FILES
## myfile1
## myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinova_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)

ament_package()
