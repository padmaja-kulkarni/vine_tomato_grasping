cmake_minimum_required(VERSION 2.8.3)
project(flex_grasp)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Tomato.msg
  Truss.msg
  Peduncle.msg
  ImageProcessingSettings.msg
  FlexGraspErrorCodes.msg
  DynamixelErrorCodes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
