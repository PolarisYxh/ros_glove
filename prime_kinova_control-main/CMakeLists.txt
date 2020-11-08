cmake_minimum_required(VERSION 3.0.2)
project(prime_kinova_control)

## Compile as C++11, supported in ROS Kinetic and newer

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kinova_msgs
  primeii_ros_msgs
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prime_kinova_control
#  CATKIN_DEPENDS geometry_msgs kinova_msgs primeii_ros_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
