cmake_minimum_required(VERSION 2.8.3)
project(bgumodo_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  robotican_common
  cv_bridge
  pcl_conversions
  pcl_ros
  tf
)

 find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotican_demos
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(coffe_demo_nav src/coffe_demo_nav.cpp)
target_link_libraries(coffe_demo_nav ${catkin_LIBRARIES})

