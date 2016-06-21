cmake_minimum_required(VERSION 2.8.3)
project(bgumodo_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  roslint
  roslib
  image_transport
  pcl_conversions
  pcl_ros
  sensor_msgs
)


add_service_files(FILES
  GetDistance.srv
  GetXY.srv
  )


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 
add_executable(get_pcl src/get_pcl.cpp)
target_link_libraries(get_pcl ${catkin_LIBRARIES})
