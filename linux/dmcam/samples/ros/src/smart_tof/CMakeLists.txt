cmake_minimum_required(VERSION 2.8.3)
project(smart_tof)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

################################################
## Declare ROS messages, services and actions ##
################################################

 add_service_files(
   FILES
   change_intg.srv
   change_power.srv
 )

 generate_messages(
   DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  lib
)

add_executable(smart_tof_main
               src/smart_tof_main.cpp
               src/tofhandle.cpp
               include/function.cpp
)

set_target_properties(  smart_tof_main
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)

# include dmcam 
include_directories(smart_tof_main ${PROJECT_SOURCE_DIR}/../../../../bin/include)
target_link_libraries(smart_tof_main
  dmcam
  pthread
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

