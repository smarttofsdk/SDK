cmake_minimum_required(VERSION 2.8.3)
project(tof_sample)

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

include_directories(
  include
  ../../devel/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tof_sample
    src/tof_sample.cpp
)
set_target_properties(tof_sample
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
target_link_libraries(tof_sample
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

