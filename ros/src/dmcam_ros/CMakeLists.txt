cmake_minimum_required(VERSION 2.8.3)
project(dmcam_ros)

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

# - add modules path -
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# - find DMCAM lib -
if(NOT DMCAM_SDK_DIR)
set (DMCAM_SDK_DIR "../../../")
endif()

find_package(OpenCV REQUIRED)
find_package(DMCAM REQUIRED)
message(STATUS "DMCAM inc: ${DMCAM_INCLUDE_DIRS}")
message(STATUS "DMCAM lib: ${DMCAM_LIBRARIES}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
find_package(PCL REQUIRED COMPONENTS common io filters)

################################################
## Declare ROS messages, services and actions ##
################################################

 add_service_files(
   FILES
   change_intg.srv
   change_power.srv
   change_frame_rate.srv
   change_mod_freq.srv
   change_sync_delay.srv
   change_filter.srv
   disable_filter.srv
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
  ${DMCAM_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIR}/../../../bin/linux/include
  #${PROJECT_SOURCE_DIR}/../../../../../bin/linux/include
)
link_directories(
  ${DMCAM_LIBRARIES} 
  #${PROJECT_SOURCE_DIR}/../../../bin/linux/lib
  #${PROJECT_SOURCE_DIR}/../../../../../bin/linux/lib
)

add_executable(dmcam_ros_main
               src/dmcam_ros_main.cpp
               src/tofhandle.cpp
               include/function.cpp
)

set_target_properties(  dmcam_ros_main
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)

# include dmcam 
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
include_directories(dmcam_ros_main   ${DMCAM_INCLUDE_DIRS})
target_link_libraries(dmcam_ros_main
  pthread
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${DMCAM_LIBRARIES}
)

