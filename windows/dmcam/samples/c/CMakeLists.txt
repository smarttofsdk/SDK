cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME TEST_AL)
project(${PROJECT_NAME})
set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/)
set(LIB_PATH ${ROOT_PATH}x86/)
message(STATUS"library PATH: ${LIB_PATH}")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fPIC -std=c++11")

include_directories(${ROOT_PATH}/include)

add_executable(sample_capture_frames sample_capture_frames.c)

find_library(LIBDMCAM dmcam PATH ${LIB_PATH})

target_link_libraries(sample_capture_frames ${LIBDMCAM})



