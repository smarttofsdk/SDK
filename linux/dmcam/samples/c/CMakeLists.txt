cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME TEST_AL)
project(${PROJECT_NAME})
set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/)
set(LIB_DMCAM_PATH ${ROOT_PATH}x64/)
set(LIB_PATH_PTHREAD /usr/lib/x86_64-linux-gnu/)

set(TARGET sample_capture_frames )
set(TEST_SRC sample_capture_frames.c)

message(STATUS"library PATH: ${LIB_DMCAM_PATH}")
message(STATUS"library PATH: ${LIB_PATH_PTHREAD}")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fPIC -std=c++11")

include_directories(${ROOT_PATH}/include)
link_directories(${LIB_DMCAM_PATH} ${LIB_PATH_PTHREAD})

#add_executable(sample_capture_frames sample_capture_frames.c)
add_executable(${TARGET} ${TEST_SRC})

find_library(LIBDMCAM dmcam PATH ${LIB_DMCAM_PATH})
find_library(LIBPHREAD pthread PATH ${LIB_PATH_PTHREAD})

target_link_libraries(sample_capture_frames ${LIBDMCAM} ${LIBPHREAD})



