cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME TEST_AL)
project(${PROJECT_NAME})
set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/)

#set(LIB_PATH ${ROOT_PATH}x86/)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(LIB_PATH ${ROOT_PATH}x64/)
else()
    set(LIB_PATH ${ROOT_PATH}x86/)
endif()

message(STATUS"library PATH: ${LIB_PATH}")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fPIC -std=c++11")

include_directories(${ROOT_PATH}/include)

add_executable(sample_capture_frames sample_capture_frames.c)

find_library(LIBDMCAM dmcam PATH ${LIB_PATH})

target_link_libraries(sample_capture_frames ${LIBDMCAM})

add_custom_command(TARGET sample_capture_frames POST_BUILD      # Adds a post-build event
    COMMAND ${CMAKE_COMMAND} -E copy_if_different     # which executes "cmake - E copy_if_different..."
	    "${LIB_PATH}libdmcam.dll"        # <--this is in-file
		$<TARGET_FILE_DIR:sample_capture_frames>      # <--this is out-file path
	COMMAND ${CMAKE_COMMAND} -E copy_if_different     # which executes "cmake - E copy_if_different..."
	    "${LIB_PATH}libusb-1.0.dll"       # <--this is in-file
		$<TARGET_FILE_DIR:sample_capture_frames>)     # <--this is out-file path



