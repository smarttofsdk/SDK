cmake_minimum_required (VERSION 2.8)

project (dmcam_cpp_samples)

# - add modules path -
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# - find DMCAM lib -
find_package(DMCAM REQUIRED)
message(STATUS "DMCAM inc: ${DMCAM_INCLUDE_DIRS}")
message(STATUS "DMCAM lib: ${DMCAM_LIBRARIES}")

if (NOT WIN32)
    # - under linux find pthread and libusb
    find_package(PkgConfig)
    find_package(Threads)
    find_package(LibUSB REQUIRED)

    # - set rpath -
    set(CMAKE_INSTALL_RPATH "'\$ORIGIN'")
endif()

# - set include -
include_directories(${PROJECT_BINARY_DIR}
    ${DMCAM_INCLUDE_DIRS}
    )

# - each sample_*.c is an executable 
file(GLOB C_SAMPLES sample_*.cpp)
foreach(S ${C_SAMPLES})
    string(REGEX REPLACE "\\.[^.]*$" "" S_FILENAME ${S})
    get_filename_component(S_FILENAME ${S_FILENAME} NAME)

    message(STATUS ${S_FILENAME})
    add_executable(${S_FILENAME} ${S})
    if (MSVC)
        target_compile_options(${S_FILENAME} PUBLIC /W3)
    else()
        target_compile_options(${S_FILENAME} PUBLIC -Wall)
    endif()
    #set_target_properties(${S_FILENAME} PROPERTIES  LINK_FLAGS "${static_link_flags}")
    target_link_libraries(${S_FILENAME} ${DMCAM_LIBRARIES} ${LibUSB_LIBRARIES})
endforeach(S)

