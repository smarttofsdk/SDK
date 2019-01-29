cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)
project (smarttof)

# - add module path
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


# find DMCAM LIB
if(NOT DMCAM_SDK_DIR)
    set(DMCAM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../Windows)
endif()

# find OpenNI2
find_package(OpenNI2 REQUIRED)

if (NOT OpenNI2_FOUND)
    message(WARNING "OpenNI2Driver smarttof dll will not be build as no OpenNI2 was found!")
    return()
endif()

find_package(DMCAM REQUIRED)
message(STATUS "DMCAM inc: ${DMCAM_INCLUDE_DIRS}")
message(STATUS "DMCAM lib: ${DMCAM_LIBRARIES}")
message(STATUS "DMCAM lib(static): ${DMCAM_STATIC_LIBRARIES}")

message(STATUS "INSTALL_TO: ${CMAKE_INSTALL_PREFIX}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR/../../../../Windows/inc}
  )

#include_directories($ENV{OPENNI2_INCLUDE64})
#include_directories($ENV{OPENNI2_INCLUDE64})

include_directories(
    SYSTEM
    ${OPENNI2_INCLUDE_DIR}
)

# ---Ôö¼Ócompile flags---
if(WIN32)
    set(generic_link_libs m advapi32 iphlpapi psapi  userenv shell32 ws2_32)
    if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        set(static_link_flags -Wl,-Bstatic)
        set(generic_c_flags -g -Wall -D__USE_MINGW_ANSI_STDIO=1)
        list(APPEND generic_link_libs -static-libgcc)

        set (CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} -s")
        set (CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -s")
        set (CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} -s")
    else()
        set (generic_c_flags /Wall)
    endif()
    message(STATUS "${CMAKE_CXX_COMPILER_ID}")
else()
    if (PROFILE_ENABLE)
        set(profile_flags -pg)
    endif()
    set (generic_c_flags -g -Wall -fPIC ${profile_flags})
    set (generic_link_libs m rt)
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread ${profile_flags}")
    set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pthread")
    set (CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")
    set (CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -s")
    set (CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} -s")
endif(WIN32)

# --- source files ---
file(GLOB CSRCS
    src/*.cpp
    src/*.h
  )


# share lib
add_library(smarttof_objlib OBJECT ${CSRCS})
target_compile_options(smarttof_objlib PUBLIC ${generic_c_flags} $<$<COMPILE_LANGUAGE:CXX>:--std=c++11>)
add_compile_options(-m64)

add_library(smarttof SHARED $<TARGET_OBJECTS:smarttof_objlib>)
set_target_properties(smarttof PROPERTIES OUTPUT_NAME "smarttof")

add_library(smarttof_static STATIC $<TARGET_OBJECTS:smarttof_objlib>)
set_target_properties(smarttof_static PROPERTIES OUTPUT_NAME "smarttof")

# target setting
set_target_properties(smarttof PROPERTIES LINK_FLAGS "${static_link_flags}")

target_link_libraries(smarttof ${DMCAM_LIBRARIES} ${OPENNI2_LIBRARY} ${generic_link_libs})

add_custom_target(clib
    DEPENDS smarttof smarttof_static
  )


# -----install-----
# install(TARGETS smarttof smarttof_static
    # RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/samples/openni2
    # LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/samples/openni2
    # ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/samples/openni2
    # )
