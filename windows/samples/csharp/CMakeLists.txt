cmake_minimum_required(VERSION 2.8)
project(dmcam_cs_samples)

# - calc platfrom is 32bit or 64bit -
if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i686")
    set(ENV_ARCH "32") 
else() 
    set(ENV_ARCH "64") 
endif()

if (WIN32)
    if (${ENV_ARCH} EQUAL 64) 
        set(CSHARP_PLATFORM x64)
    else()
        set(CSHARP_PLATFORM x86)
    endif()
    SET(CMAKE_FIND_LIBRARY_PREFIXES "")
    SET(CMAKE_FIND_LIBRARY_SUFFIXES ".dll" ".lib")
    SET(CSHARP_DOTNET_VERSION v4.0.30319 CACHE STRING "CSharp dotnet version" FORCE)
else()
    SET(CMAKE_FIND_LIBRARY_PREFIXES "")
    SET(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a" ".dll")
endif()

# - set csharp sdk library -
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )
find_package( CSharp REQUIRED )
include( ${CSHARP_USE_FILE} )

# - CSHARP_BINARY_DIRECTORY must be set - 
set(CSHARP_BINARY_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

find_library(DMCAM_LIB REQUIRED
    NAMES libdmcam PATHS 
        "../../lib/win${ENV_ARCH}/"
        "../../lib/linux${ENV_ARCH}/"
    )
find_library(DMCAM_CS_LIB REQUIRED
    NAMES dmcam_csharp PATHS "../../csharp/${ENV_ARCH}bit/")
find_library(DMCAM_CS_ADAPTER_LIB REQUIRED
    NAMES dmcam_csharp_adapter libdmcam_csharp_adapter PATHS "../../csharp/${ENV_ARCH}bit/")

csharp_add_executable (sample_basic 
    ${CMAKE_CURRENT_SOURCE_DIR}/sampleBasic.cs 
    ${DMCAM_CS_LIB})

add_custom_command(TARGET sample_basic POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DMCAM_LIB} ${CMAKE_CURRENT_BINARY_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DMCAM_CS_LIB} ${CMAKE_CURRENT_BINARY_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DMCAM_CS_ADAPTER_LIB} ${CMAKE_CURRENT_BINARY_DIR}/
    )

    #${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/sampleBasicForm.cs 
    #${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/sampleBasicForm.Designer.cs 

file(GLOB SAMPLE_UI_SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/*.cs
    ${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/Property/*.cs
    )

#csharp_add_library(sample_ui_lib
    #${DMCAM_CS_LIB}
    #${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/sampleBasicForm.cs;${CMAKE_CURRENT_SOURCE_DIR}/sampleBasicUi/sampleBasicForm.Designer.cs
    #)

csharp_add_executable (sample_ui 
    ${SAMPLE_UI_SRCS}
    System.Windows.Forms.dll
    System.Drawing.dll
    ${DMCAM_CS_LIB}
    )

