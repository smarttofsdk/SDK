# - Try to find DMCAM
# Once done this will define
# DMCAM_FOUND - System has DMCAM
# DMCAM_INCLUDE_DIRS - The DMCAM include directories
# DMCAM_LIBRARIES - The libraries needed to use DMCAM
# DMCAM_STATIC_LIBRARIES - The static libraries needed to use DMCAM
# DMCAM_DEFINITIONS - Compiler switches required for using DMCAM

if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i686")
    set(ENV_ARCH "32") 
else() 
    set(ENV_ARCH "64") 
endif()

find_path ( DMCAM_INCLUDE_DIR names dmcam.h PATHS
    ${DMCAM_SDK_DIR}/Windows/include
    ${DMCAM_SDK_DIR}/Linux/include
    ../../include
    ../src
    ../../libdmcam/src
    )
if (WIN32)
    find_library ( DMCAM_LIBRARY NAMES dmcam libdmcam PATHS
        ${DMCAM_SDK_DIR}/Windows/lib/win${ENV_ARCH}
        ../../lib/win${ENV_ARCH}
        ../bin/Windows/lib/win${ENV_ARCH}
        ../../bin/Windows/lib/win${ENV_ARCH}
        )

    find_library ( DMCAM_STATIC_LIBRARY NAMES libdmcam.a PATHS
        ${DMCAM_SDK_DIR}/Windows/lib/win${ENV_ARCH}
        ../../lib/win${ENV_ARCH}
        ../bin/Windows/lib/win${ENV_ARCH}
        ../../bin/Windows/lib/win${ENV_ARCH}
        )

    get_filename_component(DMCAM_LIBDIR ${DMCAM_LIBRARY} DIRECTORY)
    find_file(DMCAM_DLL
        libdmcam.dll 
        PATHS
        ${DMCAM_LIBDIR}/
        ${DMCAM_LIBDIR}/../
        )
    file(COPY ${DMCAM_DLL} DESTINATION ${CMAKE_BINARY_DIR})
else()
    find_library ( DMCAM_LIBRARY NAMES dmcam PATHS
        ${DMCAM_SDK_DIR}/Linux/lib/linux${ENV_ARCH}
        ../../lib/linux${ENV_ARCH}
        ../bin/Linux/lib/linux${ENV_ARCH}
        ../../bin/Linux/lib/linux${ENV_ARCH}
        )

    find_library ( DMCAM_STATIC_LIBRARY NAMES libdmcam.a PATHS
        ${DMCAM_SDK_DIR}/Linux/lib/linux${ENV_ARCH}
        ../../lib/linux${ENV_ARCH}
        ../bin/Linux/lib/linux${ENV_ARCH}
        ../../bin/Linux/lib/linux${ENV_ARCH}
        )
endif()


set ( DMCAM_LIBRARIES ${DMCAM_LIBRARY} )
set ( DMCAM_STATIC_LIBRARIES ${DMCAM_STATIC_LIBRARY} )
set ( DMCAM_INCLUDE_DIRS ${DMCAM_INCLUDE_DIR} )

include ( FindPackageHandleStandardArgs )
# handle the QUIETLY and REQUIRED arguments and set DMCAM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args ( DMCAM DEFAULT_MSG DMCAM_LIBRARY DMCAM_INCLUDE_DIR )
