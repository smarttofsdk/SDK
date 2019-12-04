# - Try to find DMCAM
# DMCAM_SDK_DIR should be defined to search libdmcam
#
# Once done this will define
# DMCAM_FOUND - System has DMCAM
# DMCAM_INCLUDE_DIRS - The DMCAM include directories
# DMCAM_LIBRARIES - The libraries needed to use DMCAM
# DMCAM_STATIC_LIBRARIES - The static libraries needed to use DMCAM
# DMCAM_DEFINITIONS - Compiler switches required for using DMCAM

cmake_policy(SET CMP0054 NEW)
if(NOT DMCAM_SDK_DIR)
    message(WARNING "DMCAM_SDK_DIR must be set")
    set (DMCAM_SDK_DIR "NOT-FOUND" CACHE PATH "DMCAM SDK home dir")
endif()

if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i686")
    set(ENV_ARCH "32") 
else() 
    set(ENV_ARCH "64") 
endif()

find_path ( DMCAM_INCLUDE_DIR names dmcam.h PATHS
    ${DMCAM_SDK_DIR}/include
    ${DMCAM_SDK_DIR}/windows/include
    ${DMCAM_SDK_DIR}/linux/include
    )
if (WIN32)
    find_library ( DMCAM_LIBRARY NAMES dmcam libdmcam PATHS
        ${DMCAM_SDK_DIR}/windows/lib/win${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/win${ENV_ARCH}
        )

    get_filename_component(DMCAM_LIBDIR ${DMCAM_LIBRARY} DIRECTORY)
    find_file(DMCAM_DLL
        libdmcam.dll 
        PATHS
        ${DMCAM_LIBDIR}/
        ${DMCAM_LIBDIR}/../
        )
    file(COPY ${DMCAM_DLL} DESTINATION ${CMAKE_BINARY_DIR})

    find_file(DMCAM_STATIC_LIBRARY NAMES libdmcam.a PATHS
        ${DMCAM_SDK_DIR}/windows/lib/win${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/win${ENV_ARCH}
        )

    if("${CMAKE_C_COMPILER_ID}" MATCHES "GNU" OR "${CMAKE_C_COMPILER_ID}" MATCHES "Clang")
       list(APPEND DMCAM_STATIC_LIBRARY -static-libgcc)
    endif()

    find_library ( DMCAM_OPENH264_STATIC_LIBRARY NAMES libopenh264.a PATHS
        ${DMCAM_SDK_DIR}/windows/lib/win${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/win${ENV_ARCH}
        )
    if (DMCAM_OPENH264_STATIC_LIBRARY)
        list(APPEND DMCAM_STATIC_LIBRARY ${DMCAM_OPENH264_STATIC_LIBRARY})
    endif()

    list(APPEND DMCAM_STATIC_LIBRARY uv advapi32 iphlpapi psapi userenv shell32 ws2_32)
else()
    find_library ( DMCAM_LIBRARY NAMES dmcam PATHS
        ${DMCAM_SDK_DIR}/linux/lib/linux${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/linux${ENV_ARCH}
        )

    find_library ( DMCAM_STATIC_LIBRARY NAMES libdmcam.a PATHS
        ${DMCAM_SDK_DIR}/linux/lib/linux${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/linux${ENV_ARCH}
        )

    find_library ( DMCAM_OPENH264_STATIC_LIBRARY NAMES libopenh264.a PATHS
        ${DMCAM_SDK_DIR}/linux/lib/linux${ENV_ARCH}
        ${DMCAM_SDK_DIR}/lib/linux${ENV_ARCH}
        )

    if (DMCAM_OPENH264_STATIC_LIBRARY)
        list(APPEND DMCAM_STATIC_LIBRARY ${DMCAM_OPENH264_STATIC_LIBRARY})
    endif()

    list(APPEND DMCAM_STATIC_LIBRARY uv usb-1.0 m)

    # libdl should not linked in static: exclude dl 
    if("${CMAKE_C_COMPILER_ID}" MATCHES "GNU" OR "${CMAKE_C_COMPILER_ID}" MATCHES "Clang")
      list(APPEND DMCAM_STATIC_LIBRARY -Wl,-Bdynamic dl -Wl,-Bstatic )
    endif()
endif()

set ( DMCAM_LIBRARIES ${DMCAM_LIBRARY} )
set ( DMCAM_STATIC_LIBRARIES ${DMCAM_STATIC_LIBRARY} )
set ( DMCAM_INCLUDE_DIRS ${DMCAM_INCLUDE_DIR} )

include ( FindPackageHandleStandardArgs )
# handle the QUIETLY and REQUIRED arguments and set DMCAM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args ( DMCAM DEFAULT_MSG DMCAM_LIBRARY DMCAM_INCLUDE_DIR )
