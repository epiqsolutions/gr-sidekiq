# - Try to find Sidekiq
# Once done this will define
#  Sidekiq_FOUND - System has Sidekiq
#  Sidekiq_LIBRARIES - The Sidekiq libraries
#  Sidekiq_INCLUDE_DIRS - The Sidekiq include directories
#  Sidekiq_LIB_DIRS - The Sidekiq library directories

if(NOT Sidekiq_FOUND)

    find_path(Sidekiq_INCLUDE_DIR
            NAMES sidekiq_api.h
            HINTS ${Sidekiq_PKG_INCLUDE_DIRS} $ENV{Sidekiq_DIR}/include
            PATHS ~/sidekiq_sdk_current/sidekiq_core/inc/ /usr/local/include /usr/include /opt/include /opt/local/include)

    execute_process (
        COMMAND uname -m
        OUTPUT_VARIABLE cpu_arch
    )

    message(STATUS "cpu_arch ${cpu_arch}")

    if (NOT ${cpu_arch} MATCHES "x86_64")
        set(SDK_DIR "$ENV{HOME}/sidekiq_sdk_current/lib")
        file(GLOB LIB_FILES "${SDK_DIR}/libsidekiq__*.a")

        if(LIB_FILES)
            list(GET LIB_FILES 0 FOUND_LIB)
            get_filename_component(LIB_NAME "${FOUND_LIB}" NAME)
            string(REPLACE "libsidekiq__" "" SUFFIX_WITH_EXT "${LIB_NAME}")
            string(REPLACE ".a" "" SUFFIX "${SUFFIX_WITH_EXT}")
            message(STATUS "Detected SDK SUFFIX: ${SUFFIX}")

        else()
            message(FATAL_ERROR "No libsidekiq__*.a file found in ${SDK_DIR}")
        endif()
    endif()

    if(${cpu_arch} MATCHES "x86_64")
        set (libname  "libsidekiq__x86_64.gcc.a")
        set (otherlib "none")
    elseif(${SUFFIX} MATCHES "msiq-x40")
        set(otherlib "none")
        set(libname  "libsidekiq__msiq-x40.a")
    elseif(${SUFFIX} MATCHES "msiq-g20g40")
        set(otherlib "none")
        set(libname  "libsidekiq__msiq-g20g40.a")
    elseif(${SUFFIX} MATCHES "z3u")
        set(otherlib "libiio")
        set(libname  "libsidekiq__z3u.a")
    elseif(${SUFFIX} MATCHES "aarch64")
        set (libname  "libsidekiq__aarch64.a")
        set (otherlib "libiio")
    elseif(${SUFFIX} MATCHES "aarch64.gcc6.3")
        set (libname  "libsidekiq__aarch64.gcc6.3.a")
        set (otherlib "libiio")
    elseif(${SUFFIX} MATCHES "arm_cortex-a9.gcc7.2.1_gnueabihf")
        set (libname  "libsidekiq__arm_cortex-a9.gcc7.2.1_gnueabihf.a")
        set (otherlib "libiio")
    else()
      message(FATAL_ERROR "Invalid platform ${SUFFIX}")
    endif()

    message(STATUS "library is ${libname} ")
    message(STATUS "otherlib is ${otherlib} ")

    find_library(Sidekiq_LIBRARY
        NAMES ${libname}
        HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
        PATHS ~/sidekiq_sdk_current/lib/)


    #    find_library(Sidekiq_LIBRARY
    #    NAMES ${libname}
    #    HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
    #    PATHS ~/sidekiq_sw)

    set(Sidekiq_LIBRARIES ${Sidekiq_LIBRARY})
    set(Sidekiq_INCLUDE_DIRS ${Sidekiq_INCLUDE_DIR})


    if(${cpu_arch} MATCHES "x86_64")
        message(STATUS "building for x86_64.gcc")
        include(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
            Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR )

        set(OTHER_LIBS "")
        set(PKGCONFIG_LIBS "")
        mark_as_advanced(Sidekiq_INCLUDE_DIRS Sidekiq_LIBRARIES OTHER_LIBS PKGCONFIG_LIBS) 
    elseif(SUFFIX MATCHES "^(z3u|aarch64|aarch64\\.gcc6\\.3|arm_cortex-a9\\.gcc7\\.2\\.1_gnueabihf)$")
        message(STATUS "building for aarch")
        find_library(OTHER_LIBS
            NAMES ${otherlib}
            HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
            PATHS /usr/lib/epiq/ /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

        set(OTHER_LIBS ${OTHER_LIBS})

        include(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
            Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR OTHER_LIBS)

        set(PKGCONFIG_LIBS "")

        mark_as_advanced(Sidekiq_INCLUDE_DIRS Sidekiq_LIBRARIES OTHER_LIBS PKGCONFIG_LIBS) 
    elseif(${SUFFIX} MATCHES "msiq-x40")
        message(STATUS "building for x40")

        # Get home directory
        get_filename_component(HOME_DIR "$ENV{HOME}" ABSOLUTE)

        # Set the PKG_CONFIG_PATH using the home directory
        set(ENV{PKG_CONFIG_PATH} "${HOME_DIR}/sidekiq_sdk_current/lib/support/msiq-x40/usr/lib/epiq/pkgconfig")

        message(STATUS "PKG_CONFIG_PATH $ENV{PKG_CONFIG_PATH}")

        execute_process(
            COMMAND pkg-config --libs-only-l grpc++ protobuf
            OUTPUT_VARIABLE PKG_LIBS
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        # Convert PKG_LIBS into a list
        string(REPLACE " " ";" PKG_LIBS_LIST ${PKG_LIBS})

        set (PKGCONFIG_LIBS ${PKG_LIBS_LIST} -lgpiod -lstdc++)
        message(STATUS "PKGCONFIG ${PKGCONFIG_LIBS}")

        execute_process(
            COMMAND pkg-config --variable=libdir protobuf
            OUTPUT_VARIABLE LIB_PATH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        message(STATUS "LIB_PATH ${LIB_PATH}")
        link_directories(${LIB_PATH})

        include(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
            Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR )

        set(OTHER_LIBS "")

        mark_as_advanced(Sidekiq_INCLUDE_DIRS Sidekiq_LIBRARIES OTHER_LIBS PKGCONFIG_LIBS) 
    else()
      message(STATUS "building for ${SUFFIX}")
        include(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
            Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR )

        set(OTHER_LIBS "")
        set(PKGCONFIG_LIBS "")
        mark_as_advanced(Sidekiq_INCLUDE_DIRS Sidekiq_LIBRARIES OTHER_LIBS PKGCONFIG_LIBS) 
    endif()

endif(NOT Sidekiq_FOUND)
