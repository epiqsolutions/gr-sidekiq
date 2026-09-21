# - Try to find Sidekiq
# Once done this will define
#  Sidekiq_FOUND - System has Sidekiq
#  Sidekiq_LIBRARIES - The Sidekiq imported target
#  Sidekiq_INCLUDE_DIRS - The Sidekiq include directories
#  Sidekiq_PKG_LIBRARY_DIRS - The Sidekiq support library directory
#  Sidekiq_BUILD_CONFIG - The Sidekiq SDK build config selected by sidekiq-config
#
# SDK 4.26 and newer provide sidekiq-config.  Older SDKs are discovered from
# their conventional include/lib layout and linked with the dependencies listed
# by the SDK's example makefiles.

if(NOT Sidekiq_FOUND)

    if(DEFINED SIDEKIQ_SDK_DIR AND NOT "${SIDEKIQ_SDK_DIR}" STREQUAL "")
        get_filename_component(_Sidekiq_SDK_DIR_HINT "${SIDEKIQ_SDK_DIR}" ABSOLUTE)
    elseif(DEFINED ENV{SIDEKIQ_SDK_DIR} AND NOT "$ENV{SIDEKIQ_SDK_DIR}" STREQUAL "")
        get_filename_component(_Sidekiq_SDK_DIR_HINT "$ENV{SIDEKIQ_SDK_DIR}" ABSOLUTE)
    else()
        get_filename_component(_Sidekiq_SDK_DIR_HINT "$ENV{HOME}/sidekiq_sdk_current" ABSOLUTE)
    endif()

    find_program(Sidekiq_CONFIG_EXECUTABLE
        NAMES sidekiq-config
        HINTS "${_Sidekiq_SDK_DIR_HINT}/bin"
        NO_DEFAULT_PATH)

    function(_sidekiq_config _out_var _flag)
        execute_process(
            COMMAND "${Sidekiq_CONFIG_EXECUTABLE}" "${_flag}"
            RESULT_VARIABLE _sidekiq_config_result
            OUTPUT_VARIABLE _sidekiq_config_output
            ERROR_VARIABLE _sidekiq_config_error
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        if(NOT _sidekiq_config_result EQUAL 0)
            message(FATAL_ERROR
                "Failed to run ${Sidekiq_CONFIG_EXECUTABLE} ${_flag}: "
                "${_sidekiq_config_error}")
        endif()

        set(${_out_var} "${_sidekiq_config_output}" PARENT_SCOPE)
    endfunction()

    if(Sidekiq_CONFIG_EXECUTABLE)
        _sidekiq_config(Sidekiq_CFLAGS "--cflags")
        _sidekiq_config(Sidekiq_LINK_FLAGS "--libs-static")
        _sidekiq_config(Sidekiq_SDK_DIR "--prefix")
        _sidekiq_config(Sidekiq_PKG_LIBRARY_DIRS "--support-dir")
        _sidekiq_config(Sidekiq_BUILD_CONFIG "--build-config")
        _sidekiq_config(Sidekiq_VERSION "--version")

        set(SIDEKIQ_SDK_DIR "${Sidekiq_SDK_DIR}" CACHE PATH "Path to the Sidekiq SDK" FORCE)

        separate_arguments(Sidekiq_CFLAGS_LIST UNIX_COMMAND "${Sidekiq_CFLAGS}")
        separate_arguments(Sidekiq_LINK_LIBRARIES UNIX_COMMAND "${Sidekiq_LINK_FLAGS}")

        set(Sidekiq_INCLUDE_DIRS "")
        set(Sidekiq_COMPILE_OPTIONS "")
        foreach(_Sidekiq_CFLAG IN LISTS Sidekiq_CFLAGS_LIST)
            if("${_Sidekiq_CFLAG}" MATCHES "^-I(.+)")
                list(APPEND Sidekiq_INCLUDE_DIRS "${CMAKE_MATCH_1}")
            else()
                list(APPEND Sidekiq_COMPILE_OPTIONS "${_Sidekiq_CFLAG}")
            endif()
        endforeach()

        set(OTHER_LIBS "")
        set(PKGCONFIG_LIBS "")
    else()
        message(STATUS
            "sidekiq-config not found in ${_Sidekiq_SDK_DIR_HINT}/bin; "
            "trying the legacy SDK layout")

        find_path(Sidekiq_INCLUDE_DIR
            NAMES sidekiq_api.h
            HINTS
                "${_Sidekiq_SDK_DIR_HINT}/sidekiq_core/inc"
                "${_Sidekiq_SDK_DIR_HINT}/include"
            NO_DEFAULT_PATH)

        if(DEFINED SIDEKIQ_BUILD_CONFIG AND NOT "${SIDEKIQ_BUILD_CONFIG}" STREQUAL "")
            set(Sidekiq_BUILD_CONFIG "${SIDEKIQ_BUILD_CONFIG}")
        elseif(DEFINED ENV{SIDEKIQ_BUILD_CONFIG} AND NOT "$ENV{SIDEKIQ_BUILD_CONFIG}" STREQUAL "")
            set(Sidekiq_BUILD_CONFIG "$ENV{SIDEKIQ_BUILD_CONFIG}")
        elseif(DEFINED SUFFIX AND NOT "${SUFFIX}" STREQUAL "" AND NOT "${SUFFIX}" STREQUAL "none")
            # Preserve the platform selector used by the original finder.
            set(Sidekiq_BUILD_CONFIG "${SUFFIX}")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
            set(Sidekiq_BUILD_CONFIG "x86_64.gcc")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
            set(Sidekiq_BUILD_CONFIG "aarch64")
        endif()

        if(Sidekiq_BUILD_CONFIG)
            set(_Sidekiq_LEGACY_LIBRARY
                "${_Sidekiq_SDK_DIR_HINT}/lib/libsidekiq__${Sidekiq_BUILD_CONFIG}.a")
            if(EXISTS "${_Sidekiq_LEGACY_LIBRARY}")
                set(Sidekiq_LIBRARY "${_Sidekiq_LEGACY_LIBRARY}")
            endif()

            set(Sidekiq_PKG_LIBRARY_DIRS
                "${_Sidekiq_SDK_DIR_HINT}/lib/support/${Sidekiq_BUILD_CONFIG}/usr/lib/epiq")
        endif()

        if(Sidekiq_INCLUDE_DIR)
            file(STRINGS "${Sidekiq_INCLUDE_DIR}/sidekiq_api.h"
                _Sidekiq_VERSION_MAJOR_LINE
                REGEX "^#define[ \t]+LIBSIDEKIQ_VERSION_MAJOR[ \t]+[0-9]+")
            file(STRINGS "${Sidekiq_INCLUDE_DIR}/sidekiq_api.h"
                _Sidekiq_VERSION_MINOR_LINE
                REGEX "^#define[ \t]+LIBSIDEKIQ_VERSION_MINOR[ \t]+[0-9]+")
            file(STRINGS "${Sidekiq_INCLUDE_DIR}/sidekiq_api.h"
                _Sidekiq_VERSION_PATCH_LINE
                REGEX "^#define[ \t]+LIBSIDEKIQ_VERSION_PATCH[ \t]+[0-9]+")

            if(_Sidekiq_VERSION_MAJOR_LINE AND
               _Sidekiq_VERSION_MINOR_LINE AND
               _Sidekiq_VERSION_PATCH_LINE)
                string(REGEX REPLACE ".*[ \t]([0-9]+).*" "\\1"
                    _Sidekiq_VERSION_MAJOR "${_Sidekiq_VERSION_MAJOR_LINE}")
                string(REGEX REPLACE ".*[ \t]([0-9]+).*" "\\1"
                    _Sidekiq_VERSION_MINOR "${_Sidekiq_VERSION_MINOR_LINE}")
                string(REGEX REPLACE ".*[ \t]([0-9]+).*" "\\1"
                    _Sidekiq_VERSION_PATCH "${_Sidekiq_VERSION_PATCH_LINE}")
                set(Sidekiq_VERSION
                    "${_Sidekiq_VERSION_MAJOR}.${_Sidekiq_VERSION_MINOR}.${_Sidekiq_VERSION_PATCH}")
            endif()
        endif()

        if(Sidekiq_LIBRARY)
            # libsidekiq is static in legacy SDKs.  Keep it first, followed by
            # the transitive libraries documented by those SDKs.
            set(Sidekiq_LINK_LIBRARIES
                "${Sidekiq_LIBRARY}"
                "-L${Sidekiq_PKG_LIBRARY_DIRS}"
                "-Wl,-rpath-link=${Sidekiq_PKG_LIBRARY_DIRS}"
                "-Wl,--enable-new-dtags"
                "-Wl,-rpath,/usr/lib/epiq")

            if(Sidekiq_BUILD_CONFIG MATCHES "^(z2-armhf|z3u|arm_cortex-a9.*|aarch64\\.gcc6\\.3)$")
                list(APPEND Sidekiq_LINK_LIBRARIES iio)
            elseif(Sidekiq_BUILD_CONFIG STREQUAL "msiq-g20g40")
                list(APPEND Sidekiq_LINK_LIBRARIES hmc704x)
            elseif(Sidekiq_BUILD_CONFIG STREQUAL "msiq-x40")
                find_program(_Sidekiq_PKG_CONFIG_EXECUTABLE pkg-config)
                if(_Sidekiq_PKG_CONFIG_EXECUTABLE)
                    execute_process(
                        COMMAND "${CMAKE_COMMAND}" -E env
                            "PKG_CONFIG_PATH=${Sidekiq_PKG_LIBRARY_DIRS}/pkgconfig"
                            "${_Sidekiq_PKG_CONFIG_EXECUTABLE}"
                            --libs-only-l grpc++ protobuf
                        RESULT_VARIABLE _Sidekiq_PKG_CONFIG_RESULT
                        OUTPUT_VARIABLE _Sidekiq_X40_LIBS
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
                    if(_Sidekiq_PKG_CONFIG_RESULT EQUAL 0)
                        separate_arguments(_Sidekiq_X40_LIBS UNIX_COMMAND "${_Sidekiq_X40_LIBS}")
                        list(APPEND Sidekiq_LINK_LIBRARIES ${_Sidekiq_X40_LIBS} gpiod)
                    endif()
                endif()
            elseif(Sidekiq_BUILD_CONFIG STREQUAL "z4")
                list(APPEND Sidekiq_LINK_LIBRARIES gpiod)
            endif()

            list(APPEND Sidekiq_LINK_LIBRARIES
                usb-1.0 glib-2.0 tirpc pthread rt m stdc++ dl)
            set(Sidekiq_INCLUDE_DIRS "${Sidekiq_INCLUDE_DIR}")
            set(SIDEKIQ_SDK_DIR "${_Sidekiq_SDK_DIR_HINT}"
                CACHE PATH "Path to the Sidekiq SDK" FORCE)
        endif()
    endif()

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Sidekiq
        REQUIRED_VARS
            Sidekiq_INCLUDE_DIRS
            Sidekiq_LINK_LIBRARIES
        VERSION_VAR Sidekiq_VERSION)

    if(Sidekiq_FOUND AND NOT TARGET Sidekiq::sidekiq)
        add_library(Sidekiq::sidekiq INTERFACE IMPORTED)
        set_target_properties(Sidekiq::sidekiq PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Sidekiq_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${Sidekiq_LINK_LIBRARIES}")
        if(Sidekiq_COMPILE_OPTIONS)
            set_target_properties(Sidekiq::sidekiq PROPERTIES
                INTERFACE_COMPILE_OPTIONS "${Sidekiq_COMPILE_OPTIONS}")
        endif()
    endif()

    set(Sidekiq_LIBRARIES Sidekiq::sidekiq)

    mark_as_advanced(
        Sidekiq_CONFIG_EXECUTABLE
        Sidekiq_INCLUDE_DIRS
        Sidekiq_LIBRARIES
        Sidekiq_LINK_LIBRARIES
        Sidekiq_PKG_LIBRARY_DIRS
        OTHER_LIBS
        PKGCONFIG_LIBS)
endif(NOT Sidekiq_FOUND)
