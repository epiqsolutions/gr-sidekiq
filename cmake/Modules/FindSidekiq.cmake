# - Try to find Sidekiq
# Once done this will define
#  Sidekiq_FOUND - System has Sidekiq
#  Sidekiq_LIBRARIES - The Sidekiq imported target
#  Sidekiq_INCLUDE_DIRS - The Sidekiq include directories
#  Sidekiq_PKG_LIBRARY_DIRS - The Sidekiq support library directory
#  Sidekiq_BUILD_CONFIG - The Sidekiq SDK build config selected by sidekiq-config

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
    endif()

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Sidekiq
        REQUIRED_VARS
            Sidekiq_CONFIG_EXECUTABLE
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
