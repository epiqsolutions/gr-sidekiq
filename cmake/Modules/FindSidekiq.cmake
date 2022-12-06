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

    get_filename_component(sdk_name "~/sidekiq_sdk_current" ABSOLUTE)

    set(SIDEKIQ_SDK ${sdk_name})
    message(STATUS "sdk is: ${SIDEKIQ_SDK}")

    execute_process (
        COMMAND uname -m
        OUTPUT_VARIABLE outVar
    )    

    message(STATUS "type ${outVar}")

    if(${outVar} MATCHES "x86_64")
        set (libname  "libsidekiq__x86_64.gcc.a")
        set (otherlib "none")
    else()
        set(libname  "libsidekiq__aarch64.gcc6.3.a")
        set(otherlib "libiio.so")
    endif()

    message(STATUS "library is ${libname} ")
    message(STATUS "otherlib is ${otherlib} ")

    find_library(Sidekiq_LIBRARY
        NAMES ${libname}
        HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
        PATHS ~/sidekiq_sdk_current/lib/ /usr/local/lib /usr/lib /opt/lib /opt/local/lib)


    set(Sidekiq_LIBRARIES ${Sidekiq_LIBRARY})
    set(Sidekiq_INCLUDE_DIRS ${Sidekiq_INCLUDE_DIR})

    if(${otherlib} MATCHES "libiio.so")
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

        mark_as_advanced(Sidekiq_INCLUDE_DIR Sidekiq_LIBRARY OTHER_LIBS) 
    else()
        include(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
            Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR )

        mark_as_advanced(Sidekiq_INCLUDE_DIR Sidekiq_LIBRARY ) 
    endif()

endif(NOT Sidekiq_FOUND)
