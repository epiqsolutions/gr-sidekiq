if(NOT PKG_CONFIG_FOUND)
    INCLUDE(FindPkgConfig)
endif()
PKG_CHECK_MODULES(PC_SIDEKIQ sidekiq)

FIND_PATH(
    SIDEKIQ_INCLUDE_DIRS
    NAMES sidekiq/api.h
    HINTS $ENV{SIDEKIQ_DIR}/include
        ${PC_SIDEKIQ_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    SIDEKIQ_LIBRARIES
    NAMES gnuradio-sidekiq
    HINTS $ENV{SIDEKIQ_DIR}/lib
        ${PC_SIDEKIQ_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SIDEKIQ DEFAULT_MSG SIDEKIQ_LIBRARIES SIDEKIQ_INCLUDE_DIRS)
MARK_AS_ADVANCED(SIDEKIQ_LIBRARIES SIDEKIQ_INCLUDE_DIRS)

