find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_SIDEKIQ gnuradio-sidekiq)

FIND_PATH(
    GR_SIDEKIQ_INCLUDE_DIRS
    NAMES gnuradio/sidekiq/api.h
    HINTS $ENV{SIDEKIQ_DIR}/include
        ${PC_SIDEKIQ_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_SIDEKIQ_LIBRARIES
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

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-sidekiqTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_SIDEKIQ DEFAULT_MSG GR_SIDEKIQ_LIBRARIES GR_SIDEKIQ_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_SIDEKIQ_LIBRARIES GR_SIDEKIQ_INCLUDE_DIRS)
