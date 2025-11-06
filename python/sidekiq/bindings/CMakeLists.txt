# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT sidekiq_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT sidekiq_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
)

include(GrPybind)
include(GNUInstallDirs)  # for CMAKE_INSTALL_FULL_LIBDIR

########################################################################
# Python Bindings
########################################################################

list(APPEND sidekiq_python_files
    sidekiq_tx_python.cc
    sidekiq_rx_python.cc
    python_bindings.cc
)

GR_PYBIND_MAKE_OOT(sidekiq
   ../../..
   gr::sidekiq
   "${sidekiq_python_files}"
)

# ---- Minimal Z4 support: link SDK libs and embed runtime search paths ----
# Assumes FindSidekiq.cmake exported:
#   Sidekiq_LIBRARIES  -> SDK lib (shared on Z4)
#   OTHER_LIBS         -> extras (gpiod on Z4, iio on some ARM suffixes)
#   Sidekiq_LIB_DIRS   -> .../sidekiq_sdk_current/lib/support/<suffix>/usr/lib/epiq
if (TARGET sidekiq_python)
    # Make the SDK support dir visible to the linker at build time (older CMake behavior)
    if (DEFINED Sidekiq_LIB_DIRS AND EXISTS "${Sidekiq_LIB_DIRS}")
        link_directories("${Sidekiq_LIB_DIRS}")
    endif()

    # Link what the finder exports (don’t hardcode system libs here)
    if (DEFINED Sidekiq_LIBRARIES AND NOT "${Sidekiq_LIBRARIES}" STREQUAL "")
        target_link_libraries(sidekiq_python PRIVATE ${Sidekiq_LIBRARIES})
    endif()
    if (DEFINED OTHER_LIBS AND NOT "${OTHER_LIBS}" STREQUAL "")
        target_link_libraries(sidekiq_python PRIVATE ${OTHER_LIBS})
    endif()

    # RPATH so the module can dlopen() both the SDK libs and the core GR lib
    set(_SDK_DIR  "${Sidekiq_LIB_DIRS}")           # Z4: libsidekiq*.so, libgpiod.so
    set(_CORE_DIR "${CMAKE_INSTALL_FULL_LIBDIR}")  # e.g. /usr/local/lib/aarch64-linux-gnu

    # Use ';' — CMake converts to ':' on UNIX RPATH
    set_target_properties(sidekiq_python PROPERTIES
        BUILD_RPATH   "${_SDK_DIR};${_CORE_DIR};$ORIGIN"
        INSTALL_RPATH "${_SDK_DIR};${_CORE_DIR};$ORIGIN"
    )
endif()

# copy bindings extension for use in QA test module
add_custom_command(TARGET sidekiq_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/test_modules/gnuradio/sidekiq
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:sidekiq_python>
            ${CMAKE_BINARY_DIR}/test_modules/gnuradio/sidekiq/
)

install(TARGETS sidekiq_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/sidekiq
    COMPONENT pythonapi
)
