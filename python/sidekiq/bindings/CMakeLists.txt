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

########################################################################
# Python Bindings
########################################################################

list(APPEND sidekiq_python_files
    sidekiq_tx_python.cc
    sidekiq_rx_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(sidekiq
   ../../..
   gr::sidekiq
   "${sidekiq_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(TARGET sidekiq_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:sidekiq_python>
    ${CMAKE_BINARY_DIR}/test_modules/gnuradio/sidekiq/
)

install(TARGETS sidekiq_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/sidekiq COMPONENT pythonapi)
