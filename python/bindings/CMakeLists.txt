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
    sidekiq_rx_python.cc
    sidekiq_tx_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(sidekiq 
   ../..
   gr::sidekiq
   "${sidekiq_python_files}")

install(TARGETS sidekiq_python DESTINATION ${GR_PYTHON_DIR}/sidekiq COMPONENT pythonapi)
