/*
 * Copyright 2022 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually
 * edited  */
/* The following lines can be configured to regenerate this file during cmake */
/* If manual edits are made, the following tags should be modified accordingly.
 */
/* BINDTOOL_GEN_AUTOMATIC(0) */
/* BINDTOOL_USE_PYGCCXML(0) */
/* BINDTOOL_HEADER_FILE(sidekiq_tx.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(ccaa9bc778fd68f62ae93e8606eca355) */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/sidekiq/sidekiq_tx.h>
// pydoc.h is automatically generated in the build directory
#include <sidekiq_tx_pydoc.h>

void bind_sidekiq_tx(py::module &m) {

  using sidekiq_tx = ::gr::sidekiq::sidekiq_tx;

  py::class_<sidekiq_tx, gr::sync_block, gr::block, gr::basic_block,
             std::shared_ptr<sidekiq_tx>>(m, "sidekiq_tx", D(sidekiq_tx))

      .def(py::init(&sidekiq_tx::make), py::arg("card"), py::arg("handle"),
           py::arg("sample_rate"), py::arg("bandwidth"), py::arg("frequency"),
           py::arg("attenuation"), py::arg("burst_tag"), py::arg("threads"),
           py::arg("buffer_size"), py::arg("cal_mode"), D(sidekiq_tx, make))

      .def("set_tx_sample_rate", &sidekiq_tx::set_tx_sample_rate,
           py::arg("value"), D(sidekiq_tx, set_tx_sample_rate))

      .def("set_tx_attenuation", &sidekiq_tx::set_tx_attenuation,
           py::arg("value"), D(sidekiq_tx, set_tx_attenuation))

      .def("set_tx_frequency", &sidekiq_tx::set_tx_frequency, py::arg("value"),
           D(sidekiq_tx, set_tx_frequency))

      .def("set_tx_bandwidth", &sidekiq_tx::set_tx_bandwidth, py::arg("value"),
           D(sidekiq_tx, set_tx_bandwidth))

      .def("set_tx_cal_mode", &sidekiq_tx::set_tx_cal_mode, py::arg("value"),
           D(sidekiq_tx, set_tx_cal_mode))

      .def("run_tx_cal", &sidekiq_tx::run_tx_cal, py::arg("value"),
           D(sidekiq_tx, run_tx_cal))

      ;
}
