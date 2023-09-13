/* -*- c++ -*- */
/*
 * Copyright 2023 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H
#define INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H

#include <gnuradio/sidekiq/telemetry.h>
#include "sidekiq_api.h"
using pmt::pmt_t;

namespace gr {
namespace sidekiq {
    const pmt_t INPUT_MESSAGE_PORT{pmt::string_to_symbol("input")};
    const pmt_t OUTPUT_MESSAGE_PORT{pmt::string_to_symbol("output")};


class telemetry_impl : public telemetry {
private:
  // Nothing to declare in this block.

public:
  telemetry_impl(
          int input_card,
          int temp_enabled,
          int imu_enabled);

  ~telemetry_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H */
