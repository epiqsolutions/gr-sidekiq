/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sidekiq_rx_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sidekiq {

#pragma message("set the following appropriately and remove this warning")
using output_type = float;
sidekiq_rx::sptr sidekiq_rx::make() {
  return gnuradio::make_block_sptr<sidekiq_rx_impl>();
}

/*
 * The private constructor
 */
sidekiq_rx_impl::sidekiq_rx_impl()
    : gr::sync_block("sidekiq_rx", gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1 /* min outputs */,
                                            1 /*max outputs */,
                                            sizeof(output_type))) {}

/*
 * Our virtual destructor.
 */
sidekiq_rx_impl::~sidekiq_rx_impl() {}

int sidekiq_rx_impl::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items) {
  auto out = static_cast<output_type *>(output_items[0]);

#pragma message(                                                               \
    "Implement the signal processing in your block and remove this warning")
  // Do <+signal processing+>

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace sidekiq */
} /* namespace gr */
