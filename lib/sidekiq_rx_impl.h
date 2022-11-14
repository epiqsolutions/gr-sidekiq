/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H

#include <gnuradio/sidekiq/sidekiq_rx.h>

namespace gr {
namespace sidekiq {

class sidekiq_rx_impl : public sidekiq_rx {
private:
  // Nothing to declare in this block.

public:
  sidekiq_rx_impl();
  ~sidekiq_rx_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H */
