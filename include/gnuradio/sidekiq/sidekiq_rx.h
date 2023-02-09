/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_H

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

using pmt::pmt_t;

namespace gr {
namespace sidekiq {

class SIDEKIQ_API sidekiq_rx : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<sidekiq_rx> sptr;

  static sptr make(
          int input_card,
          int port1_handle,
          int port2_handle,
          double sample_rate,
          double bandwidth,
          double frequency,
          uint8_t gain_mode,
          int gain_index,
          int timestamp_tags,
          int cal_mode,
          int cal_type
          );

            virtual void set_rx_sample_rate(double value) = 0;

            virtual void set_rx_bandwidth(double value) = 0;

            virtual void set_rx_frequency(double value) = 0;

            virtual void set_rx_gain_mode(double value) = 0;

            virtual void set_rx_gain_index(int value) = 0;

            virtual void set_rx_cal_mode(int value) = 0;

            virtual void set_rx_cal_type(int value) = 0;

            virtual void run_rx_cal(int value) = 0;

};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_H */
