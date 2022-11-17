/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_H

#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sidekiq {

/*!
 * \brief <+description of block+>
 * \ingroup sidekiq
 *
 */
class SIDEKIQ_API sidekiq_rx : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<sidekiq_rx> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of sidekiq::sidekiq_rx.
   *
   * To avoid accidental use of raw pointers, sidekiq::sidekiq_rx's
   * constructor is in a private implementation
   * class. sidekiq::sidekiq_rx::make is the public interface for
   * creating new instances.
   */
  static sptr make(
          int input_card,
          int port1_handle,
          int port2_handle,
          double sample_rate,
          double bandwidth,
          double frequency,
          uint8_t gain_mode,
          int gain_index,
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
