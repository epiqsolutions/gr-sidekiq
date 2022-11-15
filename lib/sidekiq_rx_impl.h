/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H

#include <gnuradio/sidekiq/sidekiq_rx.h>
#include <sidekiq_api.h>

namespace gr {
namespace sidekiq {

class sidekiq_rx_impl : public sidekiq_rx {
private:
  // Nothing to declare in this block.

public:
  sidekiq_rx_impl(
          int input_card,
          int port1_handle,
          int port2_handle,
          double sample_rate,
          double bandwidth,
          double frequency,
          uint8_t gain_mode,
          int gain_index
          );
  ~sidekiq_rx_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;

   bool start() override;

   bool stop() override;

   void set_rx_sample_rate(double value) override;

   void set_rx_gain_mode(double value) override;

   void set_rx_gain_index(int value) override;

   void set_rx_frequency(double value) override;

   void set_rx_bandwidth(double value) override;


private:
     /* passed in parameters */
    uint8_t card{};
    skiq_rx_hdl_t hdl;
    uint32_t sample_rate{};
    uint32_t bandwidth{};
    uint64_t frequency{};
    skiq_rx_gain_t gain_mode{};
    uint8_t gain_index{};

    double adc_scaling{};

    skiq_rx_block_t *p_rx_block{};
    int16_t *curr_block_ptr{};
    int32_t curr_block_samples_left{};

    /* flags */
    bool libsidekiq_init{};
    bool rx_streaming;

    uint32_t debug_ctr{};
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H */
