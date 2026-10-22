/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/sidekiq_rx.h>
#include <sidekiq_api.h>
#include "sidekiq_session.h"
#include <array>
#include <memory>

/* calibration modes */

namespace gr {
namespace sidekiq {
using pmt::pmt_t;

class sidekiq_rx_impl : public sidekiq_rx {
public:
  sidekiq_rx_impl(
          int input_card,
          int input_topology,
          int port1_handle, double frequency1,
          int port2_handle, double frequency2,
          int port3_handle, double frequency3,
          int port4_handle, double frequency4,
          double sample_rate, double bandwidth,
          uint8_t gain_mode,
          int gain_index,
          int timestamp_tags,
          int trigger_src,
          int pps_source,
          int cal_mode,
          int cal_type
          );
  ~sidekiq_rx_impl() override;

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;

   void handle_control_message(pmt_t message);

   bool start() override;

   bool stop() override;

   void set_rx_sample_rate(double value) override;

   void set_rx_bandwidth(double value) override;

   void set_rx_frequency(double value) override;

   void set_rx_frequency_for_hdl(int handle, double value) override;

   void set_rx_gain_mode(double value) override;

   void set_rx_gain_index(int value) override;

   void set_rx_cal_mode(int value) override;

   void set_rx_cal_type(int value) override;

   void run_rx_cal(int value) override;

private:
    static constexpr int data_max_buffer_size = SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS;
    static constexpr int max_port = 4;
    static constexpr int iq_short_count = 2;
    static constexpr int cal_off = 2;
    static constexpr int cal_type_dc_offset = 0;
    static constexpr int cal_type_quadrature = 1;
    static constexpr int cal_type_both = 2;
    static constexpr int run_cal = 1;
    static constexpr int non_blocking_timeout = 10;
    // Declared first so it outlives all other members during destruction.
    std::unique_ptr<sidekiq_session> session;
    /* private methods */
    void write_rate_and_bandwidth(uint32_t rate, uint32_t bandwidth);

    /* passed in parameters */
    uint8_t card{};
    std::array<skiq_rx_hdl_t, max_port> handles{};
    std::array<uint64_t, max_port> frequencies{};
    unsigned num_ports{};
    uint32_t sample_rate{};
    uint32_t bandwidth{};
    skiq_rx_gain_t gain_mode{};
    uint8_t gain_index{};
    bool timestamp_tags{};
    skiq_rx_cal_mode_t cal_mode{};

    skiq_trigger_src_t trigger_src = skiq_trigger_src_immediate;
    skiq_1pps_source_t pps_source{};

    /* flags */

    bool rx_streaming{};
    bool cal_enabled{};
    bool swap_iq_in_software{};

    /* work parameters */
    uint64_t overrun_counter{};
    // Continuity belongs to each handle and survives work() boundaries; start()
    // resets the epoch. Output positions themselves are maintained by GNU Radio.
    bool first_block[max_port]{};
    uint64_t expected_timestamp[max_port]{};
    double adc_scaling{};

};

} // namespace sidekiq
} // namespace gr
