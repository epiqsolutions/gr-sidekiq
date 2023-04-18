/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/sidekiq_rx.h>
#include <sidekiq_api.h>

#define MAX_PORT                2        // max ports allowed
#define IQ_SHORT_COUNT          2        // number of shorts in a sample

/* calibration modes */
#define CAL_OFF                 2
#define CAL_TYPE_DC_OFFSET      0
#define CAL_TYPE_QUADRATURE     1
#define CAL_TYPE_BOTH           2

#define RUN_CAL                 1

#define NO_TRANSCEIVE           0
#define TRANSCEIVE_ENABLED      1

#define PKT_TIMEOUT             1000000 // 1ms

#define NON_BLOCKING_TIMEOUT    10 // us
using pmt::pmt_t;

namespace gr {
namespace sidekiq {

    static const double STATUS_UPDATE_RATE_SECONDS{1.0};

    const bool SIDEKIQ_IQ_PACK_MODE_UNPACKED{false}; 

    const int DATA_MAX_BUFFER_SIZE{SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS};

    const pmt_t CONTROL_MESSAGE_PORT{pmt::string_to_symbol("command")};

    static const pmt_t LO_FREQ_KEY{pmt::string_to_symbol("lo_freq")};

    static const pmt_t RATE_KEY{pmt::string_to_symbol("rate")};

    static const pmt_t BANDWIDTH_KEY{pmt::string_to_symbol("bandwidth")};

    static const pmt_t GAIN_KEY{pmt::string_to_symbol("gain")};

class sidekiq_rx_impl : public sidekiq_rx {
public:
  sidekiq_rx_impl(
          int input_card,
          int port1_handle,
          int port2_handle,
          double sample_rate,
          double bandwidth,
          double frequency,
          uint8_t gain_mode,
          int gain_index,
          int trigger_src,
          int pps_source,
          int cal_mode,
          int cal_type
          );
  ~sidekiq_rx_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;

   void handle_control_message(pmt_t message);

   bool start() override;

   bool stop() override;

   void set_rx_sample_rate(double value) override;

   void set_rx_bandwidth(double value) override;

   void set_rx_frequency(double value) override;

   void set_rx_gain_mode(double value) override;

   void set_rx_gain_index(int value) override;

   void set_rx_cal_mode(int value) override;

   void set_rx_cal_type(int value) override;

   void run_rx_cal(int value) override;

private:
    /* private methods */
    uint32_t get_new_block(uint32_t portno);
    bool determine_if_done(int32_t *samples_written, int32_t noutput_items, uint32_t *portno);
    double get_double_from_pmt_dict(pmt_t dict, pmt_t key, pmt_t not_found );

    /* passed in parameters */
    uint8_t card{};
    skiq_rx_hdl_t hdl1{};
    skiq_rx_hdl_t hdl2{};
    uint32_t sample_rate{};
    uint32_t bandwidth{};
    uint64_t frequency{};
    skiq_rx_gain_t gain_mode{};
    uint8_t gain_index{};
    skiq_rx_cal_mode_t cal_mode{};
    skiq_rx_cal_type_t cal_type{};

    skiq_trigger_src_t trigger_src = skiq_trigger_src_immediate;
    skiq_1pps_source_t pps_source{}; 

    /* flags */    
    bool libsidekiq_init{};
    bool rx_streaming{};
    bool cal_enabled{};
    bool dual_port{};
    bool rx_second{};

    /* work parameters */
    uint64_t last_status_update_sample{};
    uint64_t status_update_rate_in_samples{};
    uint64_t overrun_counter{};
    bool first_block[MAX_PORT]{};
    uint64_t last_timestamp[MAX_PORT]{};
    double adc_scaling{};
    int16_t *curr_block_ptr[MAX_PORT]{};
    int32_t curr_block_samples_left[MAX_PORT]{};

    /* used to debug the work function */
    uint32_t debug_ctr{};
    time_t last_time{};
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H */
