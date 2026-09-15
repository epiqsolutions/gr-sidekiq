/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/sidekiq_tx.h>
#include <sidekiq_api.h>
#include "sidekiq_session.h"
#include <memory>
#include "tx_buffer_pool.h"
#include <atomic>
#include <mutex>
#include <vector>

namespace gr {
namespace sidekiq {
using pmt::pmt_t;

class sidekiq_tx_impl : public sidekiq_tx
{
public:
    sidekiq_tx_impl(
                    int card,
                    int topology,
                    int handle,
                    double sample_rate,
                    double bandwidth,
                    double frequency,
                    double attenuation,
                    std::string burst_tag,
                    int threads,
                    int buffer_size,
                    int cal_mode);

    ~sidekiq_tx_impl() override;

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    /* message handler */
    void handle_control_message(pmt_t message);

    bool start() override;

    bool stop() override;

    void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

    void set_tx_sample_rate(double value) override;

    void set_tx_attenuation(double value) override;

    void set_tx_frequency(double value) override;

    void set_tx_bandwidth(double value) override;

    void set_tx_cal_mode(int value) override;

    /* User sends 1 when it wants to run calibration */
    void run_tx_cal(int value) override;

private:
    static constexpr int default_num_blocks = 20;
    static constexpr int cal_on = 1;
    static constexpr double status_update_interval_seconds = 2.0;
    // Declared first so it outlives all other members during destruction.
    std::unique_ptr<sidekiq_session> session;
    /* method prototypes */
    int work_bursts(int count, const gr_complex* input);
    void submit_packet(const gr_complex* input, size_t count);
    void finish_burst();
    void update_tx_error_count();

    /* passed in parameters */
    uint8_t card{};
    skiq_tx_hdl_t hdl{};
    uint32_t sample_rate{};
    uint32_t bandwidth{};
    uint64_t frequency{};
    uint32_t attenuation{};
    std::string burst_tag_name{};
    skiq_tx_quadcal_mode_t calibration_mode{};

    /* flags */

    std::atomic<bool> tx_streaming{false};

    /* config */
    skiq_part_t card_part{};

    /* sync/async parameters */
    bool in_async_mode{};
    std::shared_ptr<tx_buffer_pool> tx_buffers;
    std::mutex tx_lifecycle_mutex;
    bool dual_channel_packet = false;
    uint32_t num_blocks{};

    /* work() parameters */
    double dac_scaling{};
    size_t last_status_update_sample{};
    size_t status_update_rate_in_samples{};
    uint32_t last_num_tx_errors{};
    uint32_t curr_block{};
    std::vector<gr_complex> temp_buffer;
    int32_t tx_buffer_size{};

    /* bursting */
    // burst_remaining counts input samples still needed, excluding SDK padding.
    // burst_packet owns an incomplete packet until more input arrives.
    uint64_t burst_remaining{};
    std::vector<gr_complex> burst_packet;

};

} // namespace sidekiq
} // namespace gr
