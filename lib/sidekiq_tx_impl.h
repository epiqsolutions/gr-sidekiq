/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H

#include <gnuradio/sidekiq/sidekiq_tx.h>
#include <sidekiq_api.h>

namespace gr {
namespace sidekiq {

class sidekiq_tx_impl : public sidekiq_tx
{
public:
    sidekiq_tx_impl(
                    int card, 
                    int handle,
                    double sample_rate,
                    double bandwidth,
                    double frequency,
                    double attenuation,
                    int threads,
                    int buffer_size);




    ~sidekiq_tx_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;



    bool start() override;

    bool stop() override;

    void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

    void set_tx_sample_rate(double value) override;

    void set_tx_attenuation(double value) override;

    void set_tx_frequency(double value) override;

    void set_tx_bandwidth(double value) override;

private:
    /* passed in parameters */
    uint8_t card{};
    skiq_tx_hdl_t hdl;
    uint32_t sample_rate{};
    uint32_t bandwidth{};
    uint64_t frequency{};
    uint32_t attenuation{};

    /* flags */
    bool libsidekiq_init;
    bool in_async_mode;
    bool tx_streaming;

    /* sync/async parameters */
    skiq_tx_block_t **p_tx_blocks;
    skiq_tx_block_t *sync_tx_block;
    int32_t *p_tx_status;
    uint32_t num_blocks;

    /* work() parameters */
    double dac_scaling{};
    size_t last_status_update_sample{};
    size_t status_update_rate_in_samples{};
    uint32_t last_num_tx_errors{};
    uint32_t curr_block{};
    std::vector<gr_complex> temp_buffer;
    int32_t tx_buffer_size{};
    void update_tx_error_count();
    uint64_t timestamp{};
    
    uint32_t debug_ctr{};

};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H */
