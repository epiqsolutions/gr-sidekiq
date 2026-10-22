/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * GNU Radio source backed by libsidekiq RX. Each SDK packet is copied to its
 * selected output before another receive can reuse the SDK memory. Outputs
 * advance independently; rf_timestamp tags identify packet starts, not an
 * automatic alignment between channels. The session member owns SDK lifetime.
 */

#include "sidekiq_handle_utils.h"
#include "sidekiq_common.h"
#include "sidekiq_rx_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <boost/thread.hpp>
#include <algorithm>
#include <cmath>

using pmt::pmt_t;

namespace gr {
namespace sidekiq {

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int timestamp_tags,
        int trigger_src,
        int pps_source,
        int cal_mode,
        int cal_type)
{
  return sidekiq_rx::make(
          input_card,
          -1,           /* omitted topology leaves the current topology unchanged */
          port1_handle,
          port2_handle,
          sample_rate,
          bandwidth,
          frequency,
          gain_mode,
          gain_index,
          timestamp_tags,
          trigger_src,
          pps_source,
          cal_mode,
          cal_type);
}

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card,
        int input_topology,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int timestamp_tags,
        int trigger_src,
        int pps_source,
        int cal_mode,
        int cal_type)
{
  return sidekiq_rx::make(input_card, input_topology,
          port1_handle, frequency, port2_handle, frequency,
          skiq_rx_hdl_end, frequency, skiq_rx_hdl_end, frequency,
          sample_rate, bandwidth, gain_mode, gain_index, timestamp_tags,
          trigger_src, pps_source, cal_mode, cal_type);
}

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card,
        const std::string& port1_handle,
        const std::string& port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int timestamp_tags,
        int trigger_src,
        int pps_source,
        int cal_mode,
        int cal_type)
{
  return sidekiq_rx::make(
          input_card,
          -1,            /* omitted topology leaves the current topology unchanged */
          port1_handle,
          port2_handle,
          sample_rate,
          bandwidth,
          frequency,
          gain_mode,
          gain_index,
          timestamp_tags,
          trigger_src,
          pps_source,
          cal_mode,
          cal_type);
}

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card,
        int input_topology,
        const std::string& port1_handle,
        const std::string& port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int timestamp_tags,
        int trigger_src,
        int pps_source,
        int cal_mode,
        int cal_type)
{
  return sidekiq_rx::make(
          input_card,
          input_topology,
          static_cast<int>(parse_rx_handle(port1_handle)),
          static_cast<int>(parse_rx_handle(port2_handle, true)),
          sample_rate,
          bandwidth,
          frequency,
          gain_mode,
          gain_index,
          timestamp_tags,
          trigger_src,
          pps_source,
          cal_mode,
          cal_type);
}

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card, int input_topology,
        int port1_handle, double frequency1,
        int port2_handle, double frequency2,
        int port3_handle, double frequency3,
        int port4_handle, double frequency4,
        double sample_rate, double bandwidth, uint8_t gain_mode,
        int gain_index, int timestamp_tags, int trigger_src,
        int pps_source, int cal_mode, int cal_type)
{
  return gnuradio::make_block_sptr<sidekiq_rx_impl>(
          input_card, input_topology,
          port1_handle, frequency1, port2_handle, frequency2,
          port3_handle, frequency3, port4_handle, frequency4,
          sample_rate, bandwidth, gain_mode, gain_index, timestamp_tags,
          trigger_src, pps_source, cal_mode, cal_type);
}

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card, int input_topology,
        const std::string& port1_handle, double frequency1,
        const std::string& port2_handle, double frequency2,
        const std::string& port3_handle, double frequency3,
        const std::string& port4_handle, double frequency4,
        double sample_rate, double bandwidth, uint8_t gain_mode,
        int gain_index, int timestamp_tags, int trigger_src,
        int pps_source, int cal_mode, int cal_type)
{
  return sidekiq_rx::make(input_card, input_topology,
          static_cast<int>(parse_rx_handle(port1_handle)), frequency1,
          static_cast<int>(parse_rx_handle(port2_handle, true)), frequency2,
          static_cast<int>(parse_rx_handle(port3_handle, true)), frequency3,
          static_cast<int>(parse_rx_handle(port4_handle, true)), frequency4,
          sample_rate, bandwidth, gain_mode, gain_index, timestamp_tags,
          trigger_src, pps_source, cal_mode, cal_type);
}

sidekiq_rx_impl::sidekiq_rx_impl(
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
        int local_trigger_src,
        int local_pps_source,
        int cal_mode,
        int cal_type)
    : gr::sync_block("sidekiq_rx", gr::io_signature::make(0, 0, 0),
                                   gr::io_signature::make(1 /* min outputs */, max_port,
                                            sizeof(gr_complex)))
{

    d_logger->set_level("debug");

    int status = 0;
    uint8_t iq_resolution = 0;

    this->timestamp_tags = timestamp_tags;
    card = input_card;

    if (local_trigger_src == 0)
    {
        this->trigger_src = skiq_trigger_src_immediate;
    }
    else if (local_trigger_src == 1)
    {
        this->trigger_src = skiq_trigger_src_1pps;
    }
    else if (local_trigger_src == 2)
    {
        this->trigger_src = skiq_trigger_src_synced;
    }
    else
    {
        d_logger->error( "Error: Invalid trigger source {}" , local_trigger_src);
        throw std::runtime_error("Failure: trigger_src");
    }

    this->pps_source = skiq_1pps_source_unavailable;

    if (trigger_src == skiq_trigger_src_1pps)
    {
        if (local_pps_source == 0)
        {
            this->pps_source = skiq_1pps_source_host;
        }
        else if (local_pps_source == 1)
        {
            this->pps_source = skiq_1pps_source_external;
        }
        else
        {
            d_logger->error( "Error: Invalid pps source {}" , local_pps_source);
            throw std::runtime_error("Failure: trigger_src");
        }

    }

    d_logger->debug("trigger {}, pps_source {}", static_cast<int>(this->trigger_src), static_cast<int>(this->pps_source));

    const int requested_handles[max_port] = {
        port1_handle, port2_handle, port3_handle, port4_handle};
    const double requested_frequencies[max_port] = {
        frequency1, frequency2, frequency3, frequency4};
    for (int i = 0; i < max_port; ++i)
    {
        const int handle = requested_handles[i];
        if (handle == skiq_rx_hdl_end || handle == 100)
        {
            if (i == 0) throw std::invalid_argument("First RX handle is required");
            continue;
        }
        if (handle < 0 || handle >= skiq_rx_hdl_end)
            throw std::invalid_argument("Invalid RX handle");
        if (!std::isfinite(requested_frequencies[i]) || requested_frequencies[i] < 0)
            throw std::invalid_argument("Invalid RX frequency");
        const auto selected = static_cast<skiq_rx_hdl_t>(handle);
        if (std::find(handles.begin(), handles.begin() + num_ports, selected) !=
            handles.begin() + num_ports)
            throw std::invalid_argument("Duplicate RX handle");
        handles[num_ports] = selected;
        frequencies[num_ports] = static_cast<uint64_t>(requested_frequencies[i]);
        ++num_ports;
    }

    /* initialize libsidekiq */
    session = std::make_unique<sidekiq_session>(card);

    /* Topology 0 is valid; -1 means the caller omitted the topology argument. */
    if (input_topology < -1 || input_topology > UINT8_MAX)
    {
        throw std::invalid_argument("Invalid RX topology ID");
    }
    if (input_topology >= 0)
    {
        if (skiq_is_topology_supported(card))
        {
            status = skiq_apply_topology(card, input_topology);
            if (status != 0)
            {
                d_logger->error( "Error: unable to configure topology {} with status {}",
                                 input_topology, status);
                throw std::runtime_error("Failure: skiq_apply_topology");
            }
            d_logger->info("Info: Set topology to {}\n", input_topology);
        }
        else
        {
            d_logger->info("Info: Topology is not supported. Ignoring requested topology\n");
        }
    }

    skiq_param_t params{};
    status = skiq_read_parameters(card, &params);
    if (status != 0)
        throw std::runtime_error("Failure: read RX parameters");
    for (unsigned i = 0; i < num_ports; ++i)
        find_rx_param_index(params, handles[i]);

    set_rx_sample_rate(sample_rate);
    set_rx_bandwidth(bandwidth);

    /* configure the 1PPS source for each of the cards */
    if ( pps_source != skiq_1pps_source_unavailable )
    {
        status = skiq_write_1pps_source( card, pps_source );
        if ( status != 0 )
        {
            d_logger->error( "Error: unable to write 1pps source with status {}", status);
            throw std::runtime_error("Failure: skiq_write_1pps_source");
        }
        else
        {
            d_logger->info("Info: configured 1PPS source to {}", static_cast<int>(pps_source));
        }
      }

    /* calculate the adc scaling */
    status = skiq_read_rx_iq_resolution(card, &iq_resolution);
    if (status != 0)
    {
        d_logger->error( "Error: unable to get iq resolution with status {}", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    adc_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    d_logger->info("Info: ADC scaling {}", adc_scaling);

    // On topology-aware cards the selected topology controls channel layout.
    // Match the TX block's exceptions for cards that still require chan_mode.
    if (!skiq_is_topology_supported(card) ||
        params.card_param.part_type == skiq_nv100 ||
        params.card_param.part_type == skiq_nvm2) {
        const bool needs_dual_mode = std::any_of(
            handles.begin(), handles.begin() + num_ports,
            [](skiq_rx_hdl_t handle) {
                return handle == skiq_rx_hdl_A2 || handle == skiq_rx_hdl_B2;
            });
        status = skiq_write_chan_mode(
            card, needs_dual_mode ? skiq_chan_mode_dual : skiq_chan_mode_single);
        if (status != 0)
        {
            d_logger->error("Error: unable to configure RX channel mode, status {}", status);
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    }

    /* always assume unpacked */
    status = skiq_write_iq_pack_mode(card, detail::packed_iq);
    if (status != 0)
    {
        d_logger->error( "Error: unable to set iq pack mode to unpacked with status {}", status);
        throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    /* by default all cards are in Q/I order we want it to be I/Q so switch it */
    status = skiq_write_iq_order_mode(card, skiq_iq_order_iq) ;
    if (status == -ENOTSUP || status == -ENOSYS)
    {
        swap_iq_in_software = true;
        d_logger->warn("RX IQ order mode unsupported; swapping I and Q in software");
    }
    else if (status != 0)
    {
          d_logger->error( "Error: unable to set iq order mode to iq with status {} ", status);
          throw std::runtime_error("Failure: skiq_write_iq_order_mode");
    }

    /* support two messages */
    message_port_register_in(detail::command_port);
    set_msg_handler(detail::command_port, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });

    /* set the rest of the parameters */
    for (unsigned i = 0; i < num_ports; ++i)
        set_rx_frequency_for_hdl(static_cast<int>(handles[i]), frequencies[i]);
    set_rx_gain_mode(gain_mode);

    if (gain_mode == skiq_rx_gain_manual)
    {
        set_rx_gain_index(gain_index);
    }

    set_rx_cal_mode(cal_mode);
    set_rx_cal_type(cal_type);

    /* we need gnuradio to send in buffers of an integer multiple of our DMA block sizes */
    gr::block::set_min_noutput_items(data_max_buffer_size);
    gr::block::set_output_multiple(data_max_buffer_size);

}

/* deconstructor */
sidekiq_rx_impl::~sidekiq_rx_impl()
{
    d_logger->debug("in RX deconstructor");

    if (rx_streaming )
    {
        stop();
        rx_streaming = false;
    }

}

void sidekiq_rx_impl::handle_control_message(pmt_t msg)
{
    d_logger->debug("in handle_control_message");

    msg = detail::command_dict(msg);

     // Make sure, we use dicts!
     if (!pmt::is_dict(msg)) {
         d_logger->error("Command message is neither dict nor pair: {}", pmt::write_string(msg));
         return;
     }

    if (pmt::dict_has_key(msg, detail::frequency_key))
    {
        set_rx_frequency(detail::command_number(msg, detail::frequency_key));
    }

    // Per-handle tuning keeps the existing lo_freq broadcast command intact.
    const char* frequency_keys[] = {
        "lo_freq_rxa1", "lo_freq_rxa2", "lo_freq_rxb1",
        "lo_freq_rxb2", "lo_freq_rxc1", "lo_freq_rxd1"};
    for (int handle = 0; handle < skiq_rx_hdl_end; ++handle) {
        const auto key = pmt::intern(frequency_keys[handle]);
        if (pmt::dict_has_key(msg, key))
            set_rx_frequency_for_hdl(handle, detail::command_number(msg, key));
    }

    if (pmt::dict_has_key(msg, detail::rate_key))
    {
        set_rx_sample_rate(detail::command_number(msg, detail::rate_key));
    }

    if (pmt::dict_has_key(msg, detail::bandwidth_key))
    {
        set_rx_bandwidth(detail::command_number(msg, detail::bandwidth_key));
    }

    if (pmt::dict_has_key(msg, detail::gain_key))
    {
        set_rx_gain_index(detail::command_number(msg, detail::gain_key));
    }

}

/*
 * start streaming
 *
 * Called by the generated python code
 */
bool sidekiq_rx_impl::start()
{
    if (rx_streaming) return block::start();
    int status = 0;

    d_logger->debug("in start");

    status = skiq_reset_timestamps(card);
    if (status != 0)
    {
        d_logger->error( "Error: could not reset timestamps, status {}", status);
        throw std::runtime_error("Failure: skiq_reset_timestamps");
    }

    status = skiq_start_rx_streaming_multi_on_trigger(
        card, handles.data(), static_cast<uint8_t>(num_ports), trigger_src, 0);
    if ( status != 0 )
    {
       d_logger->error( "Error: could not start RX streaming on hdl1, status {}", status);
       throw std::runtime_error("Failure: skiq_start_rx_streaming");
    }

    rx_streaming = true;

    // A new stream starts a new continuity epoch. Tag offsets come from GNU Radio.
    for (unsigned i = 0; i < num_ports; ++i) {
        first_block[i] = true;
        expected_timestamp[i] = 0;
    }
    overrun_counter = 0;

    d_logger->info("Info: RX streaming started");

    return block::start();
}

/*
 * stop streaming
 *
 * Called by the generated python code
 */
bool sidekiq_rx_impl::stop()
{
    int status = 0;

    d_logger->debug("in stop");

    /* only call stop if we are actually streaming */
    if (rx_streaming )
    {
        status = skiq_stop_rx_streaming_multi_on_trigger(
            card, handles.data(), static_cast<uint8_t>(num_ports), trigger_src, 0);
        if ( status != 0 )
        {
           d_logger->error("Error: could not stop RX streaming, status {}", status);
           return false;
        }
        d_logger->info("Info: RX streaming stopped");
    }

    rx_streaming = false;

    return block::stop();
}

/*
 * set the sample rate
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value range is valid
 */
void sidekiq_rx_impl::write_rate_and_bandwidth(uint32_t rate, uint32_t bandwidth)
{
    int status;
    if (num_ports == 1) {
        status = skiq_write_rx_sample_rate_and_bandwidth(card, handles[0], rate, bandwidth);
    } else {
        std::array<uint32_t, max_port> rates{};
        std::array<uint32_t, max_port> bandwidths{};
        rates.fill(rate);
        bandwidths.fill(bandwidth);
        status = skiq_write_rx_sample_rate_and_bandwidth_multi(
            card, handles.data(), static_cast<uint8_t>(num_ports),
            rates.data(), bandwidths.data());
    }
    if (status != 0)
        throw std::runtime_error("Failure: set RX rate/bandwidth, status " +
                                 std::to_string(status));
}

void sidekiq_rx_impl::set_rx_sample_rate(double value)
{
    double actual_rate;
    uint32_t requested_rate, requested_bw, actual_bw;
    auto new_rate = static_cast<uint32_t>(value);
    uint32_t new_bw = 0;
    skiq_param_t params;
    int status = 0;

    d_logger->debug("in set_rx_sample_rate");

    status = skiq_read_parameters(card, &params);
    if (status != 0)
    {
        d_logger->error( "Error: could not read parameters, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    for (unsigned i = 0; i < num_ports; ++i) {
        const int param_idx = find_rx_param_index(params, handles[i]);
        if (new_rate < params.rx_param[param_idx].sample_rate_min ||
            new_rate > params.rx_param[param_idx].sample_rate_max)
            throw std::runtime_error("Failure: sample rate out of range for RX handle " +
                                     std::to_string(static_cast<int>(handles[i])));
    }

    status = skiq_read_rx_sample_rate_and_bandwidth(card, handles[0],
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl1, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    new_bw = std::min(actual_bw, new_rate);

    write_rate_and_bandwidth(new_rate, new_bw);
    status = skiq_read_rx_sample_rate_and_bandwidth(card, handles[0],
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl1, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    if (new_rate == actual_rate)
    {
        d_logger->info("Info: sample rate set to {}", actual_rate);
    }
    else
    {
        d_logger->info("Warning: Requested sample rate {} but actually set to {}", new_rate, actual_rate);
    }

    this->sample_rate = static_cast<uint32_t>(actual_rate);
    this->bandwidth = actual_bw;
}

/*
 * set the bandwidth
 * this may be called from the generated python code if the user changes the variablea
 *
 * let libsidekiq determine if the value range is valid
 */
void sidekiq_rx_impl::set_rx_bandwidth(double value)
{
    int status = 0;
    double actual_rate;
    uint32_t requested_rate, requested_bw, actual_bw;
    auto new_bw = static_cast<uint32_t>(value);

    d_logger->debug("in set_rx_bandwidth");

    status = skiq_read_rx_sample_rate_and_bandwidth(card, handles[0],
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl1, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    write_rate_and_bandwidth(static_cast<uint32_t>(actual_rate), new_bw);

    status = skiq_read_rx_sample_rate_and_bandwidth(card, handles[0],
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl1, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }
    if (new_bw == actual_bw)
    {
        d_logger->info("Info: bandwidth set to {}", actual_bw);
    }
    else
    {
        d_logger->info("Warning: Requested bandwidth {} but actually set to {}", new_bw, actual_bw);
    }

    this->sample_rate = static_cast<uint32_t>(actual_rate);
    this->bandwidth = actual_bw;
}

/*
 * set the LO frequency
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_frequency(double value)
{
    for (unsigned i = 0; i < num_ports; ++i)
        set_rx_frequency_for_hdl(static_cast<int>(handles[i]), value);
}

void sidekiq_rx_impl::set_rx_frequency_for_hdl(int handle, double value)
{
    // GRC emits callbacks for hidden optional handles too.
    if (handle == skiq_rx_hdl_end || handle == 100) return;
    if (handle < 0 || handle >= skiq_rx_hdl_end ||
        !std::isfinite(value) || value < 0)
        throw std::invalid_argument("Invalid RX handle or frequency");
    const auto it = std::find(handles.begin(), handles.begin() + num_ports,
                              static_cast<skiq_rx_hdl_t>(handle));
    if (it == handles.begin() + num_ports)
        throw std::invalid_argument("RX handle is not selected");
    const auto requested = static_cast<uint64_t>(value);
    const auto status = skiq_write_rx_LO_freq(card, *it, requested);
    if (status != 0)
        throw std::runtime_error("Failure: set RX frequency on handle " +
                                 std::to_string(handle) + ", status " +
                                 std::to_string(status));
    frequencies[std::distance(handles.begin(), it)] = requested;
}

/*
 * set the gain_mode
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_gain_mode(double value)
{
    int status = 0;

    d_logger->debug("in set_rx_gain_mode");

    auto gain_mode = static_cast<skiq_rx_gain_t>(value);

    for (unsigned i = 0; i < num_ports; ++i) {
        status = skiq_write_rx_gain_mode(card, handles[i], gain_mode);
        if (status != 0)
        {
            d_logger->error("Error: write_rx_gain_mode failed on RX handle {}, status {}",
                            static_cast<int>(handles[i]), status);
            throw std::runtime_error("Failure: set write_rx_gain_mode");
        }
    }

    d_logger->info("Info: gain_mode set to {}", static_cast<int>(gain_mode));

    this->gain_mode = gain_mode;

}

/*
 * set the gain_index
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_gain_index(int value)
{
    int status = 0;
    uint8_t min_range = 0;
    uint8_t max_range = 0;

    d_logger->debug("in set_rx_gain_index");

    auto gain = static_cast<uint8_t>(value);

    if (this->gain_mode == skiq_rx_gain_manual)
    {
        // Check every selected handle before changing any gain setting.
        for (unsigned i = 0; i < num_ports; ++i) {
            status = skiq_read_rx_gain_index_range(card, handles[i], &min_range, &max_range);
            if (status != 0 || gain < min_range || gain > max_range)
                throw std::runtime_error("Failure: gain index unavailable on RX handle " +
                                         std::to_string(static_cast<int>(handles[i])));
        }
        for (unsigned i = 0; i < num_ports; ++i) {
            status = skiq_write_rx_gain(card, handles[i], gain);
            if (status != 0)
            {
                d_logger->error("Error: write_rx_gain failed on RX handle {}, status {}",
                                static_cast<int>(handles[i]), status);
                throw std::runtime_error("Failure: set read_rx_gain_index");
            }
        }

        d_logger->info("Info: gain index {}", gain);

        this->gain_index = gain;
    }
    else
    {
        d_logger->warn("set_gain_index called but in Auto Gain Mode");
    }
}

/*
 * set the cal_mode
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_cal_mode(int value)
{
    if (value == cal_off) {
        cal_enabled = false;
        d_logger->info("RX calibration mode off");
        return;
    }
    // Preserve the existing unsupported-mode warning and configuration policy.
    cal_enabled = true;
    const auto requested = static_cast<skiq_rx_cal_mode_t>(value);
    for (unsigned i = 0; i < num_ports; ++i) {
        const auto status = skiq_write_rx_cal_mode(card, handles[i], requested);
        if (status == -ENOTSUP) {
            d_logger->warn("RX calibration mode {} unsupported on handle {}",
                           value, static_cast<int>(handles[i]));
        } else if (status != 0) {
            d_logger->error("Failed to set RX calibration mode: {}", status);
            throw std::runtime_error("Failure: set rx_cal_mode");
        }
    }
    cal_mode = requested;
    d_logger->info("RX calibration mode set to {}", value);
}

/*
 * set the cal_type
 *
 * Some cards have DC_OFFSET some QUADRATURE and some BOTH
 *
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_cal_type(int value)
{
    if (!cal_enabled) return;
    uint32_t requested;
    switch (value) {
    case cal_type_dc_offset: requested = skiq_rx_cal_type_dc_offset; break;
    case cal_type_quadrature: requested = skiq_rx_cal_type_quadrature; break;
    case cal_type_both:
        requested = skiq_rx_cal_type_dc_offset | skiq_rx_cal_type_quadrature;
        break;
    default: throw std::invalid_argument("Invalid RX calibration type");
    }
    uint32_t masks[max_port]{};
    // Resolve every handle's capabilities before changing any masks. Never
    // enable an unrequested algorithm just because the hardware supports it.
    for (unsigned i = 0; i < num_ports; ++i) {
        uint32_t available = 0;
        const auto status = skiq_read_rx_cal_types_avail(card, handles[i], &available);
        if (status != 0)
            throw std::runtime_error("Failure: read RX calibration capabilities, status " +
                                     std::to_string(status));
        masks[i] = requested & available;
        if (!masks[i])
            throw std::runtime_error("Requested RX calibration types unavailable on handle " +
                                     std::to_string(static_cast<int>(handles[i])));
        if (masks[i] != requested)
            d_logger->warn("RX handle {} supports calibration mask 0x{:X} of requested 0x{:X}",
                           static_cast<int>(handles[i]), masks[i], requested);
    }
    for (unsigned i = 0; i < num_ports; ++i) {
        const auto status = skiq_write_rx_cal_type_mask(card, handles[i], masks[i]);
        if (status != 0)
            throw std::runtime_error("Failure: write RX calibration mask, status " +
                                     std::to_string(status));
    }
}

void sidekiq_rx_impl::run_rx_cal(int value)
{
    if (value != run_cal || !cal_enabled || cal_mode != skiq_rx_cal_mode_manual) return;
    for (unsigned i = 0; i < num_ports; ++i) {
        const auto status = skiq_run_rx_cal(card, handles[i]);
        if (status != 0)
            throw std::runtime_error("Failure: run RX calibration on handle " +
                                     std::to_string(static_cast<int>(handles[i])) +
                                     ", status " + std::to_string(status));
    }
}

// Publish each packet independently. A faster handle must not overwrite or
// discard samples while waiting for the other handle to fill its output.
int sidekiq_rx_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    (void)input_items;
    boost::this_thread::interruption_point();
    skiq_rx_hdl_t handle;
    skiq_rx_block_t* packet = nullptr;
    uint32_t bytes = 0;
    const auto status = skiq_receive(card, &handle, &packet, &bytes);
    if (status == skiq_rx_status_no_data || status == skiq_rx_status_error_overrun) {
        // Keep idle polling interruptible, including a radio awaiting a trigger.
        boost::this_thread::sleep(boost::posix_time::microseconds(non_blocking_timeout));
        return 0;
    }
    if (status != skiq_rx_status_success)
        throw std::runtime_error("Failure: skiq_receive, status " + std::to_string(status));
    const auto selected = std::find(handles.begin(), handles.begin() + num_ports, handle);
    if (selected == handles.begin() + num_ports)
        throw std::runtime_error("Failure: unexpected RX handle");
    const unsigned port = std::distance(handles.begin(), selected);
    // Unpacked IQ uses one 32-bit word per complex sample. The returned byte
    // count includes the SDK header; do not treat header bytes as samples.
    const uint32_t header_bytes = SKIQ_RX_HEADER_SIZE_IN_WORDS * sizeof(uint32_t);
    if (!packet || bytes <= header_bytes || bytes > SKIQ_MAX_RX_BLOCK_SIZE_IN_BYTES ||
        (bytes - header_bytes) % sizeof(uint32_t))
        throw std::runtime_error("Failure: invalid RX packet length");
    const auto samples = (bytes - header_bytes) / sizeof(uint32_t);
    if (samples > static_cast<unsigned>(noutput_items))
        throw std::runtime_error("Failure: RX output buffer too small");
    const uint64_t timestamp = packet->rf_timestamp;
    // Track each handle across work calls. A gap is reported, not repaired by
    // inserting samples or moving the other channel's output position.
    if (!first_block[port] && timestamp != expected_timestamp[port]) {
        ++overrun_counter;
        d_logger->warn("RX timestamp discontinuity on port {}: expected {}, received {} (count {})",
                       port, expected_timestamp[port], timestamp, overrun_counter);
    }
    first_block[port] = false;
    expected_timestamp[port] = timestamp + samples;
    auto* output = static_cast<gr_complex*>(output_items[port]);
    volk_16i_s32f_convert_32f_u(reinterpret_cast<float*>(output),
                              const_cast<const int16_t*>(packet->data),
                              adc_scaling, samples * iq_short_count);
    if (swap_iq_in_software) {
        for (uint32_t i = 0; i < samples; ++i)
            output[i] = gr_complex(output[i].imag(), output[i].real());
    }
    if (timestamp_tags)
        add_item_tag(port, nitems_written(port), pmt::intern("rf_timestamp"),
                     pmt::from_uint64(timestamp));
    // SDK memory is consumed before the next receive. GNU Radio owns the copy.
    produce(port, samples);
    for (unsigned i = 0; i < num_ports; ++i)
        if (i != port) produce(i, 0);
    // A normal positive return would advance every output by the same count.
    // Explicit produce() advances only the port that actually received data.
    return WORK_CALLED_PRODUCE;
}

} // namespace sidekiq
} // namespace gr
