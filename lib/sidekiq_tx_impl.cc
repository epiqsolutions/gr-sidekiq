/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/*
 * GNU Radio sink backed by libsidekiq TX. Converts complex input into fixed-size
 * SDK packets for immediate transmission, length-tag bursts, or UHD-compatible
 * timed SOB/EOB bursts. The buffer pool protects asynchronous transfers; the
 * session protects SDK lifetime.
 */

#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>
#include <cmath>
#include <limits>

#include "sidekiq_handle_utils.h"
#include "sidekiq_common.h"
#include "sidekiq_tx_impl.h"

namespace gr {
namespace sidekiq {


/* This is the top level class instantiated by gnuradio */
sidekiq_tx::sptr sidekiq_tx::make(int card,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  std::string burst_tag,
                                  int threads,
                                  int buffer_size,
                                  int cal_mode,
                                  int timed_tx,
                                  int reset_timestamps_on_start)
{
    return sidekiq_tx::make(card,
                            0,
                            handle,
                            sample_rate,
                            bandwidth,
                            frequency,
                            attenuation,
                            burst_tag,
                            threads,
                            buffer_size,
                            cal_mode,
                            timed_tx,
                            reset_timestamps_on_start);
}

sidekiq_tx::sptr sidekiq_tx::make(int card,
                                  int topology,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  std::string burst_tag,
                                  int threads,
                                  int buffer_size,
                                  int cal_mode,
                                  int timed_tx,
                                  int reset_timestamps_on_start)
{
    /* then make instantiates the tx_block */
    return gnuradio::make_block_sptr<sidekiq_tx_impl>(
                                  card,
                                  topology,
                                  handle,
                                  sample_rate,
                                  bandwidth,
                                  frequency,
                                  attenuation,
                                  burst_tag,
                                  threads,
                                  buffer_size,
                                  cal_mode,
                                  timed_tx,
                                  reset_timestamps_on_start);
}

sidekiq_tx::sptr sidekiq_tx::make(int card,
                                  const std::string& handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  std::string burst_tag,
                                  int threads,
                                  int buffer_size,
                                  int cal_mode,
                                  int timed_tx,
                                  int reset_timestamps_on_start)
{
    return sidekiq_tx::make(card,
                            0,
                            handle,
                            sample_rate,
                            bandwidth,
                            frequency,
                            attenuation,
                            burst_tag,
                            threads,
                            buffer_size,
                            cal_mode,
                            timed_tx,
                            reset_timestamps_on_start);
}

sidekiq_tx::sptr sidekiq_tx::make(int card,
                                  int topology,
                                  const std::string& handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  std::string burst_tag,
                                  int threads,
                                  int buffer_size,
                                  int cal_mode,
                                  int timed_tx,
                                  int reset_timestamps_on_start)
{
    return sidekiq_tx::make(card,
                            topology,
                            static_cast<int>(parse_tx_handle(handle)),
                            sample_rate,
                            bandwidth,
                            frequency,
                            attenuation,
                            burst_tag,
                            threads,
                            buffer_size,
                            cal_mode,
                            timed_tx,
                            reset_timestamps_on_start);
}

/* constructor
 * Initialize the card
 */
sidekiq_tx_impl::sidekiq_tx_impl( int input_card,
                                  int topology,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  std::string burst_tag,
                                  int threads,
                                  int buffer_size,
                                  int cal_mode,
                                  int timed_tx,
                                  int reset_timestamps_on_start)
    : gr::sync_block("sidekiq_tx",
                     gr::io_signature::make( 1 /* min inputs */, 1 /* max inputs */, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0))   //sync block
{

    d_logger->set_level("debug");

    int status = 0;
    skiq_param_t param;
    uint8_t iq_resolution = 0;
    status_update_rate_in_samples = static_cast<size_t >(sample_rate * status_update_interval_seconds);

    card = input_card;
    hdl = static_cast<skiq_tx_hdl_t>(handle);
    curr_block = 0;
    tx_buffer_size = buffer_size;
    num_blocks = default_num_blocks;

    burst_tag_name = burst_tag;
    this->timed_tx = timed_tx != 0;
    this->reset_timestamps_on_start = reset_timestamps_on_start != 0;
    d_logger->debug("burst_tag_name: {}", burst_tag_name);

    session = std::make_unique<sidekiq_session>(card);

    status = skiq_read_parameters(card, &param);
    if (status != 0)
    {
        d_logger->error( "Error: unable to read card parameters with status {}", status);
        throw std::runtime_error("Failure: skiq_read_parameters");
    }
    card_part = param.card_param.part_type;

    bool topology_supported = false;
#if LIBSIDEKIQ_VERSION >= 42600
    topology_supported = skiq_is_topology_supported(card);

    /* set topology if it has changed from default (0) */
    if (topology != 0)
    {
        if (topology_supported)
        {
            status = skiq_apply_topology(card, topology);
            if (status != 0)
            {
                d_logger->error( "Error: unable to configure topology {} with status {}",
                                 topology, status);
                throw std::runtime_error("Failure: skiq_apply_topology");
            }
            d_logger->info("Info: Set topology to {}\n", topology);
        }
        else
        {
            d_logger->info("Info: Topology is not supported. Ignoring requested topology\n");
        }
    }
#else
    if (topology != 0)
    {
        d_logger->warn(
            "Sidekiq SDK {}.{}.{} does not support topology selection; ignoring topology {}",
            LIBSIDEKIQ_VERSION_MAJOR,
            LIBSIDEKIQ_VERSION_MINOR,
            LIBSIDEKIQ_VERSION_PATCH,
            topology);
    }
#endif

    if (session->initialized_card())
    {
        set_tx_sample_rate(sample_rate);
        set_tx_bandwidth(bandwidth);
    }
    else
    {
        uint32_t requested_rate = 0;
        uint32_t requested_bandwidth = 0;
        uint32_t actual_bandwidth = 0;
        double actual_rate = 0;
        status = skiq_read_tx_sample_rate_and_bandwidth(
            card, hdl, &requested_rate, &actual_rate, &requested_bandwidth, &actual_bandwidth);
        if (status != 0 || actual_rate <= 0) {
            d_logger->error("Error: unable to read active TX sample rate with status {}", status);
            throw std::runtime_error("Failure: read active TX sample rate");
        }
        this->sample_rate = static_cast<uint32_t>(actual_rate);
        this->bandwidth = actual_bandwidth;
    }

    status = skiq_read_tx_iq_resolution(card, &iq_resolution);
    if (status != 0)
    {
        d_logger->error( "Error: unable to get iq resolution with status {}", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    dac_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    d_logger->info("Info: dac scaling {}", dac_scaling);

    if (this->timed_tx) {
        status = skiq_write_tx_timestamp_base(card, skiq_tx_rf_timestamp);
        if (status != 0) {
            d_logger->error("Error: could not set TX timestamp base with status {}", status);
            throw std::runtime_error("Failure: skiq_write_tx_timestamp_base");
        }
    }
    const auto flow_mode = this->timed_tx ? skiq_tx_with_timestamps_data_flow_mode
                                          : skiq_tx_immediate_data_flow_mode;
    status = skiq_write_tx_data_flow_mode(card, hdl, flow_mode);
    if (status != 0)
    {
        d_logger->error( "Error: could not set TX dataflow mode with status {}", status);
        throw std::runtime_error("Failure: skiq_write_tx_flow_mode");
    }

    bool use_legacy_channel_mode = !topology_supported || card_part == skiq_nv100;
#if LIBSIDEKIQ_VERSION >= 41900
    use_legacy_channel_mode = use_legacy_channel_mode || card_part == skiq_nvm2;
#endif
    if (use_legacy_channel_mode)
    {
        /* if A2 or B2 is used, we need to set the channel mode to dual */
        if (hdl == skiq_tx_hdl_A2 || hdl == skiq_tx_hdl_B2)
        {
            status = skiq_write_chan_mode(card, skiq_chan_mode_dual);
            if (status != 0)
            {
                d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
                throw std::runtime_error("Failure: skiq_write_chan_mode");
            }

	    /*
	     * Buffer size was assumed to be single channel (2^n - 4).
	     * For dual channel, only half the header size (4) applies to each channel
	     */
            tx_buffer_size += 2;
            dual_channel_packet = true;
        }
        else {
            status = skiq_write_chan_mode(card, skiq_chan_mode_single);
            if (status != 0)
            {
                d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
                throw std::runtime_error("Failure: skiq_write_chan_mode");
            }
        }
    }

    /* write the block size to the passed in amount */
    status = skiq_write_tx_block_size(card, hdl, tx_buffer_size);
    if (status != 0)
    {
        d_logger->error( "Error: unable to configure TX block size: {} with status {}",
                tx_buffer_size, status);
        throw std::runtime_error("Failure: skiq_write_tx_block_size");
    }
    d_logger->info("Info: TX block size {}", tx_buffer_size);

    /* Set conversion vector buffer size */
    temp_buffer.resize(tx_buffer_size);

    /* handle sync vs async mode */
    in_async_mode = threads > 1;
    if (in_async_mode)
    {
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_async);
        if (status != 0)
        {
            d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }

        status = skiq_write_num_tx_threads(card, threads);
        if (status != 0)
        {
            d_logger->error("Error: unable to configure TX number of threads with status {}", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }

        status = skiq_register_tx_complete_callback( card, &tx_buffer_pool::complete );
        if (status != 0)
        {
            d_logger->error( "Error: unable to configure TX callback with status {}", status);
            throw std::runtime_error("Failure: skiq_register_tx_complete_callback");
        }
        d_logger->info("Info: in async mode with {} threads", threads);
    }
    else {
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_sync);
        if (status != 0)
        {
            d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }
        num_blocks = 1;
        d_logger->info("Info: in sync mode ");
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
    if (status != 0)
    {
          d_logger->error( "Error: unable to set iq order mode to iq with status {} ", status);
          throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    // The SDK block size is per channel. Dual-channel packets carry two
    // contiguous payloads; the existing single input feeds the selected A2/B2.
    tx_buffers = std::make_shared<tx_buffer_pool>(
        num_blocks, tx_buffer_size * (dual_channel_packet ? 2 : 1));

    message_port_register_in(detail::command_port);
    set_msg_handler(detail::command_port, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });

    /* set the frequency and attenuation */
    set_tx_frequency(frequency);
    set_tx_attenuation(attenuation);
    set_tx_cal_mode(cal_mode);

}

/* Destructor, free all the memory allocated */
sidekiq_tx_impl::~sidekiq_tx_impl()
{
    d_logger->debug("in TX destructor");

    // Stop/cancel transfers before releasing storage. Callback state is owned
    // independently by the pool in case a transport completes asynchronously.
    stop();


}

void sidekiq_tx_impl::handle_control_message(pmt_t msg)
{
    d_logger->debug("in handle_control ");

    msg = detail::command_dict(msg);

     // Make sure, we use dicts!
     if (!pmt::is_dict(msg)) {
         d_logger->error("Command message is neither dict nor pair: {}", pmt::write_string(msg));
         return;
     }

    if (pmt::dict_has_key(msg, detail::frequency_key))
    {
        set_tx_frequency(detail::command_number(msg, detail::frequency_key));
    }

    if (pmt::dict_has_key(msg, detail::rate_key))
    {
        set_tx_sample_rate(detail::command_number(msg, detail::rate_key));
    }

    if (pmt::dict_has_key(msg, detail::bandwidth_key))
    {
        set_tx_bandwidth(detail::command_number(msg, detail::bandwidth_key));
    }

    if (pmt::dict_has_key(msg, detail::attenuation_key))
    {
        set_tx_attenuation(detail::command_number(msg, detail::attenuation_key));
    }
}

/* start streaming */
bool sidekiq_tx_impl::start()
{
    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
    if (tx_streaming) return block::start();
    if (reset_timestamps_on_start) {
        const auto status = skiq_reset_timestamps(card);
        if (status != 0) {
            d_logger->error("Error: could not reset timestamps, status {}", status);
            throw std::runtime_error("Failure: skiq_reset_timestamps");
        }
    }
    tx_buffers->restart();
    burst_remaining = 0;
    burst_packet.clear();
    sob_eob_burst_active = false;
    next_tx_timestamp = 0;
    last_num_tx_errors = 0;
    if (burst_tag_name.empty() && !timed_tx) {
        const auto status = skiq_start_tx_streaming(card, hdl);
        if (status != 0) {
            d_logger->error("Error: could not start TX streaming, status {}", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
        tx_streaming = true;
        return block::start();
    }
    return block::start();
}

/* Stop is an abort, not a guarantee that queued samples have aired. */
bool sidekiq_tx_impl::stop()
{
    boost::this_thread::disable_interruption no_interruption;
    tx_buffers->cancel();
    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
    if (tx_streaming) {
        const auto status = skiq_stop_tx_streaming(card, hdl);
        if (status != 0) {
            // GNU Radio calls stop from block_executor's destructor. Throwing
            // here can terminate the process; leave state set for a later retry.
            d_logger->error("Error: could not stop TX streaming, status {}", status);
            return false;
        }
        tx_streaming = false;
    }
    const auto completion_error = tx_buffers->error();
    if (completion_error != 0)
        d_logger->error("Error: asynchronous TX transfer failed, status {}", completion_error);
    return block::stop() && completion_error == 0;
}

/* set the sample rate
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_sample_rate(double value)
{
    double actual_rate;
    uint32_t requested_rate, requested_bw, actual_bw;
    auto new_rate = static_cast<uint32_t>(value);
    uint32_t new_bw = 0;
    skiq_param_t params;
    int param_idx = -1;
    int status = 0;
    d_logger->debug("in set_tx_sample_rate() ");

    status = skiq_read_parameters(card, &params);
    if (status != 0)
    {
        d_logger->error( "Error: could not read parameters, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    try
    {
        param_idx = find_tx_param_index(params, hdl);
    }
    catch (const std::exception& ex)
    {
        d_logger->error("Error: {}", ex.what());
        throw std::runtime_error("Failure: set samplerate");
    }

    if ((new_rate < params.tx_param[param_idx].sample_rate_min) ||
        (new_rate > params.tx_param[param_idx].sample_rate_max))
    {
        d_logger->error( "Error: Invalid sample rate requested: {}  Must be {} - {} Hz",
                         new_rate, params.tx_param[param_idx].sample_rate_min,
			 params.tx_param[param_idx].sample_rate_max);
        throw std::runtime_error("Failure: set samplerate");
    }

    status = skiq_read_tx_sample_rate_and_bandwidth(card, hdl,
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    new_bw = std::min(actual_bw, new_rate);

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, new_rate, new_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not set sample_rate, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    status = skiq_read_tx_sample_rate_and_bandwidth(card, hdl,
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    this->sample_rate = static_cast<uint32_t>(actual_rate);
    this->bandwidth = actual_bw;
}

/* set the bandwidth
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_bandwidth(double value)
{
    int status = 0;
    double actual_rate;
    uint32_t requested_rate, requested_bw, actual_bw;
    auto new_bw = static_cast<uint32_t>(value);

    d_logger->debug("in set_tx_bandwidth() ");

    status = skiq_read_tx_sample_rate_and_bandwidth(card, hdl,
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl, status {}, {}",
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, static_cast<uint32_t>(actual_rate), new_bw);
    if (status != 0)
    {
        d_logger->error("Error: could not set bandwidth {} on hdl, status {}, {}",
                new_bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set bandwidth");
    }

    status = skiq_read_tx_sample_rate_and_bandwidth(card, hdl,
		                                    &requested_rate, &actual_rate,
						    &requested_bw, &actual_bw);
    if (status != 0)
    {
        d_logger->error( "Error: could not read sr/bw on hdl, status {}, {}",
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

/* set the LO frequency
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_frequency(double value)
{
    int status = 0;
    d_logger->debug("in set_tx_frequency() ");

    auto freq = static_cast<uint64_t>(value);

    status = skiq_write_tx_LO_freq(card, hdl, freq);
    if (status != 0)
    {
        d_logger->error("Error: could not set frequency {}, status {}, {}",
                freq, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    this->frequency = freq;
}

/* set the attenuation
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_attenuation(double value)
{
    int status = 0;
    d_logger->debug("in set_tx_attenuation() ");

    auto att = static_cast<uint32_t>(value);

    status = skiq_write_tx_attenuation(card, hdl, att);
    if (status != 0)
    {
        d_logger->error( "Error: could not set TX attenuation to {} with status {}, {}",
                att, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
    }
    this->attenuation = att;
}

/* set the cal_mode
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_cal_mode(int value)
{
    int status = 0;
    auto cal_mode = static_cast<skiq_tx_quadcal_mode_t>(value);
    d_logger->debug("in set_tx_cal_mode() ");

    // configure the calibration mode
    status = skiq_write_tx_quadcal_mode( card, hdl, cal_mode );
    if ( 0 != status )
    {
        d_logger->error( "Error: unable to configure quadcal mode with {}", status);
        throw std::runtime_error("Failure: skiq_write_tx_quadcal_mode");
    }

    this->calibration_mode = cal_mode;

}

/* run tx calibration
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::run_tx_cal(int value)
{
    int status = 0;

    if (value == cal_on )
    {
        if (calibration_mode == skiq_tx_quadcal_mode_manual)
        {
            d_logger->info("Info: forcing calibration to run");
            status = skiq_run_tx_quadcal( card, hdl );
            if( status != 0 )
            {
                d_logger->error( "Error: calibration failed to run properly ({})", status);
                throw std::runtime_error("Failure: skiq_run_tx_quadcal");
            }
        }
        else
        {
                d_logger->info("Info: calibration cannot run, check mode");
        }
    }
}

/* GNURadio will call this before each "work()" call.  It tells them the minimum size of the
 * buffer they can send us send with samples.
 */
void sidekiq_tx_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{

    (void)(noutput_items);
    ninput_items_required[0] = (burst_tag_name.empty() && !timed_tx) ? tx_buffer_size : 1;
}

/* This will determine if we received any more underruns/late timestamps than already reported
 * This is called after a defined number of samples are handled.
 * That way it is like a timer going off.
 */
void sidekiq_tx_impl::update_tx_error_count() {
    int status = 0;
    uint32_t num_tx_errors;

    status = timed_tx ? skiq_read_tx_num_late_timestamps(card, hdl, &num_tx_errors)
                      : skiq_read_tx_num_underruns(card, hdl, &num_tx_errors);
    if (status != 0)
    {
        d_logger->error("Error: unable to read TX {} count, status {}",
                        timed_tx ? "late timestamp" : "underrun", status);
        throw std::runtime_error("Failure: read TX error count");
    }

    if (last_num_tx_errors != num_tx_errors)
    {
        d_logger->warn("TX {} count: {}", timed_tx ? "late timestamp" : "underrun",
                       num_tx_errors);
        last_num_tx_errors = num_tx_errors;
	}
}

// Submit one SDK packet. Padding is physical zero IQ, not an empty packet.
void sidekiq_tx_impl::submit_packet(const gr_complex* input, size_t count)
{
    auto buffer = tx_buffers->acquire(curr_block);
    if (!buffer) throw boost::thread_interrupted();
    auto* payload = buffer.block()->data;
    std::fill_n(payload, 2 * tx_buffer_size * (dual_channel_packet ? 2 : 1), int16_t{0});
    if (dual_channel_packet) payload += 2 * tx_buffer_size;
    if (timed_tx) skiq_tx_set_block_timestamp(buffer.block(), next_tx_timestamp);
    volk_32f_s32f_multiply_32f(reinterpret_cast<float*>(temp_buffer.data()),
                             reinterpret_cast<const float*>(input), dac_scaling,
                             static_cast<unsigned int>(2 * count));
    volk_32fc_convert_16ic(reinterpret_cast<lv_16sc_t*>(payload),
                          reinterpret_cast<const lv_32fc_t*>(temp_buffer.data()), count);
    for (;;) {
        boost::this_thread::interruption_point();
        // Capture before submission: an async callback may run before the SDK
        // call returns. Only an accepted packet transfers ownership to the SDK.
        const auto generation = tx_buffers->generation();
        int status;
        {
            std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
            if (tx_buffers->stopping()) throw boost::thread_interrupted();
            status = skiq_transmit(card, hdl, buffer.block(),
                                   in_async_mode ? buffer.context() : nullptr);
        }
        if (status == 0) {
            if (in_async_mode) buffer.handoff();
            if (timed_tx) next_tx_timestamp += tx_buffer_size;
            break;
        }
        if (status != SKIQ_TX_ASYNC_SEND_QUEUE_FULL)
            throw std::runtime_error("Failure: skiq_transmit, status " + std::to_string(status));
        tx_buffers->wait_for_completion(generation);
    }
    curr_block = (curr_block + 1) % num_blocks;
}

void sidekiq_tx_impl::finish_burst()
{
    // A normal burst boundary must not cancel outstanding host transfers.
    // This is a transport drain, not a guarantee of completion at the antenna.
    if (!tx_buffers->drain()) throw boost::thread_interrupted();
    // The SDK clears this counter when timestamped streaming stops.
    if (timed_tx) update_tx_error_count();
    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
    if (tx_buffers->stopping()) throw boost::thread_interrupted();
    const auto status = skiq_stop_tx_streaming(card, hdl);
    if (status != 0)
        throw std::runtime_error("Failure: burst stop, status " + std::to_string(status));
    tx_streaming = false;
}

uint64_t sidekiq_tx_impl::parse_tx_time(const pmt_t& value) const
{
    if (!pmt::is_tuple(value) || pmt::length(value) != 2)
        throw std::runtime_error("tx_time must be a (uint64 seconds, real fractional seconds) tuple");
    const auto seconds_value = pmt::tuple_ref(value, 0);
    const auto fraction_value = pmt::tuple_ref(value, 1);
    uint64_t seconds;
    if (pmt::is_uint64(seconds_value)) {
        seconds = pmt::to_uint64(seconds_value);
    } else if (pmt::is_integer(seconds_value) && pmt::to_long(seconds_value) >= 0) {
        seconds = static_cast<uint64_t>(pmt::to_long(seconds_value));
    } else {
        throw std::runtime_error("tx_time seconds must be a non-negative integer");
    }
    if (!pmt::is_real(fraction_value))
        throw std::runtime_error("tx_time fractional seconds must be real");
    const double fraction = pmt::to_double(fraction_value);
    if (!std::isfinite(fraction) || fraction < 0.0 || fraction >= 1.0)
        throw std::runtime_error("tx_time fractional seconds must be in [0, 1)");
    if (seconds > std::numeric_limits<uint64_t>::max() / sample_rate)
        throw std::runtime_error("tx_time is outside the RF timestamp range");
    uint64_t timestamp = seconds * sample_rate;
    auto fractional_samples = static_cast<uint64_t>(std::llround(fraction * sample_rate));
    if (fractional_samples == sample_rate) {
        if (timestamp > std::numeric_limits<uint64_t>::max() - sample_rate)
            throw std::runtime_error("tx_time is outside the RF timestamp range");
        fractional_samples = 0;
        timestamp += sample_rate;
    }
    if (timestamp > std::numeric_limits<uint64_t>::max() - fractional_samples)
        throw std::runtime_error("tx_time is outside the RF timestamp range");
    return timestamp + fractional_samples;
}

void sidekiq_tx_impl::start_burst(uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
    if (tx_buffers->stopping()) throw boost::thread_interrupted();
    const auto status = skiq_start_tx_streaming(card, hdl);
    if (status != 0)
        throw std::runtime_error("Failure: burst start, status " + std::to_string(status));
    tx_streaming = true;
    next_tx_timestamp = timestamp;
}

int sidekiq_tx_impl::work_bursts(int count, const gr_complex* input)
{
    std::vector<tag_t> tags;
    std::vector<tag_t> time_tags;
    // GNU Radio tag offsets are absolute; consumed is relative to this call.
    // Persistent burst_packet/burst_remaining bridge scheduler boundaries.
    const auto base = nitems_read(0);
    get_tags_in_range(tags, 0, base, base + count, pmt::intern(burst_tag_name));
    if (timed_tx)
        get_tags_in_range(time_tags, 0, base, base + count, pmt::intern("tx_time"));
    std::stable_sort(tags.begin(), tags.end(), [](const tag_t& a, const tag_t& b) {
        return a.offset < b.offset;
    });
    std::stable_sort(time_tags.begin(), time_tags.end(), [](const tag_t& a, const tag_t& b) {
        return a.offset < b.offset;
    });
    size_t next = 0;
    size_t next_time = 0;
    int consumed = 0;
    while (consumed < count) {
        boost::this_thread::interruption_point();
        const auto offset = base + consumed;
        if (timed_tx && next_time < time_tags.size() && time_tags[next_time].offset == offset &&
            (next >= tags.size() || tags[next].offset != offset))
            throw std::runtime_error("tx_time tag must coincide with a TX burst length tag");
        if (next < tags.size() && tags[next].offset == offset) {
            if (burst_remaining)
                throw std::runtime_error("Overlapping TX burst tags");
            const auto value = tags[next++].value;
            uint64_t length = 0;
            if (pmt::is_uint64(value)) length = pmt::to_uint64(value);
            else if (pmt::is_integer(value) && pmt::to_long(value) > 0)
                length = static_cast<uint64_t>(pmt::to_long(value));
            if (!length) throw std::runtime_error("TX burst length must be a positive integer");
            if (next < tags.size() && tags[next].offset == offset)
                throw std::runtime_error("Duplicate TX burst tags");
            uint64_t timestamp = 0;
            if (timed_tx) {
                if (next_time >= time_tags.size() || time_tags[next_time].offset != offset)
                    throw std::runtime_error("Timed TX burst is missing a tx_time tag");
                timestamp = parse_tx_time(time_tags[next_time++].value);
                if (next_time < time_tags.size() && time_tags[next_time].offset == offset)
                    throw std::runtime_error("Duplicate tx_time tags");
            }
            start_burst(timestamp);
            burst_remaining = length;
        }
        auto boundary = next < tags.size() ? tags[next].offset : base + count;
        if (timed_tx && next_time < time_tags.size())
            boundary = std::min(boundary, time_tags[next_time].offset);
        const auto available = boundary - offset;
        if (!burst_remaining) {
            consumed += available; // Samples outside tagged bursts are discarded.
            continue;
        }
        // Stop at the next tag, the burst end, or the SDK packet boundary.
        // Padding added by submit_packet must never consume the next input burst.
        const auto take = std::min<uint64_t>(
            std::min<uint64_t>(available, burst_remaining), tx_buffer_size - burst_packet.size());
        burst_packet.insert(burst_packet.end(), input + consumed, input + consumed + take);
        consumed += take;
        burst_remaining -= take;
        if (burst_packet.size() == static_cast<size_t>(tx_buffer_size) || !burst_remaining) {
            submit_packet(burst_packet.data(), burst_packet.size());
            burst_packet.clear();
        }
        if (!burst_remaining) finish_burst();
    }
    return consumed;
}

int sidekiq_tx_impl::work_sob_eob_bursts(int count, const gr_complex* input)
{
    std::vector<tag_t> tags;
    const auto base = nitems_read(0);
    get_tags_in_range(tags, 0, base, base + count);
    tags.erase(std::remove_if(tags.begin(), tags.end(), [](const tag_t& tag) {
        return !pmt::eq(tag.key, pmt::intern("tx_sob")) &&
               !pmt::eq(tag.key, pmt::intern("tx_eob")) &&
               !pmt::eq(tag.key, pmt::intern("tx_time"));
    }), tags.end());
    std::stable_sort(tags.begin(), tags.end(), [](const tag_t& a, const tag_t& b) {
        return a.offset < b.offset;
    });

    size_t next = 0;
    int consumed = 0;
    while (consumed < count) {
        boost::this_thread::interruption_point();
        const auto offset = base + consumed;
        bool sob = false;
        bool eob = false;
        size_t sob_count = 0;
        size_t eob_count = 0;
        size_t time_count = 0;
        pmt_t time_value = pmt::PMT_NIL;
        while (next < tags.size() && tags[next].offset == offset) {
            const auto& tag = tags[next++];
            if (pmt::eq(tag.key, pmt::intern("tx_time"))) {
                ++time_count;
                time_value = tag.value;
            } else {
                if (!pmt::is_bool(tag.value))
                    throw std::runtime_error("tx_sob and tx_eob tag values must be boolean");
                const bool enabled = pmt::to_bool(tag.value);
                if (pmt::eq(tag.key, pmt::intern("tx_sob"))) {
                    if (enabled) ++sob_count;
                    sob = sob || enabled;
                } else {
                    if (enabled) ++eob_count;
                    eob = eob || enabled;
                }
            }
        }
        if (sob_count > 1) throw std::runtime_error("Duplicate tx_sob tags");
        if (eob_count > 1) throw std::runtime_error("Duplicate tx_eob tags");

        if (!sob_eob_burst_active) {
            if (eob && !sob) throw std::runtime_error("tx_eob received outside a TX burst");
            if (time_count && !sob) throw std::runtime_error("tx_time tag must coincide with tx_sob");
            if (sob) {
                if (time_count != 1)
                    throw std::runtime_error(time_count ? "Duplicate tx_time tags"
                                                        : "Timed TX burst is missing a tx_time tag");
                start_burst(parse_tx_time(time_value));
                sob_eob_burst_active = true;
            }
        } else {
            if (sob) throw std::runtime_error("Overlapping tx_sob tags");
            if (time_count) throw std::runtime_error("tx_time tag must coincide with tx_sob");
        }

        const auto boundary = next < tags.size() ? tags[next].offset : base + count;
        if (!sob_eob_burst_active) {
            consumed += boundary - offset;
            continue;
        }
        const auto available = eob ? uint64_t{1} : boundary - offset;
        const auto take = std::min<uint64_t>(available, tx_buffer_size - burst_packet.size());
        burst_packet.insert(burst_packet.end(), input + consumed, input + consumed + take);
        consumed += take;
        if (burst_packet.size() == static_cast<size_t>(tx_buffer_size) || eob) {
            submit_packet(burst_packet.data(), burst_packet.size());
            burst_packet.clear();
        }
        if (eob) {
            finish_burst();
            sob_eob_burst_active = false;
        }
    }
    return consumed;
}

int sidekiq_tx_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    (void)output_items;
    const auto* input = static_cast<const gr_complex*>(input_items[0]);
    int consumed = 0;
    if (!burst_tag_name.empty()) {
        consumed = work_bursts(noutput_items, input);
    } else if (timed_tx) {
        consumed = work_sob_eob_bursts(noutput_items, input);
    } else {
        // Preserve immediate mode's complete-packet consumption contract.
        while (noutput_items - consumed >= tx_buffer_size) {
            submit_packet(input + consumed, tx_buffer_size);
            consumed += tx_buffer_size;
        }
    }
    if (nitems_read(0) + consumed - last_status_update_sample > status_update_rate_in_samples) {
        update_tx_error_count();
        last_status_update_sample = nitems_read(0) + consumed;
    }
    return consumed;
}

} // namespace sidekiq
} // namespace gr
