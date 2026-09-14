/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <algorithm>

#include "sidekiq_handle_utils.h"
#include "sidekiq_tx_impl.h"


#define DEBUG_LEVEL "debug"  //Can be debug, info, warning, error, critical

namespace gr {
namespace sidekiq {

using input_type = float;

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
                                  int cal_mode)
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
                            cal_mode);
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
                                  int cal_mode)
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
                                  cal_mode);
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
                                  int cal_mode)
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
                            cal_mode);
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
                                  int cal_mode)
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
                            cal_mode);
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
                                  int cal_mode)
    : gr::sync_block("sidekiq_tx",
                     gr::io_signature::make( 1 /* min inputs */, 1 /* max inputs */, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0))   //sync block
{
    std::string str;

    d_logger->set_level(DEBUG_LEVEL);
    d_logger->get_level(str);

    printf("in TX constructor, debug level:%s\n", str.c_str());
    
    int status = 0;
    skiq_param_t param;
    uint8_t iq_resolution = 0;
    status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

    card = input_card;
    hdl = (skiq_tx_hdl_t)handle;
    curr_block = 0;
    tx_buffer_size = buffer_size;
    num_blocks = NUM_BLOCKS;

    burst_tag_name = burst_tag;
    d_logger->debug("burst_tag_name: {}", burst_tag_name);   

    if( 0 == burst_tag_name.compare("") )
    {
        bursting_cmd = NO_BURSTING_ENABLED;
    }
    else
    {
        bursting_cmd = BURSTING_OFF;
    }

    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0) 
    {
        if (status != -EEXIST)
        {
            d_logger->error( "Error: unable to initialize libsidekiq with status {}", status);
            throw std::runtime_error("Failure: skiq_init");
        }
        else 
        {
            d_logger->info("Info: If not running Transceive Mode, then this is an error");
            tx_second = true;
        }
    }
    else
    {
        libsidekiq_init = true;
        d_logger->info("Info: libsidkiq initialized successfully");

    }

    status = skiq_read_parameters(card, &param);
    if (status != 0)
    {
        d_logger->error( "Error: unable to read card parameters with status {}", status);
        throw std::runtime_error("Failure: skiq_read_parameters");
    }
    card_part = param.card_param.part_type;

    /* set topology if it has changed from default (0) */
    if (topology != 0)
    {
        if (skiq_is_topology_supported(card))
        {
            status = skiq_apply_topology(card, topology);
            if (status != 0)
            {
                d_logger->error( "Error: unable to configure topology %d with status {}",
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

    if (tx_second == false)
    {
        set_tx_sample_rate(sample_rate);
        set_tx_bandwidth(bandwidth);
    }

    status = skiq_read_tx_iq_resolution(card, &iq_resolution);
    if (status != 0) 
    {
        d_logger->error( "Error: unable to get iq resolution with status {}", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    dac_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    d_logger->info("Info: dac scaling {}", dac_scaling);

    /* always use immediate mode */
    status = skiq_write_tx_data_flow_mode(card, hdl, skiq_tx_immediate_data_flow_mode);
    if (status != 0) 
    {
        d_logger->error( "Error: could not set TX dataflow mode with status {}", status);
        throw std::runtime_error("Failure: skiq_write_tx_flow_mode");
    }

    if ( !skiq_is_topology_supported(card) ||
         card_part == skiq_nv100 || card_part == skiq_nvm2)
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
    status = skiq_write_iq_pack_mode(card, SIDEKIQ_IQ_PACK_MODE_UNPACKED);
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

    message_port_register_in(CONTROL_MESSAGE_PORT);
    set_msg_handler(CONTROL_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });

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

    /* disable libsidekiq */
    if (libsidekiq_init == true)
    {
        skiq_exit();
    }
}

double sidekiq_tx_impl::get_double_from_pmt_dict(pmt_t dict, pmt_t key, pmt_t not_found = pmt::PMT_NIL) 
{
    auto message_value = pmt::dict_ref(dict, key, not_found);

    return pmt::to_double(message_value);
}


void sidekiq_tx_impl::handle_control_message(pmt_t msg) 
{
    d_logger->debug("in handle_control ");

    // pmt_dict is a subclass of pmt_pair. Make sure we use pmt_pair!
    // Old behavior was that these checks were interchangeable. Be aware of this change!
    if (!(pmt::is_dict(msg)) && pmt::is_pair(msg)) {
        d_logger->debug(
            "Command message is pair, converting to dict: '{}': car({}), cdr({})",
            pmt::write_string(msg),
            pmt::write_string(pmt::car(msg)),
            pmt::write_string(pmt::cdr(msg)));
        msg = pmt::dict_add(pmt::make_dict(), pmt::car(msg), pmt::cdr(msg));
     }

     // Make sure, we use dicts!
     if (!pmt::is_dict(msg)) {
         d_logger->error("Command message is neither dict nor pair: {}", pmt::write_string(msg));
         return;
     }

    if (pmt::dict_has_key(msg, LO_FREQ_KEY)) 
    {
        set_tx_frequency(get_double_from_pmt_dict(msg, LO_FREQ_KEY));
    }

    if (pmt::dict_has_key(msg, RATE_KEY)) 
    {
        set_tx_sample_rate(get_double_from_pmt_dict(msg, RATE_KEY));
    }

    if (pmt::dict_has_key(msg, BANDWIDTH_KEY)) 
    {
        set_tx_bandwidth(get_double_from_pmt_dict(msg, BANDWIDTH_KEY));
    }

    if (pmt::dict_has_key(msg, ATTENUATION_KEY)) 
    {
        set_tx_attenuation(get_double_from_pmt_dict(msg, ATTENUATION_KEY));
    }
}


/* start streaming */
bool sidekiq_tx_impl::start()
{
    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
    if (tx_streaming) return block::start();
    tx_buffers->restart();
    if (bursting_cmd == BURSTING_ON || bursting_cmd == NO_BURSTING_ENABLED) {
        const auto status = skiq_start_tx_streaming(card, hdl);
        if (status != 0) {
            d_logger->error("Error: could not start TX streaming, status {}", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
        tx_streaming = true;
        return block::start();
    }
    return false;
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
        return;
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
        return;
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
        return;
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

    if (value == CAL_ON )
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
    ninput_items_required[0] = tx_buffer_size;
}

/* This will determine if we received any more underruns than already reported 
 * This is called after a defined number of samples are handled.
 * That way it is like a timer going off.
 */
void sidekiq_tx_impl::update_tx_error_count() {
    int status = 0;
    uint32_t num_tx_errors;


    status =  skiq_read_tx_num_underruns(card, hdl, &num_tx_errors);
    if (status != 0)
    {
        d_logger->error( "Error: skiq_read_tx_num_underruns failed with status {} ", status);
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
        return;
    }

    if (last_num_tx_errors != num_tx_errors) 
    {
        printf("TX underrun count: %u\n", num_tx_errors);
        last_num_tx_errors = num_tx_errors;
	}
}

int sidekiq_tx_impl::handle_tx_burst_tag(tag_t tag) 
{
    if (bursting_cmd != NO_BURSTING_ENABLED)
    {
        /* old way does not compile anymore */
#ifdef OLDWAY
        d_logger->debug("in handle_tx_burst_tag, tag offset {}, cmd {}, length {}", 
                tag.offset, bursting_cmd, tag.value);
#endif
        d_logger->debug("in handle_tx_burst_tag, tag offset {:d}, cmd {}, length {:d}", 
                tag.offset, bursting_cmd, pmt::write_string(tag.value));

        burst_length = pmt::to_uint64(tag.value);
        burst_samples_sent = 0;
        bursting_cmd = BURSTING_ON;

        if (tx_streaming == false)
        {
            start();
        }

        return burst_length;
    }
    else
    {
        return 0;
    }
}



/* This is called by GNURadio when it has received a buffer of samples to be transmitted. */
int sidekiq_tx_impl::work(
		int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) 
{
	int32_t status{};
	int32_t samples_written{};
    int32_t ninput_items{};
    std::vector<tag_t> tags;

    (void)(output_items);

    /* get a pointer to the buffer with the samples to be transmitted */
    auto in = static_cast<const gr_complex *>(input_items[0]);

    /* noutput_items should always be larger than tx_buffer_size 
     * because we did the "forecast" function */
    if ( noutput_items >= tx_buffer_size)
    {
         /* get the size of the input aligned to our buffer size */
	     ninput_items = noutput_items - (noutput_items % tx_buffer_size);
    }
    else
    {

        d_logger->error( "Error: noutput_items {} is smaller than tx_buffer_size {}", 
                noutput_items, tx_buffer_size);
        throw std::runtime_error("Failure: input items too small");
    }

    pmt_t tx_burst_key{pmt::string_to_symbol(burst_tag_name)};

    /* see if we received the TX_BURST tag, if so process it */
    get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items);
    if (not tags.empty())
    {
        BOOST_FOREACH( const tag_t &tag, tags) 
        {
            if (pmt::equal(tag.key, tx_burst_key))
            {
                handle_tx_burst_tag(tag);
            }
        }
    }

    if (bursting_cmd == BURSTING_OFF)
    {
        // We are not transmitting yet
        return noutput_items;
    }

    int32_t samples_to_write = tx_buffer_size;

    /* if we are streaming in bursts, tx_streaming goes on and off */
    if (tx_streaming)
    {
        /* loop until we have sent the samples we have been given */
        while (samples_written < ninput_items) 
        {
            /* if we are bursting then we need to only send the amount of samples in the burst */
            if (burst_length != 0)
            {
                uint64_t delta = burst_length - burst_samples_sent;

                /* if this number is smaller than our buffer size, we need to send only the delta */
                if (delta < (uint64_t)tx_buffer_size)
                {
                   samples_to_write = delta;
                }
                else 
                {
                    samples_to_write = tx_buffer_size;
                }
            }
            else
            {
                samples_to_write = tx_buffer_size;
            }

            auto buffer = tx_buffers->acquire(curr_block);
            if (!buffer) return samples_written ? samples_written : WORK_DONE;
            auto* payload = buffer.block()->data;
            if (dual_channel_packet) {
                // Silence the paired A1/B1 channel; the input belongs to A2/B2.
                std::fill_n(payload, 2 * tx_buffer_size, int16_t{0});
                payload += 2 * tx_buffer_size;
            }

            /* convert the samples we have received to be within the dac_scaling values */
            volk_32f_s32f_multiply_32f(
                    reinterpret_cast<float *>(&temp_buffer[0]),
                    reinterpret_cast<const float *>(in),
                    dac_scaling,
                    static_cast<unsigned int>(samples_to_write * 2));

            /* convert those samples from float complex to int16 */
            volk_32fc_convert_16ic(
                    reinterpret_cast<lv_16sc_t *>(payload),
                    reinterpret_cast<const lv_32fc_t*>(&temp_buffer[0]),
                    samples_to_write);
            

            // Reserve before calling the SDK, which can invoke the callback
            // inline. Never hold the pool mutex across that call.
            for (;;) {
                boost::this_thread::interruption_point();
                const auto generation = tx_buffers->generation();
                {
                    std::lock_guard<std::mutex> lock(tx_lifecycle_mutex);
                    if (tx_buffers->stopping())
                        return samples_written ? samples_written : WORK_DONE;
                    status = skiq_transmit(card, hdl, buffer.block(),
                                          in_async_mode ? buffer.context() : nullptr);
                }
                if (status == 0) {
                    if (in_async_mode) buffer.handoff();
                    break;
                }
                if (status != SKIQ_TX_ASYNC_SEND_QUEUE_FULL) {
                    d_logger->error("Error: sidekiq transmit failed, status {}", status);
                    throw std::runtime_error("Failure: skiq_transmit");
                }
                tx_buffers->wait_for_completion(generation);
            }
            // Only accepted samples advance input/burst counters. A rejected
            // packet is retried unchanged, using the same buffer.
            samples_written += samples_to_write;
            in += samples_to_write;
            curr_block = (curr_block + 1) % num_blocks;

            /* if we are bursting, check to see if we are done */
            if (burst_length != 0)
            {
                burst_samples_sent += samples_to_write;
                if (burst_samples_sent >= burst_length) 
                {
                    d_logger->debug("done bursting, sent {}, length {} stop streaming", burst_samples_sent, burst_length);
                    burst_length = 0;
                    burst_samples_sent = 0;
                    stop();
                    bursting_cmd = BURSTING_OFF;
                    break;
                }
            }
        }

        /* Determine if the time has elapsed and display any underruns we have received */
        if (nitems_read(0) - last_status_update_sample > status_update_rate_in_samples) 
        {
            update_tx_error_count();
            last_status_update_sample = nitems_read(0);


            d_logger->debug("noutput_items {}, tx_buffer_size {}, sample_written {}", 
                    noutput_items, tx_buffer_size, samples_written);
        }
    }

    /* if we are bursting and we have not written anything we need to lie and say we did.  Otherwise 
     * the flowchart stops sending samples */
    if (samples_written == 0)
    {
        samples_written = ninput_items;
    }

	
	return samples_written;
}


} /* namespace sidekiq */
} /* namespace gr */
