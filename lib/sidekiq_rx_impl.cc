/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */


#include "sidekiq_rx_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <boost/asio.hpp>
#include <chrono>

#define DEBUG_LEVEL "debug" //Can be debug, info, warning, error, critical

//#define COUNTER             // used for data integrity testing

using pmt::pmt_t;
const pmt_t CONTROL_MESSAGE_PORT{pmt::string_to_symbol("command")};

namespace gr {
namespace sidekiq {

using output_type = float;
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
  return gnuradio::make_block_sptr<sidekiq_rx_impl>(
          input_card,
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

sidekiq_rx_impl::sidekiq_rx_impl(
        int input_card,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int timestamp_tags,
        int local_trigger_src,
        int local_pps_source,
        int cal_mode,
        int cal_type) 
    : gr::sync_block("sidekiq_rx", gr::io_signature::make(0, 0, 0),
                                   gr::io_signature::make(1 /* min outputs */, 2 /*max outputs */,
                                            sizeof(gr_complex))) 
{
    std::string str;

    /* the goal is to use the debugger for debug info, but not normal Info or Errors 
     * this will allow us to turn it on and off.  But Info and Errors look better with printf */
    d_logger->set_level(DEBUG_LEVEL);
    d_logger->get_level(str);

    printf("in constructor RX, debug level: %s\n", str.c_str());

    int status = 0;
    uint8_t iq_resolution = 0;
    status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

    curr_rf_block_tag.key = pmt::intern("rf_timestamp");
    curr_rf_block_tag.value = pmt::from_uint64(0);

    this->timestamp_tags = timestamp_tags;
    card = input_card;
    hdl1 = (skiq_rx_hdl_t) port1_handle;
    this->card = input_card;
    this->hdl1 = (skiq_rx_hdl_t) port1_handle;



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

    d_logger->debug("trigger {}, pps_source {}", this->trigger_src, this->pps_source);

    /* determine if we are in dual port */
    if (port2_handle < skiq_rx_hdl_end)
    {
        this->dual_port = true;
        this->hdl2 = (skiq_rx_hdl_t) port2_handle;
    }
    else
    {
        this->hdl2 = skiq_rx_hdl_end;
        this->dual_port = false;
    }

    /* initialize libsidekiq */
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
            rx_second = true;
        }
    }
    else
    {
        libsidekiq_init = true;
        d_logger->info("Info: libsidkiq initialized successfully");
    }

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
            d_logger->info("Info: configured 1PPS source to {}", pps_source);
        }
      }


#ifdef COUNTER
    skiq_write_rx_data_src(card, hdl1, skiq_data_src_counter);
#endif

    /* calculate the adc scaling */
    status = skiq_read_rx_iq_resolution(card, &iq_resolution);
    if (status != 0)
    {
        d_logger->error( "Error: unable to get iq resolution with status {}", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    adc_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    d_logger->info("Info: ADC scaling {}", adc_scaling);


    /* if A2 or B2 is used, we need to set the channel mode to dual */
    if (hdl1 == skiq_rx_hdl_A2 || hdl1 == skiq_rx_hdl_B2 || 
            hdl2 == skiq_rx_hdl_A2 || hdl2 == skiq_rx_hdl_B2)
    {
        status = skiq_write_chan_mode(card, skiq_chan_mode_dual);
        if (status != 0)
        {
            d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    }
    else {
        status = skiq_write_chan_mode(card, skiq_chan_mode_single);
        if (status != 0)
        {
            d_logger->error( "Error: unable to configure TX channel mode with status {}", status);
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
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

    /* support two messages */
    message_port_register_in(CONTROL_MESSAGE_PORT);
    set_msg_handler(CONTROL_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });

    /* set the rest of the parameters */
    set_rx_frequency(frequency);
    set_rx_gain_mode(gain_mode);

    if (gain_mode == skiq_rx_gain_manual)
    {
        set_rx_gain_index(gain_index);
    }

    set_rx_cal_mode(cal_mode);
    set_rx_cal_type(cal_type);

#ifdef COUNTER
    skiq_write_rx_data_src(card, hdl1, skiq_data_src_counter);
#endif

    /* we need gnuradio to send in buffers of an integer multiple of our DMA block sizes */
    gr::block::set_min_noutput_items(DATA_MAX_BUFFER_SIZE);
    gr::block::set_output_multiple(DATA_MAX_BUFFER_SIZE);

    last_time = Clock::now();


}


/* deconstructor */
sidekiq_rx_impl::~sidekiq_rx_impl() 
{
    d_logger->debug("in RX deconstructor");

    if (rx_streaming == true)
    {
        stop();
        rx_streaming = false;
    }

    if (libsidekiq_init == true)
    {
        skiq_exit();
        libsidekiq_init = false;
    }
}



double sidekiq_rx_impl::get_double_from_pmt_dict(pmt_t dict, pmt_t key, pmt_t not_found = pmt::PMT_NIL) {
    auto message_value = pmt::dict_ref(dict, key, not_found);

    return pmt::to_double(message_value);
}

/*
 * Handle control messages
 *
 */
void sidekiq_rx_impl::handle_control_message(pmt_t msg) 
{
    d_logger->debug("in handle_control_message");

    // pmt_dict is a subclass of pmt_pair. Make sure we use pmt_pair!
    // Old behavior was that these checks were interchangeable. Be aware of this change!
    if (!(pmt::is_dict(msg)) && pmt::is_pair(msg)) {
        d_logger->debug(
            "Command message is pair, converting to dict: '{}': car({}), cdr({})",
            /* old way of doing it, doesn't compile now */
#ifdef OLDWAY
            msg,
            pmt::car(msg),
            pmt::cdr(msg));
#endif
            pmt::write_string(msg),
            pmt::write_string(pmt::car(msg)),
            pmt::write_string(pmt::cdr(msg)));
        msg = pmt::dict_add(pmt::make_dict(), pmt::car(msg), pmt::cdr(msg));
     }

     // Make sure, we use dicts!
     if (!pmt::is_dict(msg)) {
         d_logger->error("Command message is neither dict nor pair: {}", msg);
         return;
     }

    if (pmt::dict_has_key(msg, LO_FREQ_KEY)) 
    {
        set_rx_frequency(get_double_from_pmt_dict(msg, LO_FREQ_KEY));
    }

    if (pmt::dict_has_key(msg, RATE_KEY)) 
    {
        set_rx_sample_rate(get_double_from_pmt_dict(msg, RATE_KEY));
    }

    if (pmt::dict_has_key(msg, BANDWIDTH_KEY)) 
    {
        set_rx_bandwidth(get_double_from_pmt_dict(msg, BANDWIDTH_KEY));
    }

    if (pmt::dict_has_key(msg, GAIN_KEY)) 
    {
        set_rx_gain_index(get_double_from_pmt_dict(msg, GAIN_KEY));
    }


}

/* 
 * start streaming
 * 
 * Called by the generated python code
 */
bool sidekiq_rx_impl::start() 
{
    int status = 0;
    skiq_rx_hdl_t handles[skiq_rx_hdl_end];
    uint8_t nrhandles = 0;

    d_logger->debug("in start");

    status = skiq_reset_timestamps(card);
    if (status != 0)
    {
        d_logger->error( "Error: could not reset timestamps, status {}", status);
        throw std::runtime_error("Failure: skiq_reset_timestamps");
    }

    handles[0] = hdl1;
    nrhandles = 1;

    if (dual_port == true)
    {
        handles[1] = hdl2;
        nrhandles = 2;
    }

    status = skiq_start_rx_streaming_multi_on_trigger(card, handles, nrhandles, trigger_src, 0);
    if ( status != 0 )
    {
       d_logger->error( "Error: could not start RX streaming on hdl1, status {}", status);
       throw std::runtime_error("Failure: skiq_start_rx_streaming");
    }

    rx_streaming = true;

    /* tag indexes are absolute starting from the first sample out
     * so they must be reset when starting a stream
     */
    tag_index[0] = 0;
    tag_index[1] = 0;

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
    skiq_rx_hdl_t handles[skiq_rx_hdl_end];
    uint8_t nrhandles = 0;
    
    d_logger->debug("in stop");

    /* only call stop if we are actually streaming */
    if (rx_streaming == true)
    {
        handles[0] = hdl1;
        nrhandles = 1;
        if (dual_port == true)
        {
            handles[1] = hdl2;
            nrhandles = 2;
        }

        status = skiq_stop_rx_streaming_multi_on_trigger(card, handles, nrhandles, trigger_src, 0);
        if ( status != 0 )
        {
           d_logger->error( "Error: could not start RX streaming on hdl1, status {}", status);
           throw std::runtime_error("Failure: skiq_start_rx_streaming");
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
void sidekiq_rx_impl::set_rx_sample_rate(double value) 
{

    int status = 0;
    d_logger->debug("in set_rx_sample_rate");

    auto rate = static_cast<uint32_t>(value);
    auto bw = static_cast<uint32_t>(this->bandwidth);

    status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl1, rate, bw); 
    if (status != 0) 
    {
        d_logger->error( "Error: could not set sample_rate on hdl1, status {}, {}", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    if (dual_port)
    {
        status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl2, rate, bw); 
        if (status != 0) 
        {
            d_logger->error( "Error: could not set sample_rate on hdl2, status {}, {}", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set samplerate");
        }
    }
    d_logger->info("Info: sample_rate set to {}", rate);

    this->sample_rate = rate;
    this->bandwidth = bw;

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
    d_logger->debug("in set_rx_bandwidth");

    auto rate = static_cast<uint32_t>(this->sample_rate);
    auto bw = static_cast<uint32_t>(value);

    status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl1, rate, bw); 
    if (status != 0) 
    {
        d_logger->error("Error: could not set bandwidth {} on hdl1, status {}, {}", 
                bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set bandwidth");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl2, rate, bw); 
        if (status != 0) 
        {
            d_logger->error("Error: could not set bandwidth {} on hdl2, status {}, {}", 
                    bw, status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set bandwidth");
            return;
        }
    }
    d_logger->info("Info: bandwidth set to {}", bw);

    this->sample_rate = rate;
    this->bandwidth = bw;

}

/* 
 * set the LO frequency
 * this may be called from the generated python code if the user changes the variable
 *
 * let libsidekiq determine if the value is valid
 */
void sidekiq_rx_impl::set_rx_frequency(double value) 
{
    int status = 0;
    d_logger->debug("in set_rx_frequency");

    auto freq = static_cast<uint64_t>(value);

    status = skiq_write_rx_LO_freq(card, hdl1, freq);
    if (status != 0) 
    {
        d_logger->error("Error: could not set frequency {} on hdl1, status {}, {}", 
                freq, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set frequency");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_LO_freq(card, hdl2, freq);
        if (status != 0) 
        {
            d_logger->error("Error: could not set frequency {} on hdl2, status {}, {}", 
                    freq, status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set frequency");
            return;
        }
    }

    d_logger->info("Info: frequency set to {}", freq);

    this->frequency = freq;
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

    status = skiq_write_rx_gain_mode(card, hdl1, gain_mode);
    if (status != 0) 
    {
        d_logger->error("Error: write_rx_gain_mode failed on hdl1, status {}, {} ", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set write_rx_gain_mode");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_gain_mode(card, hdl2, gain_mode);
        if (status != 0) 
        {
            d_logger->error("Error: write_rx_gain_mode failed on hdl2, status {}, {}", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set write_rx_gain_mode");
            return;
        }
    }

    d_logger->info("Info: gain_mode set to {}", gain_mode);

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
        status = skiq_read_rx_gain_index_range(card, hdl1, &min_range, &max_range);
        if (status != 0) 
        {
            d_logger->error("Error: read_rx_gain_index failed, status {}, {}", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set read_rx_gain_index");
            return;
        }
        d_logger->info("Info: gain range for current frequency is {} - {}", min_range, max_range);

        if (gain > max_range || gain < min_range)
        {
            d_logger->error("Error: gain_index {} is out of range", gain);
            throw std::runtime_error("Failure: gain index is out of range");
            return;
        }

        status = skiq_write_rx_gain(card, hdl1, gain);
        if (status != 0) 
        {
            d_logger->error("Error: write_rx_gain failed on hdl1, status {}, {}", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set read_rx_gain_index");
            return;
        }

        if (dual_port == true)
        {
            status = skiq_write_rx_gain(card, hdl2, gain);
            if (status != 0) 
            {
                d_logger->error("Error: write_rx_gain failed on hdl2, status {}, {}", 
                        status, strerror(abs(status)) );
                throw std::runtime_error("Failure: set read_rx_gain_index");
                return;
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
    int status = 0;

    d_logger->debug("in set_cal_mode");

    if (value == CAL_OFF)
    {
        cal_enabled = false;
        d_logger->info("Info: cal_mode set to off");
    }
    else
    {
        cal_enabled = true;
        auto cmode = static_cast<skiq_rx_cal_mode_t>(value);

        /* set the calibration mode */
        status = skiq_write_rx_cal_mode( card, hdl1, cmode );
        if( status != 0 )
        {
            if( status != -ENOTSUP )
            {
                d_logger->error( "Error: failed to configure RX calibration mode with {}", status);
                throw std::runtime_error("Failure: set rx_cal_mode");
            }
            else
            {
                d_logger->warn("Warning: calibration mode {} unsupported with product", cmode);
            }
        }

        if (dual_port == true)
        {
            status = skiq_write_rx_cal_mode( card, hdl2, cmode );
            if( status != 0 )
            {
                if( status != -ENOTSUP )
                {
                    d_logger->error( "Error: failed to configure RX calibration mode with {}", status);
                    throw std::runtime_error("Failure: set rx_cal_mode");
                }
                else
                {
                    d_logger->warn("Warning: calibration mode {} unsupported with product", cmode);
                }
            }
        }

        this->cal_mode = cmode;
        d_logger->info("Info: cal_mode set to {}", cmode);
    }

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
    int status = 0;
    uint32_t cal_mask = (uint32_t)(skiq_rx_cal_type_none);

    d_logger->debug("in set_cal_type");

    /* The cal_mask is a bitmap of the types of calibration */    
    if (cal_enabled == true)
    {
        if (value == CAL_TYPE_BOTH)
        {
            cal_mask = skiq_rx_cal_type_dc_offset | skiq_rx_cal_type_quadrature;
        } 
        else if (value == CAL_TYPE_DC_OFFSET)
        {
            cal_mask = skiq_rx_cal_type_dc_offset;
        }
        else if (value == CAL_TYPE_QUADRATURE)
        {
            cal_mask = skiq_rx_cal_type_quadrature;
        }

        /* read in what this card can handle */
        uint32_t read_cal_mask = 0;
        if( (status = skiq_read_rx_cal_types_avail( card, hdl1, &read_cal_mask )) == 0 )
        {
            if( read_cal_mask != cal_mask )
            {
                d_logger->warn("Warning: RX calibration mask available for card is (0x{:02X})" 
                       " does not match what is desired (0x{:02X})",
                       read_cal_mask, cal_mask);
                d_logger->info("Info: Setting cal_mask to 0x{:02X}", read_cal_mask);
                cal_mask = read_cal_mask;
            }
        }
        else
        {
            d_logger->error("Error: unable to read calibration mask (status={})", status);
        }

        /* write the cal mask */
        status = skiq_write_rx_cal_type_mask( card, hdl1, cal_mask );
        if( status != 0 )
        {
            d_logger->error( "Error: failed to configure RX calibration type with status {}", status);
            throw std::runtime_error("Failure: set rx_cal_type");
        }

        if (dual_port == true)
        {
            status = skiq_write_rx_cal_type_mask( card, hdl2, cal_mask );
            if( status != 0 )
            {
                d_logger->error( "Error: failed to configure RX calibration type with status {}", status);
                throw std::runtime_error("Failure: set rx_cal_type");
            }
        }

        d_logger->info("Info: rx cal_mask 0x{:02X}, written successfully", cal_mask);

    }

}

/* run_cal
 *
 * This manually runs the calibration set by the mode and type.
 *
 * this may be called from the generated python if the user changes the variable
 */
void sidekiq_rx_impl::run_rx_cal(int value) 
{
    int status = 0;

    d_logger->debug("in run_rx_cal");

    /* only run calibration if calibration is enabled, in manual mode, 
     * and this call has the right parameter */
    if ((value == RUN_CAL) && (cal_enabled == true) && (cal_mode == skiq_rx_cal_mode_manual) )
    {    
        d_logger->debug("in run_rx_cal() ");
        status = skiq_run_rx_cal( card, hdl1);
        if( status != 0 )
        {
            d_logger->error( "Error: run_rx_cal failed with status %" PRIi32 "", status);
            throw std::runtime_error("Failure: set rx_cal_type");
        }

        if (dual_port == true)
        {
            status = skiq_run_rx_cal( card, hdl1);
            if( status != 0 )
            {
                d_logger->error( "Error: run_rx_cal failed with status %" PRIi32 "", status);
                throw std::runtime_error("Failure: set rx_cal_type");
            }
        }

        d_logger->info("Info: run_rx_cal executed");
    }
}


/*
 * get_new_block
 *
 * This call will wait until we get a new block of data.
 *
 */
uint32_t sidekiq_rx_impl::get_new_block(uint32_t portno)
{
    int status = 0;
    skiq_rx_hdl_t tmp_hdl{};
    uint32_t data_length_bytes{};
    skiq_rx_block_t *p_rx_block{};
    uint32_t new_portno = portno;
    bool done = false;


    while (done == false)
    {
        status = skiq_receive(card, &tmp_hdl, &p_rx_block, &data_length_bytes);
        if (status  == skiq_rx_status_success) 
        {
            /* determine which port the received block is from */
            if (tmp_hdl == hdl1)
            {
                new_portno = 0;
            }
            else if (tmp_hdl == hdl2)
            {
                new_portno = 1;
            }
            else
            {
              d_logger->error( "Error : invalid hdl received {}", tmp_hdl);
              throw std::runtime_error("Failure:  invalid handle");
            }

            /* check timestamp for overrun */
            if (first_block[new_portno] == false)
            {
                uint64_t actual_tx = p_rx_block->rf_timestamp;
                uint64_t expected_ts = last_timestamp[new_portno] + DATA_MAX_BUFFER_SIZE;

                if (expected_ts != actual_tx)
                {
                    overrun_counter++;
                }
            }


            /* if enabled for stream tags, set the tag value */
            if (timestamp_tags == true)
            {
                curr_rf_block_tag.key = pmt::intern("rf_timestamp");
                curr_rf_block_tag.value = pmt::from_uint64(p_rx_block->rf_timestamp);
            }

            last_timestamp[new_portno] = p_rx_block->rf_timestamp;
            first_block[new_portno] = false;


            /* update the data with the new block */
            curr_block_ptr[new_portno] = (int16_t *)p_rx_block->data;
            curr_block_samples_left[new_portno] = DATA_MAX_BUFFER_SIZE;
            done = true;
        }
        else if (status == skiq_rx_status_no_data)
        {
            /* we are non-blocking so we will get this status */
            done = false;
            usleep(NON_BLOCKING_TIMEOUT);
        }
        else if (status == skiq_rx_status_error_overrun)
        {
            /* if we get an overrun, it will be detected in the next timestamp overrun test */
        }
        else 
        {
          done = true;
          d_logger->error( "Error : skiq_rcv failure, status {}", status);
          throw std::runtime_error("Failure: skiq_receive failure");
        }

    }

    /* we need to work on this new port so pass it back */
    return new_portno;

}

/*
 * determine_if_done
 *
 * With multiple ports, we need to get all the data from both ports then we are done.
 *
 * With single port, this will just determine if we have enough data for the single port
 *
 */
bool sidekiq_rx_impl::determine_if_done(int32_t *samples_written, int32_t noutput_items, uint32_t *portno)
{
    bool looping = true;


    /* handle single port different than dual port */
    if (dual_port)
    {
        /* neither port is done so just leave the port as it is */
        if (((samples_written[0] + DATA_MAX_BUFFER_SIZE) <= noutput_items) && 
                ((samples_written[1] + DATA_MAX_BUFFER_SIZE)  <= noutput_items))
        {
            looping = true;
        }
        /* port 0 is done, but port 1 is not, force port to 1 */
        else if (((samples_written[1] + DATA_MAX_BUFFER_SIZE) <= noutput_items) && 
                (samples_written[0] + DATA_MAX_BUFFER_SIZE) > noutput_items)
        {
            *portno = 1;
            looping = true;
        }
        /* port 1 is done, but port 0 is not, force port to 0 */
        else if (((samples_written[0] + DATA_MAX_BUFFER_SIZE) <= noutput_items) && 
                (samples_written[1] + DATA_MAX_BUFFER_SIZE) > noutput_items)
        {
            *portno = 0;
            looping = true;
        } 
        /* both ports are done, reset portno to 0 and leave loop */
        else
        {
            *portno = 0;
            looping = false;
        }
    }
    else
    {
        /* single port, always port number is 0 */
        if ((samples_written[0] + DATA_MAX_BUFFER_SIZE) <= noutput_items )
        {
            *portno = 0;
            looping = true;
        }
        else
        {
            *portno = 0;
            looping = false;
        }
    }

    return looping;
}

/*
 * work
 *
 * This is called by the gnuradio scheduler when it wants to receive a buffer full of samples
 */
int sidekiq_rx_impl::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items) 
{
    int32_t samples_written[MAX_PORT]{};
    int32_t delta_samples[MAX_PORT] = {noutput_items, noutput_items};
    uint32_t samples_to_write[MAX_PORT]{};
    uint32_t portno{};
    bool looping = true; 
    Clock::time_point this_time;

    this_time = Clock::now();
    gr_complex *out[MAX_PORT] = {NULL, NULL};
    gr_complex *curr_out_ptr[MAX_PORT] = {NULL, NULL} ;

    /* initialize the one-port output variables */    
    out[0] = static_cast<gr_complex *>(output_items[0]);
    curr_out_ptr[0] = out[0];

    /* if dual port, initialize the other */
    if (dual_port)
    { 
        out[1] = static_cast<gr_complex *>(output_items[1]);
        curr_out_ptr[1] = out[1];
    }

    first_block[0]  = true;
    first_block[1]  = true;

    /* We told gnuradio to not call us with a buffer size smaller than our block, so error out. */
    if (noutput_items < DATA_MAX_BUFFER_SIZE)
    {
        d_logger->error( "Error : invalid noutput_items {}", noutput_items);
        throw std::runtime_error("Failure: invalid noutput items");
    }


    /* Determine if the time has elapsed and display any underruns we have received */
    if ((nitems_written(0) - last_status_update_sample) > status_update_rate_in_samples)
    {

        if (overrun_counter > 0)
        {
            d_logger->info("Overruns detected: {}", overrun_counter);
        }

#ifdef DEBUG
        milliseconds ms = std::chrono::duration_cast<milliseconds>(this_time - last_time);
        last_time = this_time;

        d_logger->debug("delta time {}, noutput_items {}, nitems_written {}, last_update {} update_rate {}, work calls {}",
               ms.count(), noutput_items, nitems_written(0), last_status_update_sample, status_update_rate_in_samples, debug_ctr );
#else
        d_logger->debug("noutput_items {}, nitems_written {}, last_update {}",
               noutput_items, nitems_written(0), last_status_update_sample);
#endif

        last_status_update_sample = nitems_written(0);
    }

    /* loop until we have filled up these "out" packet(s) */
    while (looping == true)
    {
        /* if we don't have a block get one, if the block is from another port, it will change the portno */
        portno = get_new_block(portno);

        /* fill the output packet for this portno up with the contents of the block */
        if ((curr_block_samples_left[portno] > 0) && (samples_written[portno] < noutput_items))
        {
            /* figure out how many samples we have left to write */
            delta_samples[portno] = noutput_items - samples_written[portno];

            /* determine how many samples we can write */
            if (delta_samples[portno] <= curr_block_samples_left[portno])
            {
                /* the amount we have if the block is more than we need */
                samples_to_write[portno] = delta_samples[portno];

            } 
            else {
               /* there are fewer items left in the block than we need to write */
                samples_to_write[portno] = curr_block_samples_left[portno];
            }
//#define DEBUG
#ifdef DEBUG
            if (debug_ctr < 2)
            {
                printf("portno %d, overrun ctr %lu, samples_left %d, samples_written %d, samples_to_write %u, noutput_items %d\n",
                        portno, overrun_counter, curr_block_samples_left[portno], samples_written[portno], 
                        samples_to_write[portno], noutput_items);

#ifdef POO
                if (samples_written[portno] == 1018)
                {
                    printf("0x%08X ", (1143 * 4));
                    for (int i=125; i < 141; i++)
                    {
                        printf("0x%04X ", curr_block_ptr[portno][i * IQ_SHORT_COUNT + 1]);
                        printf("0x%04X ", curr_block_ptr[portno][i * IQ_SHORT_COUNT]);
                        if (i%4 == 0)
                        {
                            printf("\n");
                        }
                    }
                    printf("\n");
                }
#endif
                fflush(stdout);
            }
#endif

            /* convert and write the samples */
            volk_16i_s32f_convert_32f_u(
                  (float *) curr_out_ptr[portno],
                  (const int16_t *) curr_block_ptr[portno],
                  adc_scaling,
                  (samples_to_write[portno] * IQ_SHORT_COUNT ));



            /* increment all the pointers and counters */
            samples_written[portno] += samples_to_write[portno];
            curr_out_ptr[portno] += samples_to_write[portno];
            curr_block_ptr[portno] += (samples_to_write[portno] * IQ_SHORT_COUNT);
            curr_block_samples_left[portno] -= samples_to_write[portno];

            if (timestamp_tags == true)
            {
                add_item_tag(portno, tag_index[portno], 
                                curr_rf_block_tag.key, curr_rf_block_tag.value);

                /* update the absolute index into the stream */
                tag_index[portno] = tag_index[portno] + samples_written[portno];

                if (debug_ctr < 10)
                {
                    d_logger->debug("add item: ctr {}, portno {}, samples_written {}, noutput_items {}, buffer_size {}", 
                            debug_ctr, portno, samples_written[portno], noutput_items, DATA_MAX_BUFFER_SIZE);
                    d_logger->debug("key {}, value {}, abs_tag_index {}",
                            tag_index[portno], curr_rf_block_tag.key, curr_rf_block_tag.value);
                }
            }
        }


        /* determine if we are done with this work() call */
        looping = determine_if_done(samples_written, noutput_items, &portno);

    }


    if (curr_block_samples_left[portno] == 0)
    {
        curr_block_ptr[portno] = NULL;
    }
    
#ifdef DEBUG
    if (debug_ctr < 30)
    {
        milliseconds ms = std::chrono::duration_cast<milliseconds>(this_time - last_time);
        d_logger->debug("dual_port {}, items written {}, noutput_items {}, samples_written {}", 
                dual_port, nitems_written(0), noutput_items, samples_written[portno]);
        std::cout << ms.count() << "ms\n";
        last_time = this_time;
    }
#endif


    debug_ctr++;

    // Tell runtime system how many output items we produced.
    return samples_written[0];
}

} /* namespace sidekiq */
} /* namespace gr */
