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

#define DEBUG_LEVEL "info" //Can be debug, info, warning, error, critical

using pmt::pmt_t;
const pmt_t CONTROL_MESSAGE_PORT{pmt::string_to_symbol("command")};

namespace gr {
namespace sidekiq {

using output_type = float;
sidekiq_rx::sptr sidekiq_rx::make(
        int input_card,
        int transceive,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
        int cal_mode,
        int cal_type) 
{
  return gnuradio::make_block_sptr<sidekiq_rx_impl>(
          input_card,
          transceive,
          port1_handle,
          port2_handle,
          sample_rate,
          bandwidth,
          frequency,
          gain_mode,
          gain_index,
          cal_mode,
          cal_type);
}

sidekiq_rx_impl::sidekiq_rx_impl(
        int input_card,
        int transceive,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index,
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

    card = input_card;
    transceive_mode = transceive;
    hdl1 = (skiq_rx_hdl_t) port1_handle;

    /* determine if we are in dual port */
    if (port2_handle < skiq_rx_hdl_end)
    {
        dual_port = true;
        hdl2 = (skiq_rx_hdl_t) port2_handle;
    }
    else
    {
        hdl2 = skiq_rx_hdl_end;
        dual_port = false;
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
            if (transceive_mode != TRANSCEIVE_ENABLED)
            {
                d_logger->error( "Error: unable to initialize libsidekiq with status {}", status);
                throw std::runtime_error("Failure: skiq_init");
            }
            else
            {
                d_logger->info("Info: The TX block initialized libsidekiq");
            }
        }
    }
    else
    {
        libsidekiq_init = true;
        printf("Info: libsidkiq initialized successfully\n");
    }

    set_rx_sample_rate(sample_rate);
    set_rx_bandwidth(bandwidth);

    /* calculate the adc scaling */
    status = skiq_read_rx_iq_resolution(card, &iq_resolution);
    if (status != 0)
    {
        fprintf(stderr, "Error: unable to get iq resolution with status %d\n", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    adc_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    printf("Info: adc scaling %f\n", adc_scaling);


    /* if A2 or B2 is used, we need to set the channel mode to dual */
    if (hdl1 == skiq_rx_hdl_A2 || hdl1 == skiq_rx_hdl_B2 || 
            hdl2 == skiq_rx_hdl_A2 || hdl2 == skiq_rx_hdl_B2)
    {
        status = skiq_write_chan_mode(card, skiq_chan_mode_dual);
        if (status != 0)
        {
            fprintf(stderr, "Error: unable to configure TX channel mode with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    }
    else {
        status = skiq_write_chan_mode(card, skiq_chan_mode_single);
        if (status != 0)
        {
            fprintf(stderr, "Error: unable to configure TX channel mode with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    }

    /* always assume unpacked */
    status = skiq_write_iq_pack_mode(card, SIDEKIQ_IQ_PACK_MODE_UNPACKED);
    if (status != 0)
    {
        fprintf(stderr, "Error: unable to set iq pack mode to unpacked with status %d\n", status);
        throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    /* by default all cards are in Q/I order we want it to be I/Q so switch it */
    status = skiq_write_iq_order_mode(card, skiq_iq_order_iq) ;
    if (status != 0)
    {
          fprintf(stderr, "Error: unable to set iq order mode to iq with status %d \n", status);
          throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    /* support two messages */
    message_port_register_in(CONTROL_MESSAGE_PORT);
    set_msg_handler(CONTROL_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });

    /* set the rest of the parameters */
    set_rx_frequency(frequency);
    set_rx_gain_mode(gain_mode);
    set_rx_gain_index(gain_index);
    set_rx_cal_mode(cal_mode);
    set_rx_cal_type(cal_type);
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
            msg,
            pmt::car(msg),
            pmt::cdr(msg));
        msg = pmt::dict_add(pmt::make_dict(), pmt::car(msg), pmt::cdr(msg));
     }

     // Make sure, we use dicts!
     if (!pmt::is_dict(msg)) {
         d_logger->error("Command message is neither dict nor pair: {}", msg);
         return;
     }

    if (pmt::dict_has_key(msg, RX_FREQ_KEY)) 
    {
        set_rx_frequency(get_double_from_pmt_dict(msg, RX_FREQ_KEY));
    }

    if (pmt::dict_has_key(msg, RX_RATE_KEY)) 
    {
        set_rx_sample_rate(get_double_from_pmt_dict(msg, RX_RATE_KEY));
    }


}

/* 
 * start streaming
 * 
 * Called by generated python code
 */
bool sidekiq_rx_impl::start() 
{
    int status = 0;

    d_logger->debug("in start");

    status = skiq_start_rx_streaming(card, hdl1);
    if (status != 0)
    {
        fprintf(stderr, "Error: could not start RX streaming on hdl1, status %d\n", status);
        throw std::runtime_error("Failure: skiq_start_rx_streaming");
    }

    if (dual_port)
    {
        status = skiq_start_rx_streaming(card, hdl2);
        if (status != 0)
        {
            fprintf(stderr, "Error: could not start RX streaming on hdl2, status %d\n", status);
            throw std::runtime_error("Failure: skiq_start_rx_streaming");
        }
    }


    rx_streaming = true;
    printf("Info: RX streaming started\n");

    return block::start();
}

/* stop streaming */
bool sidekiq_rx_impl::stop() 
{
    int status = 0;
    
    d_logger->debug("in stop");

    /* only call stop if we are actually streaming */
    if (rx_streaming == true)
    {
        status = skiq_stop_rx_streaming(card, hdl1);
        if (status != 0)
        {
            fprintf(stderr, "Error: could not stop TX streaming on hdl1, status %d\n", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
        if (dual_port)
        {
            status = skiq_stop_rx_streaming(card, hdl2);
            if (status != 0)
            {
                fprintf(stderr, "Error: could not stop TX streaming on hdl2, status %d\n", status);
                throw std::runtime_error("Failure: skiq_start_tx_streaming");
            }
        }
        printf("Info: RX streaming stopped\n");
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
        fprintf(stderr, "Error: could not set sample_rate on hdl1, status %d, %s\n", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    if (dual_port)
    {
        status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl2, rate, bw); 
        if (status != 0) 
        {
            fprintf(stderr, "Error: could not set sample_rate on hdl2, status %d, %s\n", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set samplerate");
        }
    }
    printf("Info: sample_rate set to %d\n", rate);

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
        fprintf(stderr,"Error: could not set bandwidth %d on hdl1, status %d, %s\n", 
                bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set bandwidth");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl2, rate, bw); 
        if (status != 0) 
        {
            fprintf(stderr,"Error: could not set bandwidth %d on hdl2, status %d, %s\n", 
                    bw, status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set bandwidth");
            return;
        }
    }
    printf("Info: bandwidth set to %d\n", bw);

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
        fprintf(stderr,"Error: could not set frequency %ld on hdl1, status %d, %s\n", 
                freq, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set frequency");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_LO_freq(card, hdl2, freq);
        if (status != 0) 
        {
            fprintf(stderr,"Error: could not set frequency %ld on hdl2, status %d, %s\n", 
                    freq, status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set frequency");
            return;
        }
    }

    printf("Info: frequency set to %ld\n", freq);

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
        fprintf(stderr,"Error: write_rx_gain_mode failed on hdl1, status %d, %s\n", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set write_rx_gain_mode");
        return;
    }

    if (dual_port)
    {
        status = skiq_write_rx_gain_mode(card, hdl2, gain_mode);
        if (status != 0) 
        {
            fprintf(stderr,"Error: write_rx_gain_mode failed on hdl2, status %d, %s\n", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set write_rx_gain_mode");
            return;
        }
    }

    printf("Info: gain_mode set to %d\n", gain_mode);

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
            fprintf(stderr,"Error: read_rx_gain_index failed, status %d, %s\n", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set read_rx_gain_index");
            return;
        }
        printf("Info: gain range for current frequency is %d - %d\n", min_range, max_range);

        if (gain > max_range || gain < min_range)
        {
            fprintf(stderr,"Error: gain_index %d is out of range\n", gain);
            throw std::runtime_error("Failure: gain index is out of range");
            return;
        }

        status = skiq_write_rx_gain(card, hdl1, gain);
        if (status != 0) 
        {
            fprintf(stderr,"Error: write_rx_gain failed on hdl1, status %d, %s\n", 
                    status, strerror(abs(status)) );
            throw std::runtime_error("Failure: set read_rx_gain_index");
            return;
        }

        if (dual_port)
        {
            status = skiq_write_rx_gain(card, hdl2, gain);
            if (status != 0) 
            {
                fprintf(stderr,"Error: write_rx_gain failed on hdl2, status %d, %s\n", 
                        status, strerror(abs(status)) );
                throw std::runtime_error("Failure: set read_rx_gain_index");
                return;
            }
        }

        printf("Info: gain index %d\n", gain); 

        this->gain_index = gain;
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
        printf("Info: cal_mode set to off\n");
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
                fprintf(stderr, "Error: failed to configure RX calibration mode with %" PRIi32 "\n", status);
                throw std::runtime_error("Failure: set rx_cal_mode");
            }
            else
            {
                printf("Warning: calibration mode %d unsupported with product\n", cmode);
            }
        }

        if (dual_port == true)
        {
            status = skiq_write_rx_cal_mode( card, hdl2, cmode );
            if( status != 0 )
            {
                if( status != -ENOTSUP )
                {
                    fprintf(stderr, "Error: failed to configure RX calibration mode with %" PRIi32 "\n", status);
                    throw std::runtime_error("Failure: set rx_cal_mode");
                }
                else
                {
                    printf("Warning: calibration mode %d unsupported with product\n", cmode);
                }
            }
        }

        this->cal_mode = cmode;
        printf("Info: cal_mode set to %d\n", cmode);
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

        uint32_t read_cal_mask = 0;
        if( (status = skiq_read_rx_cal_types_avail( card, hdl1, &read_cal_mask )) == 0 )
        {
            if( read_cal_mask != cal_mask )
            {
                printf("Warning: RX calibration mask available for card is (0x%x)" 
                       " does not match what is desired (0x%x)\n",
                       read_cal_mask, cal_mask);
                printf("Info: Setting cal_mask to 0x%x\n", read_cal_mask);
                cal_mask = read_cal_mask;
            }
        }
        else
        {
            printf("Error: unable to read calibration mask (status=%d)\n", status);
        }

        status = skiq_write_rx_cal_type_mask( card, hdl1, cal_mask );
        if( status != 0 )
        {
            fprintf(stderr, "Error: failed to configure RX calibration type with status %" PRIi32 "\n", status);
            throw std::runtime_error("Failure: set rx_cal_type");
        }

        if (dual_port == true)
        {
            status = skiq_write_rx_cal_type_mask( card, hdl2, cal_mask );
            if( status != 0 )
            {
                fprintf(stderr, "Error: failed to configure RX calibration type with status %" PRIi32 "\n", status);
                throw std::runtime_error("Failure: set rx_cal_type");
            }
        }

        printf("Info: rx cal_mask 0x%2X, written successfully\n", cal_mask);

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
        printf("in run_rx_cal() \n");
        status = skiq_run_rx_cal( card, hdl1);
        if( status != 0 )
        {
            fprintf(stderr, "Error: run_rx_cal failed with status %" PRIi32 "\n", status);
            throw std::runtime_error("Failure: set rx_cal_type");
        }

        if (dual_port == true)
        {
            status = skiq_run_rx_cal( card, hdl1);
            if( status != 0 )
            {
                fprintf(stderr, "Error: run_rx_cal failed with status %" PRIi32 "\n", status);
                throw std::runtime_error("Failure: set rx_cal_type");
            }
        }

        printf("Info: run_rx_cal executed\n");
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

    /* since skiq_receive is non-blocking, we need to loop till we get a new packet */
    while (curr_block_samples_left[new_portno] <= 0)
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
              fprintf(stderr, "Error : invalid hdl received %d\n", tmp_hdl);
              throw std::runtime_error("Failure:  invalid handle");
            }

            /* update the data with the new block */
            curr_block_ptr[new_portno] = (int16_t *)p_rx_block->data;
            curr_block_samples_left[new_portno] = DATA_MAX_BUFFER_SIZE;
        } 
        else if (status != skiq_rx_status_no_data)
        {
          fprintf(stderr, "Error : skiq_rcv failure, status %d\n", status);
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
        if (samples_written[0] < noutput_items && samples_written[1] < noutput_items)
        {
            looping = true;
        }
        /* port 0 is done, but port 1 is not, force port to 1 */
        else if (samples_written[1] < noutput_items && samples_written[0] == noutput_items)
        {
            *portno = 1;
            looping = true;
        }
        /* port 1 is done, but port 0 is not, force port to 0 */
        else if (samples_written[0] < noutput_items && samples_written[1] == noutput_items)
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
        if (samples_written[0] < noutput_items )
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

    if (debug_ctr < 2)
    {
        d_logger->debug("noutput_items {}, dual_port {}, buffer_size {}", 
               noutput_items, dual_port, DATA_MAX_BUFFER_SIZE);
    }

    /* loop until we have filled up these "out" packet(s) */
    while (looping == true)
    {
        /* if we don't have a block get one, if the block is from another port, it will change the portno */
        portno = get_new_block(portno);

        /* fill the output packet for this portno up with the contents of the block */
        if ((curr_block_samples_left[portno] > 0) && samples_written[portno] < noutput_items)
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
            if (curr_block_samples_left[portno] == 0)
            {
                curr_block_ptr[portno] = NULL;
            }

            if (debug_ctr < 2)
            {
                d_logger->debug("portno {}, samples_to_write {}, curr_block_samples_left {}, samples_written {}", 
                        portno, samples_to_write[portno], curr_block_samples_left[portno], 
                        samples_written[portno]);
            }
        }

        /* determine if we are done with this work() call */
        looping = determine_if_done(samples_written, noutput_items, &portno);
    }

    debug_ctr++;

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace sidekiq */
} /* namespace gr */
