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

const bool SIDEKIQ_IQ_PACK_MODE_UNPACKED{false}; 
const int DATA_MAX_BUFFER_SIZE{SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS};

#define IQ_SHORT_COUNT 2        // number of shorts in a sample

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
        int gain_index) {
  return gnuradio::make_block_sptr<sidekiq_rx_impl>(
          input_card,
          port1_handle,
          port2_handle,
          sample_rate,
          bandwidth,
          frequency,
          gain_mode,
          gain_index);
}

sidekiq_rx_impl::sidekiq_rx_impl(
        int input_card,
        int port1_handle,
        int port2_handle,
        double sample_rate,
        double bandwidth,
        double frequency,
        uint8_t gain_mode,
        int gain_index) 
    : gr::sync_block("sidekiq_rx", gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1 /* min outputs */,
                                            2 /*max outputs */,
                                            sizeof(gr_complex))) 
{
    int status = 0;
    uint8_t iq_resolution = 0;

    card = input_card;
    hdl = (skiq_rx_hdl_t) port1_handle;


    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0)
    {
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    libsidekiq_init = true;
    printf("Info: libsidkiq initialized successfully\n");

    set_rx_sample_rate(sample_rate);
    set_rx_bandwidth(bandwidth);

    status = skiq_read_rx_iq_resolution(card, &iq_resolution);
    if (status != 0)
    {
        fprintf(stderr, "Error: unable to get iq resolution with status %d\n", status);
        throw std::runtime_error("Failure: skiq_read_tx_iq_resolution");
    }
    adc_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;
    printf("Info: adc scaling %f\n", adc_scaling);

    /* if A2 or B2 is used, we need to set the channel mode to dual */
    if (hdl == skiq_rx_hdl_A2 || hdl == skiq_rx_hdl_B2)
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


    set_rx_frequency(frequency);
    set_rx_gain_mode(gain_mode);
    set_rx_gain_index(gain_index);

}


/*
 * Our virtual destructor.
 */
sidekiq_rx_impl::~sidekiq_rx_impl() 
{
}

bool sidekiq_rx_impl::start() 
{
    int status = 0;

    printf("in start() \n");

    status = skiq_start_rx_streaming(card, hdl);
    if (status != 0)
    {
        fprintf(stderr, "Error: could not start RX streaming, status %d\n", status);
        throw std::runtime_error("Failure: skiq_start_rx_streaming");
    }

    rx_streaming = true;

    return block::start();


}

/* stop streaming */
bool sidekiq_rx_impl::stop() 
{
    int status = 0;
    
    printf("in stop() \n");

    if (rx_streaming == true)
    {
        status = skiq_stop_rx_streaming(card, hdl);
        if (status != 0)
        {
            fprintf(stderr, "Error: could not stop TX streaming, status %d\n", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
    }
    

    return block::stop();
}

/* set the sample rate 
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_sample_rate(double value) 
{

    int status = 0;
    printf("in set_rx_sample_rate() \n");

    auto rate = static_cast<uint32_t>(value);
    auto bw = static_cast<uint32_t>(this->bandwidth);

    status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        fprintf(stderr, "Error: could not set sample_rate, status %d, %s\n", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

}
  
/* set the bandwidth
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_bandwidth(double value) 
{
    int status = 0;
    printf("in set_rx_bandwidth() \n");

    auto rate = static_cast<uint32_t>(this->sample_rate);
    auto bw = static_cast<uint32_t>(value);

    status = skiq_write_rx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        fprintf(stderr,"Error: could not set bandwidth %d, status %d, %s\n", 
                bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set bandwidth");
        return;
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

}

/* set the LO frequency
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_frequency(double value) 
{
    int status = 0;
    printf("in set_rx_frequency() \n");

    auto freq = static_cast<uint64_t>(value);

    status = skiq_write_rx_LO_freq(card, hdl, freq);
    if (status != 0) 
    {
        fprintf(stderr,"Error: could not set frequency %ld, status %d, %s\n", 
                freq, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set frequency");
        return;
    }

    this->frequency = freq;
}

void sidekiq_rx_impl::set_rx_gain_mode(double value) 
{
    int status = 0;

    printf("in set_gain_mode() \n");

    auto gain_mode = static_cast<skiq_rx_gain_t>(value);

    status = skiq_write_rx_gain_mode(card, hdl, gain_mode);
    if (status != 0) 
    {
        fprintf(stderr,"Error: write_rx_gain_mode failed, status %d, %s\n", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set write_rx_gain_mode");
        return;
    }
    printf("Info: gain_mode set to %d\n", gain_mode);
    
}

/* set the gain_index
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_gain_index(int value) 
{
    int status = 0;
    uint8_t min_range = 0;
    uint8_t max_range = 0;
    
    printf("in set_gain_index() \n");

    auto gain = static_cast<uint8_t>(value);

    status = skiq_read_rx_gain_index_range(card, hdl, &min_range, &max_range);
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
    printf("Info: gain index %d\n", gain); 

    this->gain_index = gain;
}


int sidekiq_rx_impl::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items) 
{
    int status = 0;
    int samples_written{};
    int32_t delta_samples = noutput_items;
    skiq_rx_hdl_t tmp_hdl{};
    uint32_t samples_to_write{};
    uint32_t data_length_bytes{};

    auto out = static_cast<gr_complex *>(output_items[0]);
    gr_complex *curr_out_ptr = out;

    if (debug_ctr < 4)
    {
        printf("buffer_size %d, noutput_items %d, curr_block_samples_left %d  \n", 
                DATA_MAX_BUFFER_SIZE, noutput_items, curr_block_samples_left);
        debug_ctr++;
    }


    /* loop until we have filled up this "out" packet */
    while (samples_written < noutput_items)
    {
        /* if we don't have a block get one */
        if (curr_block_samples_left <= 0)
        {
            status = skiq_receive(card, &tmp_hdl, &p_rx_block, &data_length_bytes);
            if (status  == skiq_rx_status_success) 
            {
                if (tmp_hdl == hdl)
                {
                    curr_block_ptr = (int16_t *)p_rx_block->data;
                    curr_block_samples_left = DATA_MAX_BUFFER_SIZE;
                }
            } 
            else if (status != skiq_rx_status_no_data)
            {
              fprintf(stderr, "Error : skiq_rcv failure, status %d\n", status);
              throw std::runtime_error("Failure: skiq_receive failure");
            }

        }

        /* fill the current packet up as much as we can */
        if ((curr_block_ptr != NULL) && (curr_block_samples_left > 0))
        {
            /* figure out how many samples we have left to write */
            delta_samples = noutput_items - samples_written;

            /* determine how many samples we can write */
            if (delta_samples <= curr_block_samples_left)
            {
                samples_to_write = delta_samples;

            } 
            else {
               /* we fewer items left in the block than we need to write */
                samples_to_write = curr_block_samples_left;
            } 

            /* convert and write the samples */
            volk_16i_s32f_convert_32f_u(
                  (float *) curr_out_ptr,
                  (const int16_t *) curr_block_ptr,
                  adc_scaling,
                  (samples_to_write * IQ_SHORT_COUNT ));

            /* increment all the pointers and counters */
            samples_written += samples_to_write;
            curr_out_ptr += samples_to_write;
            curr_block_ptr += (samples_to_write * IQ_SHORT_COUNT);

            curr_block_samples_left -= samples_to_write;
            if (curr_block_samples_left == 0)
            {
                curr_block_ptr = NULL;
                p_rx_block = NULL;
            }
            else if (curr_block_samples_left < 0)
            {
                fprintf(stderr, "Error: somehow curr_block_samples went < 0 %d\n", curr_block_samples_left);
                throw std::runtime_error("Failure: skiq_receive failure");
            }

        }

    }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace sidekiq */
} /* namespace gr */
