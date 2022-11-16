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
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    libsidekiq_init = true;
    printf("Info: libsidkiq initialized successfully\n");

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

    set_rx_frequency(frequency);
    set_rx_gain_mode(gain_mode);

    if (gain_mode == 0)
    {
        set_rx_gain_index(gain_index);
    }
}


sidekiq_rx_impl::~sidekiq_rx_impl() 
{
}

bool sidekiq_rx_impl::start() 
{
    int status = 0;

    printf("in start() \n");

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
    
    printf("in stop() \n");

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

/* set the sample rate 
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_sample_rate(double value) 
{

    int status = 0;
    printf("in set_rx_sample_rate() \n");

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
  
/* set the bandwidth
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_bandwidth(double value) 
{
    int status = 0;
    printf("in set_rx_bandwidth() \n");

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

/* set the LO frequency
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_rx_impl::set_rx_frequency(double value) 
{
    int status = 0;
    printf("in set_rx_frequency() \n");

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

void sidekiq_rx_impl::set_rx_gain_mode(double value) 
{
    int status = 0;

    printf("in set_gain_mode() \n");

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
bool sidekiq_rx_impl::determine_if_done(int32_t *samples_written, int32_t noutput_items, uint32_t *portno)
{
    bool looping = true;

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
        /* single port */
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

#define DEBUG
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

    if (dual_port)
    { 
        out[1] = static_cast<gr_complex *>(output_items[1]);
        curr_out_ptr[1] = out[1];
    }


#ifdef DEBUG
    if (debug_ctr < 2)
    {
        printf("noutput_items %d, dual_port %d, buffer_size %d, out0 %p, out1 %p\n", 
                noutput_items, dual_port, DATA_MAX_BUFFER_SIZE, out[0], out[1]);
    }
#endif

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

            if (samples_to_write[portno] == 0)
            {
                printf("samples to write is 0\n");
                exit(1);
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

#ifdef DEBUG
            if (debug_ctr < 2)
            {
                printf("portno %d, samples_to_write %d, curr_block_samples_left %d, samples_written %d\n", 
                        portno, samples_to_write[portno], curr_block_samples_left[portno], 
                        samples_written[portno]);
            }
#endif
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
