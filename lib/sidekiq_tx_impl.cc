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
#include <pthread.h>

#include "sidekiq_tx_impl.h"


#define DEBUG_LEVEL "debug"  //Can be debug, info, warning, error, critical

/* The tx_complete function needs to be outside the object so it can be registered with libsidekiq 
 * mutex to protect updates to the tx buffer
 * Any parameters it uses also needs to be global and accessible inside and outside the function.
 */
static pthread_mutex_t tx_buf_mutex;
  
static uint32_t complete_count{};

/* mutex and condition variable to signal when the tx queue may have room available */
static pthread_mutex_t space_avail_mutex;
static pthread_cond_t space_avail_cond;

/* 
 * When in async mode this is called after each block is completed by libsidekiq 
 *
 * This is outside the class since it is called by libsidekiq
 */
static void tx_complete( int32_t status, skiq_tx_block_t *p_data, void *p_user )
{
    /* -2 happens when there are outstanding buffers and we stop streaming */
    if( status != 0 && status != -2)
    {
        fprintf(stderr, "Error: packet %" PRIu32 " failed with status %d\n",
                complete_count, status);
    }

    // increment the packet completed count
    complete_count++;

    pthread_mutex_lock( &tx_buf_mutex );
    // update the in use status of the packet just completed
    if (p_user)
    {
        *(int32_t*)p_user = 0;
    }
     pthread_mutex_unlock( &tx_buf_mutex );

    // signal to the other thread that there may be space available now that a
    // packet send has completed
    pthread_mutex_lock( &space_avail_mutex );
    pthread_cond_signal(&space_avail_cond);
    pthread_mutex_unlock( &space_avail_mutex );

}

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
                                  int threads,
                                  int buffer_size,
                                  int cal_mode)
{
    /* then make instantiates the tx_block */
    return gnuradio::make_block_sptr<sidekiq_tx_impl>(
                                  card, 
                                  handle,
                                  sample_rate,
                                  bandwidth,
                                  frequency,
                                  attenuation,
                                  threads,
                                  buffer_size,
                                  cal_mode);
}


/* constructor 
 * Initialize the card
 */
sidekiq_tx_impl::sidekiq_tx_impl( int input_card,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
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

    printf("in constructor, debug level:%s\n", str.c_str());
    
    int status = 0;
    uint8_t iq_resolution = 0;

    card = input_card;
    hdl = (skiq_tx_hdl_t)handle;
    curr_block = 0;
    tx_buffer_size = buffer_size;
    temp_buffer.resize(tx_buffer_size);
    bursting_cmd = NO_BURSTING_ALLOWED;

    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0) 
    {
        d_logger->error( "Error: unable to initialize libsidekiq with status {}", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    libsidekiq_init = true;
    d_logger->info("Info: libsidkiq initialized successfully");

    set_tx_sample_rate(sample_rate);
    set_tx_bandwidth(bandwidth);

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

    /* if A2 or B2 is used, we need to set the channel mode to dual */
    if (hdl == skiq_tx_hdl_A2 || hdl == skiq_tx_hdl_B2) 
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

    /* write the block size to the passed in amount */
    status = skiq_write_tx_block_size(card, hdl, tx_buffer_size);
    if (status != 0) 
    {
        d_logger->error( "Error: unable to configure TX block size: {} with status {}", 
                tx_buffer_size, status);
        throw std::runtime_error("Failure: skiq_write_tx_block_size");
    }
    d_logger->info("Info: TX block size {}", tx_buffer_size);

    /* handle sync vs async mode */
    if (threads > 1)
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
        
        status = skiq_register_tx_complete_callback( card, &tx_complete );
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

    /* allocate memory to hold the pointers for the tx blocks */
    p_tx_blocks = (skiq_tx_block_t **)calloc( num_blocks, sizeof( skiq_tx_block_t * ));
    if( p_tx_blocks == NULL )
    {
        d_logger->error( "Error: failed to allocate memory for TX blocks.");
        throw std::runtime_error("Failure: calloc p_tx_blocks");
    }

    /* allocate memory to hold the status for each block */
    p_tx_status = (int32_t *)calloc( num_blocks, sizeof(*p_tx_status) );
    if( p_tx_status == NULL )
    {
        d_logger->error( "Error: failed to allocate memory for TX status.");
        throw std::runtime_error("Failure: calloc p_tx_status");
    }

    /* ask libsidekiq to allocate each block */ 
    for (uint32_t i = 0; i < num_blocks; i++)
    {
        /* allocate a transmit block by number of words */
        p_tx_blocks[i] = skiq_tx_block_allocate( tx_buffer_size );
        p_tx_status[i] = 0;
    }

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
    d_logger->debug("in destructor");

    for (uint32_t i = 0; i < num_blocks; i++)
    {
        /* allocate a transmit block by number of words */
        skiq_tx_block_free(p_tx_blocks[i]);
    }

    if (p_tx_blocks != NULL)
    {
        free(p_tx_blocks);
    }

    if (p_tx_status != NULL)
    {
        free(p_tx_status);
    }

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

    if (pmt::dict_has_key(msg, TX_FREQ_KEY)) 
    {
        set_tx_frequency(get_double_from_pmt_dict(msg, TX_FREQ_KEY));
    }

    if (pmt::dict_has_key(msg, TX_RATE_KEY)) 
    {
        set_tx_sample_rate(get_double_from_pmt_dict(msg, TX_RATE_KEY));
    }


    if (pmt::dict_has_key(msg, TX_START_BURST)) 
    {
        double cmd = get_double_from_pmt_dict(msg, TX_START_BURST);

        if (cmd == BURSTING_ON)
        {
            d_logger->debug("starting bursting..., cmd {}", cmd);
            bursting_cmd = cmd;
            this->start();
        }
        else if (cmd == BURSTING_OFF) 
        {
            d_logger->debug("stopping bursting, cmd {}", cmd);
            bursting_cmd = cmd;
            this->stop();
        }
        else
        {
            bursting_cmd = NO_BURSTING_ALLOWED;
        }
    }
}


/* start streaming */
bool sidekiq_tx_impl::start() 
{
    int status = 0;

    d_logger->debug("in start() ");

    if (bursting_cmd == NO_BURSTING_ALLOWED || bursting_cmd == BURSTING_ON)
    {
        
        status = skiq_start_tx_streaming(card, hdl);
        if (status != 0)
        {
            d_logger->error( "Error: could not start TX streaming, status {}", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }

        tx_streaming = true;

        return block::start();
    }
    else
    {
        return false;
    }
}

/* stop streaming */
bool sidekiq_tx_impl::stop() 
{
    int status = 0;
    
    d_logger->debug("in stop() ");

    if (tx_streaming == true)
    {
        status = skiq_stop_tx_streaming(card, hdl);
        if (status != 0)
        {
            d_logger->error( "Error: could not stop TX streaming, status {}", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
        tx_streaming = false;
    }

    

    return block::stop();
}

/* set the sample rate 
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_sample_rate(double value) 
{

    int status = 0;
    d_logger->debug("in set_tx_sample_rate() ");

    auto rate = static_cast<uint32_t>(value);
    auto bw = static_cast<uint32_t>(this->bandwidth);

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        d_logger->error( "Error: could not set sample_rate, status {}, {}", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

    status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

}
  
/* set the bandwidth
 * this may be called from the flowgraph if the user changes the variable
 */
void sidekiq_tx_impl::set_tx_bandwidth(double value) 
{
    int status = 0;
    d_logger->debug("in set_tx_bandwidth() ");

    auto rate = static_cast<uint32_t>(this->sample_rate);
    auto bw = static_cast<uint32_t>(value);

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        d_logger->error("Error: could not set bandwidth {}, status {}, {}", 
                bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
        return;
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

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
    if (bursting_cmd == BURSTING_ON)
    {
        d_logger->debug("in handle_tx_burst_tag, tag offset {}", tag.offset);


        burst_length = pmt::to_uint64(tag.value);
        burst_samples_sent = 0;

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

    /* see if we received the TX_BURST tag, if so process it */
    get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items);
    if (not tags.empty())
    {
        BOOST_FOREACH( const tag_t &tag, tags) 
        {
            if (pmt::equal(tag.key, TX_BURST_KEY)) 
            {
                handle_tx_burst_tag(tag);
            }
        }
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

            /* convert the samples we have received to be within the dac_scaling values */
            volk_32f_s32f_multiply_32f(
                    reinterpret_cast<float *>(&temp_buffer[0]),
                    reinterpret_cast<const float *>(in),
                    dac_scaling,
                    static_cast<unsigned int>(samples_to_write * 2));

            /* convert those samples from float complex to int16 */
            volk_32fc_convert_16ic(
                    reinterpret_cast<lv_16sc_t *>(p_tx_blocks[curr_block]->data),
                    reinterpret_cast<const lv_32fc_t*>(&temp_buffer[0]),
                    samples_to_write);
            

            /* transmit the samples */
            status = skiq_transmit(card, hdl, p_tx_blocks[curr_block], &(p_tx_status[curr_block]));

            /* check to see if the TX queue is full, if so wait for a released buffer */ 
            if( status == SKIQ_TX_ASYNC_SEND_QUEUE_FULL )
            {
                // update the in use status since we didn't actually send it yet
                pthread_mutex_lock( &tx_buf_mutex );
                p_tx_status[curr_block] = 0;
                pthread_mutex_unlock( &tx_buf_mutex );

                pthread_mutex_lock( &space_avail_mutex );
                pthread_cond_wait( &space_avail_cond, &space_avail_mutex );
                pthread_mutex_unlock( &space_avail_mutex );
            }
            else if ( status != 0 ) 
            {
                d_logger->info("Info: sidekiq transmit failed with error: {}", status);
            } 
            else {
                samples_written += samples_to_write;

                /* move the pointer */
                in += samples_to_write;

                /* move to the next block if we are in async mode, otherwise this is always 1 */
                curr_block = (curr_block + 1) % num_blocks;
            }

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
