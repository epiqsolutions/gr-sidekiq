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
#include <gnuradio/io_signature.h>

#include "sidekiq_tx_impl.h"

#define DEBUG
#define NUM_BLOCKS  20

static const double STATUS_UPDATE_RATE_SECONDS{2.0};

/* in order to do async mode we need to be able to call the tx_complete function outside
 * the object.  This requires all variables that the function will use to also be outside the object.
 */
static uint32_t complete_count{};

static pthread_mutex_t tx_buf_mutex;
  
// mutex and condition variable to signal when the tx queue may have room available
static pthread_mutex_t space_avail_mutex;
static pthread_cond_t space_avail_cond;

/* if in async mode this function is called by a libsidekiq tx thread to release the tx_block */
static void tx_complete( int32_t status, skiq_tx_block_t *p_data, void *p_user )
{
    /* if status == -2 happens when there are outstanding buffers and we stop streaming 
     * so this can happen and not an error */
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
sidekiq_tx::sptr sidekiq_tx::make(int card,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  int threads,
                                  int buffer_size)
{
    return gnuradio::make_block_sptr<sidekiq_tx_impl>(
                                  card, 
                                  handle,
                                  sample_rate,
                                  bandwidth,
                                  frequency,
                                  attenuation,
                                  threads,
                                  buffer_size);
}


/* call all the skiq_calls required to setup the transmission */
sidekiq_tx_impl::sidekiq_tx_impl( int input_card,
                                  int handle,
                                  double rate,
                                  double bw,
                                  double freq,
                                  double att,
                                  int threads,
                                  int buffer_size)
    : gr::sync_block("sidekiq_tx",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0))   //sync block
{
#ifdef DEBUG
    printf("in constructor\n");
    printf("card %d, handle %d rate %f, bandwidth %f, \nfrequency %f, atten %f, threads %d, buffer_size %d\n", 
            input_card, handle, rate, bw, freq, att, threads, buffer_size);
#endif

    int status = 0;
    uint8_t iq_resolution = 0;

    /* initialize the object variables */
    card = input_card;
    hdl = (skiq_tx_hdl_t)handle;
    sample_rate = rate;
    frequency = freq;
    attenuation = att;

    libsidekiq_init = false;
    in_async_mode = false;
    tx_streaming = false;
    p_tx_blocks = NULL;
    p_tx_status = NULL;
    sync_tx_block = NULL;

    curr_block = 0;
    tx_buffer_size = buffer_size;
    temp_buffer.resize(tx_buffer_size);

    /* initialize libsidekiq for this card */
    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    libsidekiq_init = true;

    printf("Info: libsidkiq initialized successfully\n");

    /* sample_rate and bandwidth can be modified by the flowgraph so they have their own functions */
    set_tx_sample_rate(sample_rate);
    set_tx_bandwidth(bandwidth);

    /* we need the card resolution to convert from float to int16_t */
    status = skiq_read_tx_iq_resolution(card, &iq_resolution);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    dac_scaling = (pow(2.0f, iq_resolution) / 2.0) - 1;

    printf("Info: dac scaling %f\n", dac_scaling);

    /* We may allow other modes in the future, but set to immediate for now */
    status = skiq_write_tx_data_flow_mode(card, hdl, skiq_tx_immediate_data_flow_mode);
    if (status != 0) 
    {
        fprintf(stderr, "Error: could not set TX dataflow mode with status %d\n", status);
        throw std::runtime_error("Failure: skiq_write_tx_flow_mode");
    }

    /* The card is in dual mode if either handle A2 or B2 is used */
    if (hdl == skiq_tx_hdl_A2 || hdl == skiq_tx_hdl_B2) 
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

    /* Set the transmit block size to what the user gave us */
    status = skiq_write_tx_block_size(card, hdl, tx_buffer_size);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to configure TX block size: %d with status %d\n", 
                tx_buffer_size, status);
        throw std::runtime_error("Failure: skiq_write_tx_block_size");
    }
    printf("Info: TX block size %d\n", tx_buffer_size);

    /* Some of the cards do not support async mode so set it to sync if the user set 1 thread */ 
    if (threads > 1)
    {
        in_async_mode = true;
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_async);
        if (status != 0) 
        {
            fprintf(stderr, "Error: unable to configure TX channel mode to async with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }

        status = skiq_write_num_tx_threads(card, threads);
        if (status != 0) 
        {
            printf("Error: unable to configure TX number of threads with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }

        status = skiq_register_tx_complete_callback( card, &tx_complete );
        if (status != 0) 
        {
            fprintf(stderr, "Error: unable to configure TX callback with status %d\n", status);
            throw std::runtime_error("Failure: skiq_register_tx_complete_callback");
        }

        /* If in async mode  we need to have an array to keep the pointers to the tx blocks */
        p_tx_blocks = (skiq_tx_block_t **)calloc( NUM_BLOCKS, sizeof( skiq_tx_block_t * ));
        if( p_tx_blocks == NULL )
        {
            fprintf(stderr, "Error: failed to allocate memory for TX blocks.\n");
            throw std::runtime_error("Failure: calloc p_tx_blocks");
        }

        /* If in async mode, we need to have an array of the status of each block */ 
        p_tx_status = (int32_t *)calloc( NUM_BLOCKS, sizeof(*p_tx_status) );
        if( p_tx_status == NULL )
        {
            fprintf(stderr, "Error: failed to allocate memory for TX status.\n");
            throw std::runtime_error("Failure: calloc p_tx_status");
        }

        /* allocate the tx_blocks */
        for (int i = 0; i < NUM_BLOCKS; i++)
        {
            /* allocate a transmit block by number of words */
            p_tx_blocks[i] = skiq_tx_block_allocate( tx_buffer_size );
            p_tx_status[i] = 0;
        }

        printf("Info: transfer mode set to async with threads %d\n", threads);
    }
    else if (threads == 1){
        in_async_mode = false;
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_sync);
        if (status != 0) 
        {
            fprintf(stderr, "Error: unable to configure TX channel mode to sync with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }

        sync_tx_block = skiq_tx_block_allocate( tx_buffer_size );

        printf("Info: transfer mode set to sync\n");
    }
    else {
        fprintf(stderr, "Error: invalid number of threads %d\n", threads);
        throw std::runtime_error("Failure: Invalid threads parameter");
    }
 
    /* all cards have the I/Q in Q/I order so switch it */ 
    status = skiq_write_iq_order_mode(card, skiq_iq_order_iq) ;
    if (status != 0) 
    {
          fprintf(stderr, "Error: unable to set iq order mode to iq with status %d \n", status);
          throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    /* set the frequency and attenuation */
    set_tx_frequency(frequency);
    set_tx_attenuation(attenuation);
}

/*
 * Our virtual destructor.
 */
sidekiq_tx_impl::~sidekiq_tx_impl() 
{
#ifdef DEBUG
    printf("in destructor\n");
#endif

    if (in_async_mode == true)
    {
        /* allocate the tx_blocks */
        for (int i = 0; i < NUM_BLOCKS; i++)
        {
            /* allocate a transmit block by number of words */
            skiq_tx_block_free(p_tx_blocks[i]);
        }
    }
    else
    {
            skiq_tx_block_free(sync_tx_block);
            sync_tx_block = NULL;
    } 

    if (p_tx_blocks != NULL)
    {
        free(p_tx_blocks);
        p_tx_blocks = NULL;
    }
    if (p_tx_status != NULL)
    {
        free(p_tx_status);
        p_tx_status = NULL;
    }

    if (libsidekiq_init == true) 
    {
        skiq_exit();
        libsidekiq_init = false;
    }
}

bool sidekiq_tx_impl::start() 
{
    int status = 0;

#ifdef DEBUG
    printf("in start() \n");
#endif

    status = skiq_start_tx_streaming(card, hdl);
    if (status != 0)
    {
        fprintf(stderr, "Error: could not start TX streaming, status %d\n", status);
        throw std::runtime_error("Failure: skiq_start_tx_streaming");
    }

    tx_streaming = true;

    return block::start();
}

bool sidekiq_tx_impl::stop() 
{
    int status = 0;
    
#ifdef DEBUG
    printf("in stop() \n");
#endif 

    if (tx_streaming == true)
    {
        status = skiq_stop_tx_streaming(card, hdl);
        if (status != 0)
        {
            fprintf(stderr, "Error: could not stop TX streaming, status %d\n", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
    }
    
    return block::stop();
}

void sidekiq_tx_impl::set_tx_sample_rate(double value) 
{

    int status = 0;

#ifdef DEBUG
    printf("in set_tx_sample_rate() \n");
#endif
    auto rate = static_cast<uint32_t>(value);
    auto bw = static_cast<uint32_t>(this->bandwidth);

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        fprintf(stderr, "Error: could not set sample_rate, status %d, %s\n", 
                status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

    status_update_rate_in_samples = 2 * static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

}
  
void sidekiq_tx_impl::set_tx_bandwidth(double value) 
{
    int status = 0;

#ifdef DEBUG
    printf("in set_tx_bandwidth() \n");
#endif

    auto rate = static_cast<uint32_t>(this->sample_rate);
    auto bw = static_cast<uint32_t>(value);

    status = skiq_write_tx_sample_rate_and_bandwidth(card, hdl, rate, bw); 
    if (status != 0) 
    {
        fprintf(stderr,"Error: could not set bandwidth %d, status %d, %s\n", 
                bw, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
        return;
    }

    this->sample_rate = rate;
    this->bandwidth = bw;

}

void sidekiq_tx_impl::set_tx_frequency(double value) 
{
    int status = 0;

#ifdef DEBUG
    printf("in set_tx_frequency() \n");
#endif

    auto freq = static_cast<uint64_t>(value);

    status = skiq_write_tx_LO_freq(card, hdl, freq);
    if (status != 0) 
    {
        fprintf(stderr,"Error: could not set frequency %ld, status %d, %s\n", 
                freq, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: set samplerate");
        return;
    }

    this->frequency = freq;
}


void sidekiq_tx_impl::set_tx_attenuation(double value) 
{
    int status = 0;

#ifdef DEBUG
    printf("in set_tx_attenuation() \n");
#endif

    auto att = static_cast<uint32_t>(value);

    status = skiq_write_tx_attenuation(card, hdl, att);
    if (status != 0)
    {
        fprintf(stderr, "Error: could not set TX attenuation to %d with status %d, %s\n", 
                att, status, strerror(abs(status)) );
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
        return;
    }
    this->attenuation = att;
}

void sidekiq_tx_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required) 
{
    (void)(noutput_items);
    ninput_items_required[0] = tx_buffer_size;
}

void sidekiq_tx_impl::update_tx_error_count() {
    int status = 0;
    uint32_t num_tx_errors;


    status =  skiq_read_tx_num_underruns(card, hdl, &num_tx_errors);
    if (status != 0)
    {
        fprintf(stderr, "Error: skiq_read_tx_num_underruns failed with status %d \n", status);
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
        return;
    }

    if (last_num_tx_errors != num_tx_errors) {
        printf("TX underrun count: %u\n", num_tx_errors);
        last_num_tx_errors = num_tx_errors;
	}
}

int sidekiq_tx_impl::work(
		int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) 
{
	int32_t status{};
	int32_t samples_written{};
    int32_t ninput_items{};

    /* get a pointer to the buffer with the samples to be transmitted */
    auto in = static_cast<const gr_complex *>(input_items[0]);
	
    (void)(output_items);


    /* We should always have noutput_items larger than tx_buffer_size 
     * because we did the "forecast" function */
    if ( noutput_items > tx_buffer_size)
    {
	     ninput_items = noutput_items - (noutput_items % tx_buffer_size);
    }
    else {
        fprintf(stderr, "Error: noutput_items is smaller than tx_buffer_size\n");
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
    }


    /* loop until we have sent the samples we have been given */
	while (samples_written < ninput_items) 
    {
        skiq_tx_block_t *block_ptr;
        int32_t user_info = 0;

        /* transmit the samples */
        if (in_async_mode == true)
        {
            block_ptr = p_tx_blocks[curr_block];
            user_info = p_tx_status[curr_block];
        }
        else
        {
            block_ptr = sync_tx_block;
        }
        
        /* convert the samples we have received to be within the dac_scaling values */
		volk_32f_s32f_multiply_32f(
				reinterpret_cast<float *>(&temp_buffer[0]),
				reinterpret_cast<const float *>(in),
				dac_scaling,
				static_cast<unsigned int>(tx_buffer_size * 2));

        /* convert those samples from float complex to int16 */
		volk_32fc_convert_16ic(
				reinterpret_cast<lv_16sc_t *>(block_ptr->data),
				reinterpret_cast<const lv_32fc_t*>(&temp_buffer[0]),
				tx_buffer_size);

        if (debug_ctr < 5 )
        {
            int ctr = 0;
            for (int i=0; i < 8; i+= 2)
            {

                float re = in[ctr].real();
                printf("%f ", re);
                re = temp_buffer[ctr].real();
                printf("%f ", re);
                printf("%d ", block_ptr->data[i]);

                float im = in[ctr].imag();
                printf("\t%f ", im);
                im = temp_buffer[ctr].imag();
                printf("%f ", im);
                printf("%d \n", block_ptr->data[i+1]);
                ctr++;
            }
            printf(" \n");
            debug_ctr++;
        }
        skiq_tx_set_block_timestamp(block_ptr, timestamp);
	    status = skiq_transmit(card, hdl, block_ptr, &user_info);

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
			printf("Info: sidekiq transmit failed with error: %d\n", status);
		} 
        else {
            samples_written += tx_buffer_size;
            timestamp += tx_buffer_size;

            /* move the pointer */
            in += tx_buffer_size;

            /* loop the tx_block index and wrap */
            curr_block = (curr_block + 1) % NUM_BLOCKS;
        }
	}

    
    /* Determine if  the time has elapsed and display any underruns we have received */
    if (nitems_read(0) - last_status_update_sample > status_update_rate_in_samples) 
    {
        update_tx_error_count();
        last_status_update_sample = nitems_read(0);

#ifdef DEBUG
        printf("noutput_items %d, tx_buffer_size %d, sample_written %d\n", 
                noutput_items, tx_buffer_size, samples_written);
#endif
    }
	
	return samples_written;
}


} /* namespace sidekiq */
} /* namespace gr */
