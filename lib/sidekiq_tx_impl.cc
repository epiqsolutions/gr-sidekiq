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

#define NUM_BLOCKS  20
const bool SIDEKIQ_IQ_PACK_MODE_UNPACKED{false};

static const double STATUS_UPDATE_RATE_SECONDS{2.0};
static uint32_t complete_count{};

/* The tx_complete function needs to be outside the object so it can be registered with libsidekiq */
// mutex to protect updates to the tx buffer
static pthread_mutex_t tx_buf_mutex;
  
// mutex and condition variable to signal when the tx queue may have room available
static pthread_mutex_t space_avail_mutex;
static pthread_cond_t space_avail_cond;

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


/*
 * The private constructor
 */
sidekiq_tx_impl::sidekiq_tx_impl( int input_card,
                                  int handle,
                                  double sample_rate,
                                  double bandwidth,
                                  double frequency,
                                  double attenuation,
                                  int threads,
                                  int buffer_size)
    : gr::sync_block("sidekiq_tx",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0))   //sync block
{
    printf("in constructor\n");
    printf("card %d, handle %d rate %f, bandwidth %f, \nfrequency %f, atten %f, threads %d, buffer_size %d\n", 
            input_card, handle, sample_rate, bandwidth, frequency, attenuation, threads, buffer_size);

    int status = 0;
    uint8_t iq_resolution = 0;

    card = input_card;
    hdl = (skiq_tx_hdl_t)handle;
    curr_block = 0;
    tx_buffer_size = buffer_size;
    temp_buffer.resize(tx_buffer_size);
    num_blocks = NUM_BLOCKS;

    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    libsidekiq_init = true;
    printf("Info: libsidkiq initialized successfully\n");

    set_tx_sample_rate(sample_rate);
    set_tx_bandwidth(bandwidth);

    status = skiq_read_tx_iq_resolution(card, &iq_resolution);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to initialize libsidekiq with status %d\n", status);
        throw std::runtime_error("Failure: skiq_init");
    }
    dac_scaling = (pow(2.0f, iq_resolution) / 2.0)-1;


    printf("Info: dac scaling %f\n", dac_scaling);

    status = skiq_write_tx_data_flow_mode(card, hdl, skiq_tx_immediate_data_flow_mode);
    if (status != 0) 
    {
        fprintf(stderr, "Error: could not set TX dataflow mode with status %d\n", status);
        throw std::runtime_error("Failure: skiq_write_tx_flow_mode");
    }

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

    status = skiq_write_tx_block_size(card, hdl, tx_buffer_size);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to configure TX block size: %d with status %d\n", 
                tx_buffer_size, status);
        throw std::runtime_error("Failure: skiq_write_tx_block_size");
    }
    printf("Info: TX block size %d\n", tx_buffer_size);

    if (threads > 1)
    {  
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_async);
        if (status != 0) 
        {
            fprintf(stderr, "Error: unable to configure TX channel mode with status %d\n", status);
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
    }
    else {
        status = skiq_write_tx_transfer_mode(card, hdl, skiq_tx_transfer_mode_sync);
        if (status != 0) 
        {
            fprintf(stderr, "Error: unable to configure TX channel mode with status %d\n", status);
            throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
        }
        num_blocks = 1;
    }
 
    status = skiq_write_iq_pack_mode(card, SIDEKIQ_IQ_PACK_MODE_UNPACKED);
    if (status != 0) 
    {
        fprintf(stderr, "Error: unable to set iq pack mode to unpacked with status %d\n", status);
        throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }
  
    status = skiq_write_iq_order_mode(card, skiq_iq_order_iq) ;
    if (status != 0) 
    {
          fprintf(stderr, "Error: unable to set iq order mode to iq with status %d \n", status);
          throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
    }

    p_tx_blocks = (skiq_tx_block_t **)calloc( num_blocks, sizeof( skiq_tx_block_t * ));
    if( p_tx_blocks == NULL )
    {
        fprintf(stderr, "Error: failed to allocate memory for TX blocks.\n");
        throw std::runtime_error("Failure: calloc p_tx_blocks");
    }

    // allocate for # blocks
    p_tx_status = (int32_t *)calloc( num_blocks, sizeof(*p_tx_status) );
    if( p_tx_status == NULL )
    {
        fprintf(stderr, "Error: failed to allocate memory for TX status.\n");
        throw std::runtime_error("Failure: calloc p_tx_status");
    } 
    for (uint32_t i = 0; i < num_blocks; i++)
    {
        /* allocate a transmit block by number of words */
        p_tx_blocks[i] = skiq_tx_block_allocate( tx_buffer_size );
        p_tx_status[i] = 0;
    }

    set_tx_frequency(frequency);
    set_tx_attenuation(attenuation);
}

sidekiq_tx_impl::~sidekiq_tx_impl() 
{
    printf("in destructor\n");

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
}

bool sidekiq_tx_impl::start() 
{
    int status = 0;

    printf("in start() \n");

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
    
    printf("in stop() \n");

    if (tx_streaming == true)
    {
        status = skiq_stop_tx_streaming(card, hdl);
        if (status != 0)
        {
            fprintf(stderr, "Error: could not stop TX streaming, status %d\n", status);
            throw std::runtime_error("Failure: skiq_start_tx_streaming");
        }
    }
    
    if (libsidekiq_init == true)
    {
        skiq_exit();
    }

    return block::stop();
}

void sidekiq_tx_impl::set_tx_sample_rate(double value) 
{

    int status = 0;
    printf("in set_tx_sample_rate() \n");

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

    status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

}
  
void sidekiq_tx_impl::set_tx_bandwidth(double value) 
{
    int status = 0;
    printf("in set_tx_bandwidth() \n");

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
    printf("in set_tx_frequency() \n");

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
    printf("in set_tx_attenuation() \n");

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
        /* convert the samples we have received to be within the dac_scaling values */
		volk_32f_s32f_multiply_32f(
				reinterpret_cast<float *>(&temp_buffer[0]),
				reinterpret_cast<const float *>(in),
				dac_scaling,
				static_cast<unsigned int>(tx_buffer_size * 2));

        /* convert those samples from float complex to int16 */
		volk_32fc_convert_16ic(
				reinterpret_cast<lv_16sc_t *>(p_tx_blocks[curr_block]->data),
				reinterpret_cast<const lv_32fc_t*>(&temp_buffer[0]),
				tx_buffer_size);
		

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
			printf("Info: sidekiq transmit failed with error: %d\n", status);
		} 
        else {
            samples_written += tx_buffer_size;

            /* move the pointer */
            in += tx_buffer_size;

            /* loop the tx_block index and wrap */
            curr_block = (curr_block + 1) % num_blocks;
        }
	}

    
    /* Determine if a second has elapsed and display any underruns we have received */
    if (nitems_read(0) - last_status_update_sample > status_update_rate_in_samples) 
    {
        update_tx_error_count();
        last_status_update_sample = nitems_read(0);

#define DEBUG
#ifdef DEBUG
        printf("noutput_items %d, tx_buffer_size %d, sample_written %d\n", 
                noutput_items, tx_buffer_size, samples_written);
#endif
    }
	
	return samples_written;
}


} /* namespace sidekiq */
} /* namespace gr */
