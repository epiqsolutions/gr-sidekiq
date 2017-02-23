/* -*- c++ -*- */
/* 
 * Copyright 2013 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "sidekiq_source_s_impl.h"

#define     SIDEKIQ_SAMPLES_PER_PACKET        (1024)
// *2 for I & Q
#define BUF_LEN      (SIDEKIQ_SAMPLES_PER_PACKET*sizeof(short)*2)

namespace gr {
  namespace sidekiq {

      sidekiq_source_s::sptr
      sidekiq_source_s::make(const std::string ip_address, uint32_t port)
      {
	  return gnuradio::get_initial_sptr
	      (new sidekiq_source_s_impl(ip_address, port));
      }

      /*
       * The private constructor
       */
      sidekiq_source_s_impl::sidekiq_source_s_impl(const std::string ip_address,
						   uint32_t port)
	  : gr::sync_block("sidekiq_source",
			   gr::io_signature::make(0, 0, 0),
			   gr::io_signature::make(1, 1, sizeof(short)))
      {
	  rcv.reset( new sidekiq(ip_address.c_str(), port) );
	  set_output_multiple(SIDEKIQ_SAMPLES_PER_PACKET*2);

          std::stringstream str;
          str << name() << "_" << unique_id();
          _id = pmt::string_to_symbol(str.str());
      }

      /*
       * Our virtual destructor.
       */
      sidekiq_source_s_impl::~sidekiq_source_s_impl()
      {
	  rcv->stop();
      }
      
      
      bool 
      sidekiq_source_s_impl::stop()
      {
	  rcv->stop();
	  return true;
      }

      bool 
      sidekiq_source_s_impl::start()
      {
	  rcv->start();
	  return true;
      }

      uint64_t 
      sidekiq_source_s_impl::set_center_freq(uint64_t freq)
      {
	  return (rcv->set_center_freq(freq));
      }

      uint64_t 
      sidekiq_source_s_impl::set_center_freq(float freq)
      {
	  return (rcv->set_center_freq(freq));
      }

      uint64_t 
      sidekiq_source_s_impl::center_freq(void)
      {
	  return (rcv->center_freq());
      }

      uint32_t 
      sidekiq_source_s_impl::set_sample_rate(uint32_t sample_rate)
      {
	  return (rcv->set_sample_rate(sample_rate));
      }

      uint32_t 
      sidekiq_source_s_impl::set_sample_rate(float sample_rate)
      {
	  return (rcv->set_sample_rate(sample_rate));
      }

      uint32_t 
      sidekiq_source_s_impl::sample_rate(void)
      {
	  return (rcv->sample_rate());
      }


      uint32_t 
      sidekiq_source_s_impl::set_bandwidth(uint32_t bandwidth)
      {
	  return (rcv->set_bandwidth(bandwidth));
      }

      uint32_t 
      sidekiq_source_s_impl::set_bandwidth(float bandwidth)
      {
	  return (rcv->set_bandwidth(bandwidth));
      }

      uint32_t 
      sidekiq_source_s_impl::bandwidth(void)
      {
	  return (rcv->bandwidth());
      }
	
      uint8_t 
      sidekiq_source_s_impl::set_rx_gain(uint8_t gain)
      {
	  return (rcv->set_rx_gain(gain));
      }

      uint8_t 
      sidekiq_source_s_impl::rx_gain(void)
      {
	  return (rcv->rx_gain());
      }

      GAIN_MODE 
      sidekiq_source_s_impl::set_rx_gain_mode(GAIN_MODE mode)
      {
	  return (rcv->set_rx_gain_mode(mode));
      }

      GAIN_MODE 
      sidekiq_source_s_impl::rx_gain_mode(void)
      {
	  return (rcv->rx_gain_mode());
      }

      int
      sidekiq_source_s_impl::work(int noutput_items,
				  gr_vector_const_void_star &input_items,
				  gr_vector_void_star &output_items)
      {
	  signed short* out1 =(signed short*) output_items[0];
	  char buffer[BUF_LEN];
	  int num_bytes_rcvd = 0;
	  int out_idx = 0;
	  int recv_result = 0;
          bool add_tag=false;

	  for(int i=0; i<floor(noutput_items*1.0/(2*SIDEKIQ_SAMPLES_PER_PACKET));i++) { 
	      recv_result = rcv->read( &buffer[0], &add_tag, BUF_LEN );
	      if( recv_result < 0 ) {
		  // no data available, send back 0
		  goto end_work;
	      }
	      num_bytes_rcvd += recv_result;
	      memcpy(&out1[out_idx], buffer, SIDEKIQ_SAMPLES_PER_PACKET * sizeof(short)*2);
              out_idx = num_bytes_rcvd/2;
	  }	

          if( add_tag ) {
              add_item_tag(0, nitems_written(0), RX_SAMP_RATE_KEY, 
                           pmt::from_double(rcv->sample_rate()), _id);
              add_item_tag(0, nitems_written(0), RX_BANDWIDTH_KEY, 
                           pmt::from_double(rcv->bandwidth()), _id);
              add_item_tag(0, nitems_written(0), RX_FREQ_KEY, 
                           pmt::from_double(rcv->center_freq()), _id);
              add_item_tag(0, nitems_written(0), RX_GAIN,
                           pmt::from_long((rcv->rx_gain())), _id);
          }

	  
      end_work:
	  return (num_bytes_rcvd/2);
      }

  } /* namespace sidekiq */
} /* namespace gr */

