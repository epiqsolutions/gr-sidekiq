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


#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_H

#include <sidekiq/api.h>
#include <gnuradio/sync_block.h>
#include <sidekiq/sidekiq_defs.h>

namespace gr {
  namespace sidekiq {

    /*!
     * \brief <+description of block+>
     * \ingroup sidekiq
     *
     */
    class SIDEKIQ_API sidekiq_source_s : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<sidekiq_source_s> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of sidekiq::sidekiq.
       *
       * To avoid accidental use of raw pointers, sidekiq::sidekiq's
       * constructor is in a private implementation
       * class. sidekiq::sidekiq::make is the public interface for
       * creating new instances.
       */
      static sptr make(const std::string ip_address, uint32_t port);

       /*! 
	* \brief Set center frequency with Hz resolution.
	* \param freq The frequency in Hz
	* \return the actual center frequency
	*
	* Set the center frequency of the Sidekiq.
	*/
       virtual uint64_t set_center_freq(uint64_t freq) = 0;

       /*! 
	* \brief Set frequency with Hz resolution.
	* \param freq The frequency in Hz
	* \return the actual center frequency
	*
	* Convenience function that uses float parameter to all 
	* engineering notation to be used in GRC.
	*/
       virtual uint64_t set_center_freq(float freq) = 0;

       /*! 
	* \brief Get center frequency with Hz resolution.
	* \return the actual center frequency
	*
	* Get the center frequency of the Sidekiq.
	*/
       virtual uint64_t center_freq(void) = 0;

       /*! 
	* \brief Set the sample rate
	* \param sample_rate The sample rate
	* \return the actual sample rate
	*
	* Set the sample rate of the Sidekiq. To calculate the rate 
	* of samples delivered, the decimation stage needs to be
	* factored in.
	*/
       virtual uint32_t set_sample_rate(uint32_t sample_rate) = 0;

       /*! 
	* \brief Set the sample rate
	* \param sample_rate The sample rate
	* \return the actual sample rate
	*
	* Convenience function that uses float parameter to all engineering
	* notation to be used in GRC.
	*/
       virtual uint32_t set_sample_rate(float sample_rate) = 0;

       /*! 
	* \brief Get the sample rate
	* \return the actual sample rate
	*
	* Get the sample rate of the Sidekiq. To calculate
	* the rate of samples delivered, the decimation stage needs to be
	* factored in.
	*/
       virtual uint32_t sample_rate(void) = 0;

       /*! 
	* \brief Set the bandwidth
	* \param bandwidth The channel bandwidth
	* \return the actual bandwidth
	*
	* Set the channel bandwidth of the Sidekiq. 
	*/
       virtual uint32_t set_bandwidth(uint32_t bandwidth) = 0;

       /*! 
	* \brief Set the bandwidth
	* \param bandwidth The channel bandwidth
	* \return the actual bandwidth
	*
	* Convenience function that uses float parameter to all engineering
	* notation to be used in GRC.
	*/
       virtual uint32_t set_bandwidth(float bandwidth) = 0;

       /*! 
	* \brief Get the channel bandwidth
	* \return the actual channel bandwidth
	*
	* Get the channel bandwidth of the Sidekiq. 
	*/
       virtual uint32_t bandwidth(void) = 0;

       /*!
	* \brief Set the value of the Rx gain 
	* \param gain rx gain value (0-76 dB)
	* \return actual rx gain
	*
	* Set the value of the Rx gain
	*/
       virtual uint8_t set_rx_gain(uint8_t gain) = 0;

       /*!
	* \brief Get the value of the Rx gain 
	* \return actual rx gain
	*
	* Get the value of the Rx gain
	*/
       virtual uint8_t rx_gain(void) = 0;

       /*!
	* \brief Set the rx gain mode
	* \param mode gain mode
	* \return gain mode
	*
	* Set the rx gain mode 
	*/
       virtual GAIN_MODE set_rx_gain_mode(GAIN_MODE mode) = 0;

       /*! 
	* \brief Get the rx gain mode
	* \return gain mode
	*
	* Get the rx gain mode
	*/
       virtual GAIN_MODE rx_gain_mode(void) = 0;

    };

  } // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SOURCE_S_H */

