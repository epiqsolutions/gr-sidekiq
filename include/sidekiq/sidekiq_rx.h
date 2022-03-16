/*
* Copyright 2018    US Naval Research Lab
* Copyright 2018    Epiq Solutions
*
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING. If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_H

#include <sidekiq/api.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <gnuradio/sync_block.h>

#pragma GCC diagnostic pop

namespace gr {
	namespace sidekiq {
		class SIDEKIQ_API sidekiq_rx : virtual public gr::sync_block {
		public:
			typedef std::shared_ptr<sidekiq_rx> sptr;

			static sptr make(
					double sample_rate,
					double gain,
					uint8_t gain_mode,
					double frequency,
					double bandwidth,
                    int port_id,
					int sync_type );

			virtual void set_rx_sample_rate(double value) = 0;

			virtual void set_rx_gain(double value) = 0;

			virtual void set_rx_frequency(double value) = 0;

			virtual void set_rx_bandwidth(double value) = 0;

			virtual void set_rx_filter_override_taps(const std::vector<float> &taps) = 0;
		};

	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_H */

