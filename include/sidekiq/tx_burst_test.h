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

#ifndef INCLUDED_SIDEKIQ_TX_BURST_TEST_H
#define INCLUDED_SIDEKIQ_TX_BURST_TEST_H

#include <sidekiq/api.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <gnuradio/sync_block.h>

#pragma GCC diagnostic pop

namespace gr {
	namespace sidekiq {

		class SIDEKIQ_API tx_burst_test : virtual public gr::sync_block {
		public:
			typedef std::shared_ptr<tx_burst_test> sptr;

			static sptr make(double sample_rate, double burst_len_millis, double burst_interval_millis);

			virtual void set_sample_rate(double sample_rate) = 0;
			virtual void set_burst_length(double burst_length) = 0;
			virtual void set_burst_interval(double burst_interval) = 0;

		};

	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_TX_BURST_TEST_H */

