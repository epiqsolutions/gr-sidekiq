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

#ifndef INCLUDED_SIDEKIQ_TX_BURST_TEST_IMPL_H
#define INCLUDED_SIDEKIQ_TX_BURST_TEST_IMPL_H

#include <sidekiq/tx_burst_test.h>
#include <gnuradio/fxpt_nco.h>

using pmt::pmt_t;
using pmt::mp;

namespace gr {
	namespace sidekiq {

		class tx_burst_test_impl : public tx_burst_test {

		public:
			tx_burst_test_impl(
					double sample_rate,
					double burst_len_millis,
					double burst_interval_millis);

			int work(
					int noutput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items) override;

			void set_sample_rate(double sample_rate) override;
			void set_burst_length(double burst_length_millis) override;
			void set_burst_interval(double burst_interval_millis) override;
			void configuration_tag_insert_pair(uint64_t index, pmt_t key, pmt_t value);
			void configuration_tag_insert_pmt_object(uint64_t index, pmt_t key, pmt_t value);
			pmt_t get_usrp_time(uint64_t time_nanos);
			bool start() override;

		private:
			double sample_rate;
			double burst_len_seconds;
			uint64_t burst_length_nanos;
			int burst_samples_remaining{};
			uint64_t burst_interval_nanos;
			gr::fxpt_nco d_nco;
			uint64_t current_time_nanos{};

			void update_nco();
			void handle_current_usrp_time_message(pmt_t message);
		};
	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_TX_BURST_TEST_IMPL_H */

