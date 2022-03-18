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

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H

#include <sidekiq/sidekiq_rx.h>
#include <sidekiq/sidekiq_base.h>

#define MAX_PORT 2 


namespace gr {
	namespace sidekiq {

		class sidekiq_rx_impl : public sidekiq_rx, sidekiq_rx_base {
		public:
			sidekiq_rx_impl(
                    int _card,
                    int port_id,
                    int port_id2,
					double sample_rate,
					double gain,
					uint8_t gain_mode,
					double frequency,
					double bandwitdh,
					int sync_type);

			int work(
					int noutput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items) override;

			bool start() override;

			bool stop() override;

			void set_rx_sample_rate(double value) override;

			void set_rx_gain(double value) override;

			void set_rx_frequency(double value) override;

			void set_rx_bandwidth(double bandwidth) override;

			void set_rx_filter_override_taps(const std::vector<float> &taps) override;


		private:
			size_t vector_length;
			bool tag_now;
			size_t timestamp_gap_count[MAX_PORT]{0,0};
			uint64_t next_timestamp[MAX_PORT]{0,0};
			pmt::pmt_t block_id;
			size_t last_status_update_sample[MAX_PORT]{0,0};
			size_t status_update_rate_in_samples{};
			std::vector<int16_t> filter_override_taps;

			uint8_t get_rx_gain_mode();
			void set_rx_gain_mode(uint8_t value);
			double get_rx_gain(int handle);
            void get_rx_gain_range( double *p_min_gain, double *p_max_gain );
			void output_telemetry_message();
			void handle_control_message(pmt::pmt_t message);
			void apply_all_tags(size_t sample_index, size_t timestamp);
		};
	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_IMPL_H */

