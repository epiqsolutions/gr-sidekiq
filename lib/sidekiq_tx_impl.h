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

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H

#include <sidekiq/sidekiq_tx.h>
#include <sidekiq/sidekiq_base.h>

using pmt::pmt_t;

namespace gr {
	namespace sidekiq {

		class sidekiq_tx_impl : public sidekiq_tx, sidekiq_tx_base {
		public:
			sidekiq_tx_impl(
                    int input_card_number,
                    int handle,
					double sample_rate,
					double attenuation,
					double frequency,
					double bandwitdh,
					int sync_type,
					bool suppress_tune_transients,
					uint8_t dataflow_mode,
					int buffer_size);

			int work(
					int noutput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items) override;

			bool start() override;

			bool stop() override;

			void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;

			void set_tx_sample_rate(double value) override;

			void set_tx_attenuation(double value) override;

			void set_tx_frequency(double value) override;

			void set_tx_bandwidth(double value) override;

			void set_tx_suppress_tune_transients(bool value);

			void set_tx_filter_override_taps(const std::vector<float> &taps) override;

		private:
			bool suppress_tune_transients;
			uint64_t timestamp{};
			uint64_t burst_length{};
			uint64_t burst_samples_sent{};
			uint64_t previous_burst_tag_offset{};
			skiq_tx_flow_mode_t dataflow_mode;
			uint32_t last_num_tx_errors{};
			uint16_t tx_buffer_size;
			skiq_tx_block_t *tx_data_block;
			size_t last_status_update_sample{};
			size_t status_update_rate_in_samples{};
			std::vector<int16_t> filter_override_taps;
			std::vector<gr_complex> temp_buffer;

			uint16_t get_tx_attenuation();
			void output_telemetry_message();
			void update_tx_error_count();
			void handle_control_message(pmt::pmt_t message);
			void handle_tx_gain_tag(tag_t tag);
			void handle_tx_freq_tag(tag_t tag);
			void handle_tx_time_tag(tag_t tag);
			void handle_tx_burst_length_tag(tag_t tag);
			void check_burst_length(size_t current_tag_offset, size_t burst_sample_length);
		};
	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H */

