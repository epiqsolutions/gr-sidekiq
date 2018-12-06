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

#include <gnuradio/io_signature.h>
#include "sidekiq_rx_impl.h"
#include <volk/volk.h>
#include <chrono>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>

using namespace gr::sidekiq;
using std::chrono::duration_cast;
using std::chrono::system_clock;
using std::chrono::nanoseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using pmt::pmt_t;


const pmt_t RX_TIME_KEY{pmt::string_to_symbol("rx_time")};

const pmt_t RX_RATE_KEY{pmt::string_to_symbol("rx_rate")};

const pmt_t RX_FREQ_KEY{pmt::string_to_symbol("rx_freq")};

const pmt_t RX_GAIN_KEY{pmt::string_to_symbol("rx_gain")};

const pmt_t CONTROL_MESSAGE_PORT{pmt::string_to_symbol("command")};

const pmt_t TELEMETRY_MESSAGE_PORT{pmt::string_to_symbol("telemetry")};

const size_t DATA_MAX_BUFFER_SIZE{SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS};

static const double STATUS_UPDATE_RATE_SECONDS{1.0};

sidekiq_rx::sptr sidekiq_rx::make(
		double sample_rate,
		double gain,
		uint8_t gain_mode,
		double frequency,
		double bandwidth,
		int sync_type,
		size_t num_items,
		const std::vector<float> &taps) {
	return boost::make_shared<sidekiq_rx_impl>(
			sample_rate,
			gain,
			gain_mode,
			frequency,
			bandwidth,
			sync_type,
			num_items,
			taps
	);
}

sidekiq_rx_impl::sidekiq_rx_impl(
		double sample_rate,
		double gain,
		uint8_t gain_mode,
		double frequency,
		double bandwidth,
		int sync_type,
		size_t num_items,
		const std::vector<float> &taps) :
		gr::sync_block{
				"sidekiq_rx",
				gr::io_signature::make(0, 0, 0),
				gr::io_signature::make(1, 1, sizeof(gr_complex) * num_items)
		},
		sidekiq_rx_base{
				sync_type,
				skiq_rx_hdl_A1,
				gr::sidekiq::sidekiq_functions<skiq_rx_hdl_t>(
						skiq_start_rx_streaming,
						skiq_stop_rx_streaming,
						skiq_write_rx_LO_freq,
						skiq_read_rx_LO_freq,
						skiq_write_rx_sample_rate_and_bandwidth,
						skiq_read_rx_sample_rate_and_bandwidth,
						skiq_read_curr_rx_timestamp,
						skiq_read_rfic_rx_fir_config,
						skiq_write_rfic_rx_fir_coeffs,
						skiq_read_rfic_rx_fir_coeffs
				)
		},
		vector_length{num_items},
		tag_now{true},
		block_id{pmt::string_to_symbol(name())} {
	set_rx_sample_rate(sample_rate);
	set_rx_bandwidth(bandwidth);
	set_rx_frequency(frequency);
	set_rx_gain_mode(gain_mode);
	set_rx_gain(gain);
	set_rx_filter_override_taps(taps);
	get_configuration_limits();
	get_rx_gain_range();

	auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(short));
	set_alignment(alignment_multiple);

	message_port_register_in(CONTROL_MESSAGE_PORT);
	set_msg_handler(CONTROL_MESSAGE_PORT, bind(&sidekiq_rx_impl::handle_control_message, this, _1));
	message_port_register_out(TELEMETRY_MESSAGE_PORT);
}

bool sidekiq_rx_impl::start() {
	if (skiq_set_rx_transfer_timeout(card, RX_TRANSFER_WAIT_FOREVER) != 0) {
		printf("Error: unable to set RX transfer timeout\n");
		this->stop();
	}
	output_telemetry_message();
	start_streaming();
	return block::start();
}

bool sidekiq_rx_impl::stop() {
	stop_streaming();
	skiq_exit();
	return block::stop();
}

void sidekiq_rx_impl::get_rx_gain_range() {
	uint8_t gain_min;
	uint8_t gain_max;

	if (skiq_read_rx_gain_index_range(card, hdl, &gain_min, &gain_max) != 0) {
		printf("Error: could not get gain index range\n");
	}
	printf("\n\nGain min/max: %d %d\n", gain_min, gain_max);
}

uint8_t sidekiq_rx_impl::get_rx_gain_mode() {
	skiq_rx_gain_t result;

	if (skiq_read_rx_gain_mode(card, hdl, &result) != 0) {
		printf("Error: could not get gain mode\n");
	}
	return static_cast<uint8_t >(result);
}

void sidekiq_rx_impl::set_rx_gain_mode(uint8_t value) {
	if (skiq_write_rx_gain_mode(card, hdl, static_cast<skiq_rx_gain_t>(value)) != 0) {
		printf("Error: could not set gain mode to %d\n", value);
	}
}

uint8_t sidekiq_rx_impl::get_rx_gain() {
	uint8_t result;

	if (skiq_read_rx_gain(card, hdl, &result) != 0) {
		printf("Error: could not get gain\n");
	}
	return result;
}

void sidekiq_rx_impl::set_rx_gain(double value) {
	if (skiq_write_rx_gain(card, hdl, static_cast<uint8_t>(value)) != 0) {
		printf("Error: could not set gain to %f\n", value);
	} else {
		tag_now = true;
	}
}

void sidekiq_rx_impl::set_rx_sample_rate(double value) {
	set_samplerate_bandwidth(static_cast<uint32_t>(value), bandwidth);
	status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

}

void sidekiq_rx_impl::set_rx_bandwidth(double value) {
	set_samplerate_bandwidth(sample_rate, static_cast<uint32_t>(value));
//	get_filter_parameters();
}

void sidekiq_rx_impl::set_rx_filter_override_taps(const std::vector<float> &taps) {
	float MAX_SIZE{65536.0f / 4.0f};

	filter_override_taps.clear();
	for (unsigned int count{0}; count < taps.size(); count++) {
		filter_override_taps.push_back(static_cast<int16_t>(taps[count] * MAX_SIZE));
		printf("%03d,%f,%05d\n", count, taps[count], filter_override_taps[count]);
	}
	if (!filter_override_taps.empty()) {
		set_filter_parameters(&filter_override_taps[0]);
	}
//	get_filter_parameters();
}

void sidekiq_rx_impl::set_rx_frequency(double value) {
	if (set_frequency(value)) {
		tag_now = true;
	}
}

void sidekiq_rx_impl::output_telemetry_message() {
	message_port_pub(TELEMETRY_MESSAGE_PORT, get_telemetry_pmt());
}

//TODO: move this to pmt_helper class
double get_double_from_pmt_dict_two(pmt_t dict, pmt_t key, pmt_t not_found = pmt::PMT_NIL) {
	auto message_value = pmt::dict_ref(dict, key, not_found);

	return pmt::to_double(message_value);
}

void sidekiq_rx_impl::handle_control_message(pmt_t message) {
	//NOTE: we are keeping backward compatibility with gr-uhd message types to enable drop in replacement
	//NOTE: card and mboard are USRP specific but could be applied to multiple sidekiq cards
//	size_t chan = get_channel_from_pmt_dict(message);
//	size_t mboard = get_mboard_from_pmt_dict(message);

	if (pmt::dict_has_key(message, FREQ_KEY)) {
		set_rx_frequency(get_double_from_pmt_dict_two(message, FREQ_KEY));
	}
	if (pmt::dict_has_key(message, GAIN_KEY)) {
		set_rx_gain(get_double_from_pmt_dict_two(message, GAIN_KEY));
	}
	if (pmt::dict_has_key(message, RATE_KEY)) {
		set_rx_sample_rate(get_double_from_pmt_dict_two(message, RATE_KEY));
	}
	//TODO: timed freq change must be implemented before this will work
	if (pmt::dict_has_key(message, USRP_TIMED_COMMAND_KEY)) {
//		execute_timed_freq_change_command(message, true);
	}
}

void sidekiq_rx_impl::apply_all_tags(size_t sample_index, size_t timestamp) {
	unsigned int output{};
	const pmt::pmt_t sample_time_pmt = get_pmt_tuple_from_timestamp(timestamp);
	const pmt::pmt_t rate_pmt = pmt::from_double(get_sample_rate());
	const pmt::pmt_t freq_pmt = pmt::from_double(get_frequency());
	const pmt::pmt_t gain_pmt = pmt::from_double(get_rx_gain());

	this->add_item_tag(output, sample_index, RX_TIME_KEY, sample_time_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_RATE_KEY, rate_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_FREQ_KEY, freq_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_GAIN_KEY, gain_pmt, block_id);
	tag_now = false;
}

int sidekiq_rx_impl::work(
		int noutput_items,
		gr_vector_const_void_star &,
		gr_vector_void_star &output_items) {
	unsigned int output_port{};
	int samples_receive_count{};
	unsigned int data_length_bytes{};
	skiq_rx_block_t *p_rx_block{};
	auto out = static_cast<gr_complex *>(output_items[output_port]);

	if (nitems_written(output_port) - last_status_update_sample > status_update_rate_in_samples) {
		printf("Timestamp gap count: %ld\n", timestamp_gap_count);
		last_status_update_sample = nitems_written(output_port);
		printf("Last System Timestamp: %ld\n", get_last_pps_timestamp());
	}

	while ((unsigned int)(noutput_items - samples_receive_count) >= (DATA_MAX_BUFFER_SIZE)) {
		if (skiq_receive(card, &hdl, &p_rx_block, &data_length_bytes) == skiq_rx_status_success) {
			unsigned int buffer_sample_count{
                            static_cast<unsigned int>((data_length_bytes) / (sizeof(short) * IQ_SHORT_COUNT)) -
                                SKIQ_RX_HEADER_SIZE_IN_WORDS
			};

			volk_16i_s32f_convert_32f_u(
					(float *) out,
					(const int16_t *) p_rx_block->data,
					adc_scaling,
					(buffer_sample_count * IQ_SHORT_COUNT));

			if (p_rx_block->rf_timestamp != next_timestamp && next_timestamp != 0) {
				printf("Dropped %ld samples\n", p_rx_block->rf_timestamp - next_timestamp);
				timestamp_gap_count++;
				tag_now = true;
			}

			next_timestamp = p_rx_block->rf_timestamp + buffer_sample_count;
			if (tag_now) {
				auto ts = p_rx_block->sys_timestamp * sidekiq_system_time_interval_nanos;
				apply_all_tags(nitems_written(output_port) + samples_receive_count, ts);
			}
			samples_receive_count += buffer_sample_count;
			out += buffer_sample_count;
		}
	}
	return samples_receive_count;
}
