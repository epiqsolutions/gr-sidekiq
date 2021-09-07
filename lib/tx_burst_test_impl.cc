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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "tx_burst_test_impl.h"
#include <sidekiq/sidekiq_base.h>

using namespace gr::sidekiq;

static const uint64_t MILLIS_IN_NANO = 1e6;

static const uint64_t MICROS_IN_ONE_SECOND = 1e6;

static const uint64_t NANOS_IN_ONE_SECOND = 1e9;

const int MODE_TAG_OUTPUT = 0;

static const double NCO_AMPLITUDE = 1.0;

static const int NCO_FREQUENCY = 1000;

static const double MILLIS_IN_SECOND = 1000;

static const uint64_t TX_START_TIME_INVALID = 0L;

static const uint64_t TX_START_TIME_ADVANCE_SECONDS = 3.0;

static const double BURST_FREQ_CHANGE_DEADTIME_SECONDS = 0.001;


static const pmt_t TELEMETRY_MESSAGE_PORT{pmt::string_to_symbol("telemetry")};


tx_burst_test::sptr tx_burst_test::make(double sample_rate, double burst_len_millis, double burst_interval_millis) {
	return gnuradio::get_initial_sptr
			(new tx_burst_test_impl(sample_rate, burst_len_millis, burst_interval_millis));
}

tx_burst_test_impl::tx_burst_test_impl(double sample_rate, double burst_len_millis, double burst_interval_millis)
		: gr::sync_block(
		"tx_burst_test",
		gr::io_signature::make(0, 0, 0),
		gr::io_signature::make(1, 1, sizeof(gr_complex))),
		  sample_rate{sample_rate} {
	set_burst_length(burst_len_millis);
	set_burst_interval(burst_interval_millis);
	update_nco();

	message_port_register_in(TELEMETRY_MESSAGE_PORT);
	set_msg_handler(TELEMETRY_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_current_usrp_time_message(msg); });
}

bool tx_burst_test_impl::start() {
	return block::start();
}

void tx_burst_test_impl::handle_current_usrp_time_message(pmt_t message) {
	uint64_t time_full_seconds{};
//	double time_fractional_seconds{};
	pmt::pmt_t timespec_p = pmt::dict_ref(message, CMD_CURRENT_USRP_TIME, pmt::PMT_NIL);

	time_full_seconds = pmt::to_uint64(pmt::car(timespec_p));
//	time_fractional_seconds = pmt::to_double(pmt::cdr(timespec_p));
//	printf("USRP Time: %ld %1.9f\n", time_full_seconds, time_fractional_seconds);
	if (current_time_nanos == TX_START_TIME_INVALID) {
		current_time_nanos = time_full_seconds + TX_START_TIME_ADVANCE_SECONDS;
		current_time_nanos *= NANOS_IN_ONE_SECOND;
	}
}

void tx_burst_test_impl::update_nco() {
	d_nco.set_freq(static_cast<float>(2 * M_PI * NCO_FREQUENCY / sample_rate));
}


void tx_burst_test_impl::set_sample_rate(double sample_rate) {
	this->sample_rate = sample_rate;
}

void tx_burst_test_impl::set_burst_length(double burst_length_millis) {
	burst_len_seconds = burst_length_millis / MILLIS_IN_SECOND;
	burst_length_nanos = static_cast<uint64_t >(burst_length_millis) * MILLIS_IN_NANO;
}

void tx_burst_test_impl::set_burst_interval(double burst_interval_millis) {
	burst_interval_nanos = static_cast<uint64_t >(burst_interval_millis) * MILLIS_IN_NANO;
}

void tx_burst_test_impl::configuration_tag_insert_pair(uint64_t index, pmt_t key, pmt_t value) {
	add_item_tag(MODE_TAG_OUTPUT, index, key, value, alias_pmt());
}

void tx_burst_test_impl::configuration_tag_insert_pmt_object(uint64_t index, pmt_t key, pmt_t value) {
	add_item_tag(MODE_TAG_OUTPUT, index, key, value, alias_pmt());
}

pmt_t tx_burst_test_impl::get_usrp_time(uint64_t time_nanos) {
	uint64_t seconds{time_nanos / NANOS_IN_ONE_SECOND};
	double fractional_seconds{static_cast<double>(time_nanos % NANOS_IN_ONE_SECOND)};

	fractional_seconds /= NANOS_IN_ONE_SECOND;
	return pmt::make_tuple(pmt::from_uint64(seconds), pmt::from_double(fractional_seconds));
}

int tx_burst_test_impl::work(
		int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {
	unsigned int output_port{};
	auto out = static_cast<gr_complex *>(output_items[output_port]);

	int num_burst_samples{static_cast<int>(sample_rate * burst_len_seconds)};
	int input_index{};

	(void)(input_items);
	
	if (current_time_nanos == TX_START_TIME_INVALID) {
		//we wait for the telemetry message to come in to set initial time
		noutput_items = 0;
	}

	while (input_index < noutput_items) {
		if (burst_samples_remaining == 0) {
			burst_samples_remaining = num_burst_samples;
			configuration_tag_insert_pair(
					nitems_written(0) + input_index,
					TX_BURST_LENGTH_KEY,
					pmt::from_long((long) burst_samples_remaining));
			configuration_tag_insert_pair(
					nitems_written(0) + input_index,
					TX_TIME_KEY,
					get_usrp_time(current_time_nanos));
			current_time_nanos += burst_interval_nanos;
		}
		int remaining = std::min(burst_samples_remaining, noutput_items - input_index);
		d_nco.sincos(&out[input_index], remaining, NCO_AMPLITUDE);
		burst_samples_remaining -= remaining;
		input_index += remaining;
	}

	return noutput_items;
}
