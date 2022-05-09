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

static const double STATUS_UPDATE_RATE_SECONDS{2.0};

//#define DEBUG_1PPS_TIMESTAMP (1)

sidekiq_rx::sptr sidekiq_rx::make(
        int input_card_number,
        int port1_handle,
        int port2_handle,
		double sample_rate,
		double gain,
		uint8_t gain_mode,
		double frequency,
		double bandwidth,
		int sync_type) {
	return std::make_shared<sidekiq_rx_impl>(
            input_card_number,
            port1_handle,
            port2_handle,
			sample_rate,
			gain,
			gain_mode,
			frequency,
			bandwidth,
			sync_type
	);
}

sidekiq_rx_impl::sidekiq_rx_impl(
        int input_card_number,
        int port1_handle,
        int port2_handle,
		double sample_rate,
		double gain,
		uint8_t gain_mode,
		double frequency,
		double bandwidth,
		int sync_type) :
		gr::sync_block{
				"sidekiq_rx",
				gr::io_signature::make(0, 0, 0),
				gr::io_signature::make(1, 2, sizeof(gr_complex) * DATA_MAX_BUFFER_SIZE)
		},
		sidekiq_rx_base{
                input_card_number,
				sync_type,
				(skiq_rx_hdl_t)port1_handle,
				(skiq_rx_hdl_t)port2_handle,
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
		tag_now{true},
		block_id{pmt::string_to_symbol(name())} {
	set_rx_sample_rate(sample_rate);
	set_rx_bandwidth(bandwidth);
	set_rx_frequency(frequency);
	set_rx_gain_mode(gain_mode);
	set_rx_gain(gain);
	get_configuration_limits();

	vector_length = DATA_MAX_BUFFER_SIZE;
	auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(short));
	set_alignment(alignment_multiple);

	message_port_register_in(CONTROL_MESSAGE_PORT);
	set_msg_handler( CONTROL_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });
    ctr = 0;


	message_port_register_out(TELEMETRY_MESSAGE_PORT);
}

bool sidekiq_rx_impl::start() {
	if (skiq_set_rx_transfer_timeout(card, RX_TRANSFER_WAIT_FOREVER) != 0) {
		printf("Error: unable to set RX transfer timeout\n");
		this->stop();
	}
	output_telemetry_message();

    //this will start both ports if in dual mode
	start_streaming();
	return block::start();
}

bool sidekiq_rx_impl::stop() {
    //this will stop both ports if in dual mode
	stop_streaming();
	skiq_exit();
	return block::stop();
}

uint8_t sidekiq_rx_impl::get_rx_gain_mode() {
	skiq_rx_gain_t result;

    /* gain mode is same for both channels */
	if (skiq_read_rx_gain_mode(card, hdl, &result) != 0) {
		printf("Error: could not get gain mode\n");
		this->stop();
	}
	return static_cast<uint8_t >(result);
}

void sidekiq_rx_impl::set_rx_gain_mode(uint8_t value) {
    if(dual_channel) {

        if (skiq_write_rx_gain_mode(card, hdl2, static_cast<skiq_rx_gain_t>(value)) != 0) {
            printf("Error: could not set gain mode to %d\n", value);
        }
    }
	if (skiq_write_rx_gain_mode(card, hdl, static_cast<skiq_rx_gain_t>(value)) != 0) {
		printf("Error: could not set gain mode to %d\n", value);
	}
}

double sidekiq_rx_impl::get_rx_gain(int handle) {
        double cal_offset;

	if (skiq_read_rx_cal_offset(card, (skiq_rx_hdl_t)handle, &cal_offset) != 0) {
		printf("Error: could not get calibration offset\n");
		this->stop();
	}

	return cal_offset;
}

void sidekiq_rx_impl::get_rx_gain_range( double *p_min_gain, double *p_max_gain )
{
    // determine the minimum and maximum gain we can achieve
    // same for all ports
    skiq_read_rx_cal_offset_by_gain_index( card,
                                           hdl,
                                           sidekiq_params.rx_param[hdl].gain_index_min,
                                           p_min_gain );
    skiq_read_rx_cal_offset_by_gain_index( card,
                                           hdl,
                                           sidekiq_params.rx_param[hdl].gain_index_max,
                                           p_max_gain );
}

void sidekiq_rx_impl::set_rx_gain(double value) {    
    double min_cal_offset;
    double max_cal_offset;
    uint8_t updated_gain_index=0;
    double updated_cal_offset=0;
    double prev_cal_offset=0;
    bool found_gain_index=false;

    get_rx_gain_range( &min_cal_offset, &max_cal_offset );
    printf("Gain range for current frequency is %f - %f\n", min_cal_offset, max_cal_offset);

    if( value <= min_cal_offset )
    {
        updated_gain_index = sidekiq_params.rx_param[hdl].gain_index_min;
        updated_cal_offset = min_cal_offset;
    }
    else if( value >= max_cal_offset )
    {
        updated_gain_index = sidekiq_params.rx_param[hdl].gain_index_max;
        updated_cal_offset = max_cal_offset;
    }
    else
    {
        // TODO: a more effective search should be used to determine optimal gain index
        prev_cal_offset = min_cal_offset;
        for( uint8_t i=(sidekiq_params.rx_param[hdl].gain_index_min)+1;
             (i<sidekiq_params.rx_param[hdl].gain_index_max) && (found_gain_index==false);
             i++ )
        {
            // new read the actual calibration offset with the new gain index
            skiq_read_rx_cal_offset_by_gain_index( card,
                                                   hdl,
                                                   i,
                                                   &updated_cal_offset );
            if( value >= prev_cal_offset &&
                value <= updated_cal_offset )
            {
                found_gain_index = true;
                updated_gain_index=i--;
                updated_cal_offset=prev_cal_offset;
                printf("Found gain index %u, actual gain is %f\n",
                       updated_gain_index, updated_cal_offset);
            }
            else
            {
                prev_cal_offset = updated_cal_offset;
            }
        }
    }
    printf("Updated gain is %f for gain index %u\n",
           updated_cal_offset, updated_gain_index);

    if (dual_channel) {
        if (skiq_write_rx_gain(card, hdl2, updated_gain_index) != 0) {
            printf("Error: could not set gain to %f\n", value);
        }
    }

    if (skiq_write_rx_gain(card, hdl, updated_gain_index) != 0) {
        printf("Error: could not set gain to %f\n", value);
    } else {
        tag_now = true;
    }
}

void sidekiq_rx_impl::set_rx_sample_rate(double value) {
    int status;
    skiq_chan_mode_t chan_mode = skiq_chan_mode_single;

    // see if we are doing dual channel
    // single channel is indicated by hdl2 being larger than rx_hdl_end
    if (hdl2 < skiq_rx_hdl_end) {
        dual_channel = true;

        // if either port is A2 or B2 we need to configure in dual channel mode
        // even if we are only streaming one port.
        if (hdl2 == skiq_rx_hdl_A2 || hdl2 == skiq_rx_hdl_B2 )
        {
            chan_mode = skiq_chan_mode_dual;
        }
    }

    if (hdl == skiq_rx_hdl_A2 || hdl == skiq_rx_hdl_B2 )
    {
        chan_mode = skiq_chan_mode_dual;
    }

    /* set the channel mode */
    status = skiq_write_chan_mode(card, chan_mode);
    if ( status != 0 )
    {
        printf("Error: failed to set Rx channel mode to %u with status %d (%s)\n", chan_mode, status, strerror(abs(status)));
		this->stop();
    }

	set_samplerate_bandwidth(static_cast<uint32_t>(value), bandwidth);

	status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);

}

void sidekiq_rx_impl::set_rx_bandwidth(double value) {

    // This handles both ports if in dual channel mode
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

    // this will work for both ports if in dual mode
    set_frequency(value);
    tag_now = true;
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
	const pmt::pmt_t rate_pmt = pmt::from_double(get_sample_rate(hdl));
	const pmt::pmt_t freq_pmt = pmt::from_double(get_frequency(hdl));
	const pmt::pmt_t gain_pmt = pmt::from_double(get_rx_gain(hdl));

	this->add_item_tag(output, sample_index, RX_TIME_KEY, sample_time_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_RATE_KEY, rate_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_FREQ_KEY, freq_pmt, block_id);
	this->add_item_tag(output, sample_index, RX_GAIN_KEY, gain_pmt, block_id);
	tag_now = false;
}

/****************************************************************************/
/**
 * The work function is called by GNURadio when it wants data put into it's 
 * buffers from the radio
 * 
 * Since this is a streaming service the result is expecting a "vector" back
 * In this case the vector length is the number words stored in each "buffer"
 * received in the output_items array.
 *
 * In the constructor of this object, we tell GNURadio that we can have at most
 * 2 ports with a vector size of sizeof(gr-complex) * vector_size which is
 * the number of words in a block.
 *
 * If there are two ports then work gets called for both ports at the same time.  
 * Each port will have its own buffer in the output_items array 
 *
 * The noutput_items is the number of vectors to put into each buffer.  So the 
 * actual size of each buffer is noutput_items * vector_size number of words.
 *
 * So if noutput_items is greater than one, this routine will need to wait for multiple 
 * skiq_receive buffers to fill it.
 */
int sidekiq_rx_impl::work(
		int noutput_items,
		gr_vector_const_void_star &,
		gr_vector_void_star &output_items) 
{
	int samples_receive_count[MAX_PORT]{};
	gr_complex *out[MAX_PORT];
    static size_t last_timestamp_gap_update[MAX_PORT]{0,0} ;
	unsigned int data_length_bytes{};
	int samples_to_rx = noutput_items*vector_length;
	int num_vectors=0;
	skiq_rx_block_t *p_rx_block{};
    int port = 0;
    int num_ports;
    bool looping = false;
    skiq_rx_hdl_t tmp_hdl;
    int status = 0;

	out[0] = static_cast<gr_complex *>(output_items[0]);
	out[1] = static_cast<gr_complex *>(output_items[1]);

    if (dual_channel) {
        num_ports = 2;
    } else { 
        num_ports = 1;
    }

    /* This prints out the timestamp gaps every STATUS_UPDATE_RATE_SECONDS */ 
    for (port = 0; port < num_ports; port++) {
        if ((nitems_written(port) * vector_length) - last_status_update_sample[port] > status_update_rate_in_samples) {
            if( timestamp_gap_count[port] != last_timestamp_gap_update[port] ) {
                printf("Port %d, Timestamp gap count: %ld\n", port, timestamp_gap_count[port]);
                last_timestamp_gap_update[port] = timestamp_gap_count[port];
            }
            last_status_update_sample[port] = vector_length * nitems_written(port);
#ifdef DEBUG_1PPS_TIMESTAMP                
            printf("Last System Timestamp: %ld\n", get_last_pps_timestamp());
#endif                
        }
    }
    
    /* we need to continue looping until both channels have filled up the respective buffer */
    if (dual_channel) {
        looping = 
            ((unsigned int)(samples_to_rx - samples_receive_count[0]) >= (DATA_MAX_BUFFER_SIZE) &&
	        (unsigned int)(samples_to_rx - samples_receive_count[1]) >= (DATA_MAX_BUFFER_SIZE));
    } else { 
        looping = 
            ((unsigned int)(samples_to_rx - samples_receive_count[0]) >= (DATA_MAX_BUFFER_SIZE)); 
    }

	while ( looping ) {
        /* block until either channel gets a block */
		status = skiq_receive(card, &tmp_hdl, &p_rx_block, &data_length_bytes);
		if (status  == skiq_rx_status_success) {
            /* determine which port gave us a block */
            if (tmp_hdl == hdl) {
                port = 0;
            } else if (tmp_hdl == hdl2) {
                port = 1;
            } else {
                printf("Error : Received a block from an unknown handle %d\n", tmp_hdl);
                this->stop();
            }

            /* determine how many samples are in the received block */
			unsigned int buffer_sample_count{
                     static_cast<unsigned int>((data_length_bytes) / (sizeof(short) * IQ_SHORT_COUNT)) -
                                SKIQ_RX_HEADER_SIZE_IN_WORDS
			};
            
            /* convert from a block of int16x2 samples to the output buffer of floatx2 samples */
			volk_16i_s32f_convert_32f_u(
					(float *) out[port],
					(const int16_t *) p_rx_block->data,
					adc_scaling,
					(buffer_sample_count * IQ_SHORT_COUNT));

            /* determine if we droped a block somewhere and lost samples */
			if (p_rx_block->rf_timestamp != next_timestamp[port] && next_timestamp[port] != 0) {
				printf("Dropped %ld samples\n", p_rx_block->rf_timestamp - next_timestamp[port]);
				timestamp_gap_count[port]++;
				tag_now = true;
			}

            /* calculate what the next timestamp will be */
			next_timestamp[port] = p_rx_block->rf_timestamp + buffer_sample_count;

            /* if something happened to insert a tag do it */
			if (tag_now) {
				auto ts = p_rx_block->sys_timestamp * sidekiq_system_time_interval_nanos;
				apply_all_tags(nitems_written(port) + samples_receive_count[port], ts);
			}

            /* each block is one vector long */
			num_vectors++;

            /* update the counters */
			samples_receive_count[port] += buffer_sample_count;
			out[port] += buffer_sample_count;
		} else {
            printf("Error : skiq_rcv failure, status %d\n", status);
            this->stop();
        }

        /* determine if we should still be looping or if we are done */
        if (dual_channel) {
            looping = 
                ((unsigned int)(samples_to_rx - samples_receive_count[0]) >= (DATA_MAX_BUFFER_SIZE) &&
                (unsigned int)(samples_to_rx - samples_receive_count[1]) >= (DATA_MAX_BUFFER_SIZE));
        }else { 
            looping = 
                ((unsigned int)(samples_to_rx - samples_receive_count[0]) >= (DATA_MAX_BUFFER_SIZE)); 
        }

	}

    /* if we are done and the number of vectors processes is not noutput_items, then something bad happened */
    if (num_vectors != noutput_items) {
        printf("Error : Number of vectors processed is invalid, num_vectors %d\n", num_vectors);
        this->stop();
    }


	return num_vectors;
}
