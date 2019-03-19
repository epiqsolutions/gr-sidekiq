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

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_BASE_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_BASE_H

#include <sidekiq/api.h>
#include <pmt/pmt.h>
#include <sidekiq_api.h>
#include "sidekiq_functions.h"

using pmt::pmt_t;

namespace gr {
	namespace sidekiq {

		static const pmt_t CMD_CURRENT_USRP_TIME{pmt::string_to_symbol("usrp_time")};

		static const pmt_t CMD_CURRENT_HOST_TIME{pmt::string_to_symbol("host_time")};

		static const pmt_t CHAN_KEY{pmt::string_to_symbol("chan")};

		static const pmt_t MBOARD_KEY{pmt::string_to_symbol("mboard")};

		static const pmt_t TIME_KEY{pmt::string_to_symbol("time")};

		static const pmt_t TUNE_KEY{pmt::string_to_symbol("tune")};

		static const pmt_t FREQ_KEY{pmt::string_to_symbol("freq")};

		static const pmt_t RATE_KEY{pmt::string_to_symbol("rate")};

		static const pmt_t GAIN_KEY{pmt::string_to_symbol("gain")};

		static const pmt_t USRP_TIMED_COMMAND_KEY{pmt::string_to_symbol("usrp_timed_command")};

		static const pmt_t TX_TIME_KEY{pmt::string_to_symbol("tx_time")};

		static const pmt_t TX_BURST_LENGTH_KEY{pmt::string_to_symbol("tx_burst_length")};

		static const pmt_t TX_FREQ_KEY{pmt::string_to_symbol("tx_freq")};

		static const pmt_t TX_GAIN_KEY{pmt::string_to_symbol("tx_gain")};

		const int IQ_SHORT_COUNT{2};


		template<typename HdlType>
		class SIDEKIQ_API sidekiq_base {
		public:
			sidekiq_base(int sync_type, HdlType handle_type, gr::sidekiq::sidekiq_functions<HdlType> sidekiq_functions);

		protected:
			uint8_t card;
			HdlType hdl;
			uint32_t sample_rate;
			uint32_t bandwidth;
			size_t sidekiq_system_time_interval_nanos;
			size_t timestamp_frequency;
			gr::sidekiq::sidekiq_functions<HdlType> sidekiq_functions;
                        skiq_param_t sidekiq_params;
                        float adc_scaling;
                        float dac_scaling;

			pmt::pmt_t get_pmt_tuple_from_timestamp(size_t timestamp);
			pmt::pmt_t get_pmt_cons_from_timestamp(size_t timestamp);
			pmt::pmt_t get_telemetry_pmt();
			void set_sync_type(int type);
			void get_configuration_limits();
			int32_t get_ref_clock_configuration();
			void set_zero_timestamp();
			void set_next_pps_timestamp();
			uint64_t get_last_pps_timestamp();
			void set_sidekiq_system_timestamp(uint64_t timestamp);
			uint64_t get_sidekiq_system_timestamp();
			uint64_t get_sys_timestamp_frequency();
			void read_accelerometer();
			uint8_t read_rfic_register(uint16_t address);
			void write_rfic_register(uint16_t address, uint8_t data);
			float read_temperature();

			bool start_streaming();
			bool stop_streaming();
			bool set_frequency(double value);
			double get_frequency();
			bool set_samplerate_bandwidth(double sample_rate, double bandwidth);
			double get_sample_rate();
			uint64_t get_timestamp();
			int64_t get_set_frequency_call_latency();
			void set_filter_parameters(int16_t *coeffs);
			void get_filter_parameters();
		};

		using sidekiq_rx_base = sidekiq_base<skiq_rx_hdl_t>;
		using sidekiq_tx_base = sidekiq_base<skiq_tx_hdl_t>;

	} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_BASE_H */

