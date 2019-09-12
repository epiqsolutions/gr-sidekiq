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

#ifndef GR_SIDEKIQ_SIDEKIQ_FUNCTIONS_H
#define GR_SIDEKIQ_SIDEKIQ_FUNCTIONS_H

#include <functional>

namespace gr {
	namespace sidekiq {

		template<typename hdl_type>
		struct sidekiq_functions {

			sidekiq_functions(
					std::function<int32_t(uint8_t, hdl_type)> start_streaming_func,
					std::function<int32_t(uint8_t, hdl_type)> stop_streaming_func,
					std::function<int32_t(uint8_t, hdl_type, uint64_t)> set_frequency_func,
					std::function<int32_t(uint8_t, hdl_type, uint64_t *, double *)> get_frequency_func,
					std::function<int32_t(uint8_t, hdl_type, uint32_t, uint32_t)> set_sample_rate_func,
					std::function<int32_t(uint8_t, hdl_type, uint32_t *, double *, uint32_t *, uint32_t *)>
					get_sample_rate_func,
					std::function<int32_t(uint8_t, hdl_type, uint64_t *)> get_timestamp_func,
					std::function<int32_t(uint8_t, uint8_t *, uint8_t *)> get_rfic_fir_config_func,
					std::function<int32_t(uint8_t, int16_t *)> set_rfic_fir_coeffs_func,
					std::function<int32_t(uint8_t, int16_t *)> get_rfic_fir_coeffs_func) :
                    
					start_streaming_func{start_streaming_func},
					stop_streaming_func{stop_streaming_func},
					set_frequency_func{set_frequency_func},
					get_frequency_func{get_frequency_func},
					set_sample_rate_func{set_sample_rate_func},
					get_sample_rate_func{get_sample_rate_func},
					get_timestamp_func{get_timestamp_func},
					get_rfic_fir_config_func{get_rfic_fir_config_func},
					set_rfic_fir_coeffs_func(set_rfic_fir_coeffs_func),
					get_rfic_fir_coeffs_func{get_rfic_fir_coeffs_func} {
			}

			std::function<int32_t(uint8_t, hdl_type)> start_streaming_func;
			std::function<int32_t(uint8_t, hdl_type)> stop_streaming_func;
			std::function<int32_t(uint8_t, hdl_type, uint64_t)> set_frequency_func;
			std::function<int32_t(uint8_t, hdl_type, uint64_t *, double *)> get_frequency_func;
			std::function<int32_t(uint8_t, hdl_type, uint32_t, uint32_t)> set_sample_rate_func;
			std::function<int32_t(uint8_t, hdl_type, uint32_t *, double *, uint32_t *, uint32_t *)>
					get_sample_rate_func;
			std::function<int32_t(uint8_t, hdl_type, uint64_t *)> get_timestamp_func;
			std::function<int32_t(uint8_t, uint8_t *, uint8_t *)> get_rfic_fir_config_func;
			std::function<int32_t(uint8_t, int16_t *)> set_rfic_fir_coeffs_func;
			std::function<int32_t(uint8_t, int16_t *)> get_rfic_fir_coeffs_func;
		};

	}
}

#endif //GR_SIDEKIQ_SIDEKIQ_FUNCTIONS_H
