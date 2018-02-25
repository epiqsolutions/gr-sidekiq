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

#include <cppunit/TestAssert.h>
#include "qa_tx_burst_test.h"
#include "sidekiq_rx_impl.h"
#include <sidekiq/tx_burst_test.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/top_block.h>
#include <volk/volk_complex.h>

namespace gr {
	namespace sidekiq {


		void volk_standin(lv_16sc_t *outputVector, const lv_32fc_t *inputVector, unsigned int num_points) {
			float *inputVectorPtr = (float *) inputVector;
			int16_t *outputVectorPtr = (int16_t *) outputVector;
			const float min_val = (float) SHRT_MIN;
			const float max_val = (float) SHRT_MAX;
			float aux;
			unsigned int i;

			for (i = 0; i < num_points * 2; i++) {
				aux = *inputVectorPtr++;
				if (aux > max_val) {
					aux = max_val;
				} else if (aux < min_val) {
					aux = min_val;
				}
				printf("%d\n", (int16_t) rintf(aux));
				*outputVectorPtr++ = (int16_t) rintf(aux);
			}
		}

		void qa_tx_burst_test::volk_test() {
			gr_complex input = {.1, .5};
			int16_t output[2];

			volk_standin(reinterpret_cast<lv_16sc_t *>(&output), &input, 1);

			printf("%d %d %f %f\n", output[0], output[1], input.real(), input.imag());
		}

		void qa_tx_burst_test::t1() {
			int num_taps{128};
			double gain{1.0};
			double sample_rate{1e6};
			double rolloff{0.5};
			double samples_per_symbol{2.0};
			double frequency{2400e6};
			const std::vector<float> filter_override_taps = {0.1, 0.2, 0.34};

			gr::top_block_sptr top_block = gr::make_top_block("test");
			auto sidekiq_recv = sidekiq_rx::make(sample_rate, 1, 1, frequency, .5e6, 1, 1, filter_override_taps);
			auto nullsink = blocks::null_sink::make(sizeof(gr_complex));

			auto rrc_filter = filter::firdes::root_raised_cosine(
					gain,
					sample_rate,
					sample_rate / samples_per_symbol,
					rolloff,
					num_taps
			);
			auto gaussian_filter = filter::firdes::gaussian(gain, samples_per_symbol, rolloff, num_taps);

			(*top_block).connect(sidekiq_recv, 0, nullsink, 0);
			top_block->run(10000000);
		}
	} /* namespace sidekiq */
} /* namespace gr */
