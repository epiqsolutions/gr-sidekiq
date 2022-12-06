/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_H

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

using pmt::pmt_t;

namespace gr {
namespace sidekiq {

class SIDEKIQ_API sidekiq_tx : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<sidekiq_tx> sptr;

    static sptr make(
                        int card,
                        int handle,
                        double sample_rate,
                        double bandwidth,
                        double frequency,
                        double attenuation,
                        double bursting,
                        int threads,
                        int buffer_size,
                        int cal_mode);

            virtual void set_tx_sample_rate(double value) = 0;

            virtual void set_tx_attenuation(double value) = 0;

            virtual void set_tx_frequency(double value) = 0;

            virtual void set_tx_bandwidth(double value) = 0;

            virtual void set_tx_cal_mode(int value) = 0;

            virtual void run_tx_cal(int value) = 0;


};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_H */
