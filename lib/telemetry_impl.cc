/* -*- c++ -*- */
/*
 * Copyright 2023 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "telemetry_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sidekiq {

using input_type = float;
telemetry::sptr telemetry::make(
        int input_card,
        int temp_enabled,
        int imu_enabled) 
{
  return gnuradio::make_block_sptr<telemetry_impl>(
          input_card,
          temp_enabled,
          imu_enabled);
}

/*
 * The private constructor
 */
telemetry_impl::telemetry_impl(
        int input_card,
        int temp_enabled,
        int imu_enabled) 
    : gr::sync_block("telemetry",
                     gr::io_signature::make(0 /* min inputs */,
                                            0 /* max inputs */,
                                            0),
                     gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */,
                                            sizeof(gr_complex)))

{
    int status;
//    bool libsidekiq_init;
    uint8_t card;

    card = input_card;

    printf("card %d, temp_enabled %d, imu_enabled %d\n", input_card, temp_enabled, imu_enabled); 
    /* initialize libsidekiq */
    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_basic, &card, 1);
    if (status != 0)
    {
        if (status != -EEXIST)
        {
            d_logger->error( "Error: unable to initialize libsidekiq with status {}", status);
            throw std::runtime_error("Failure: skiq_init");
        }
        else
        {
            d_logger->info("Info: If not running Transceive Mode, then this is an error");
        }
    }
    else
    {
//        libsidekiq_init = true;
        d_logger->info("Info: libsidkiq initialized successfully");
    }



}

/*
 * Our virtual destructor.
 */
telemetry_impl::~telemetry_impl() {}

int telemetry_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items) {

  // Do <+signal processing+>
    printf("in work, noutput_items %d\n", noutput_items);

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace sidekiq */
} /* namespace gr */
