/* -*- c++ -*- */
/*
 * Copyright 2023 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H
#define INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H

#include <gnuradio/sidekiq/telemetry.h>
#include "sidekiq_api.h"
using pmt::pmt_t;

namespace gr {
namespace sidekiq {
    const pmt_t INPUT_MESSAGE_PORT{pmt::string_to_symbol("input")};
    const pmt_t OUTPUT_MESSAGE_PORT{pmt::string_to_symbol("output")};


class telemetry_impl : public telemetry {
public:
  telemetry_impl(
          int input_card,
          int temp_enabled,
          int imu_enabled);

  ~telemetry_impl();

private:
    void temp(const pmt::pmt_t& msg);
    void imu(const pmt::pmt_t& msg);

    uint8_t card{};
    int temp_enabled{};
    int imu_enabled{};

};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_TELEMETRY_IMPL_H */
