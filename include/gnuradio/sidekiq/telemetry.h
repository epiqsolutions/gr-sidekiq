/* -*- c++ -*- */
/*
 * Copyright 2023 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_TELEMETRY_H
#define INCLUDED_SIDEKIQ_TELEMETRY_H

#include <gnuradio/sidekiq/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace sidekiq {

/*!
 * \brief <+description of block+>
 * \ingroup sidekiq
 *
 */
class SIDEKIQ_API telemetry : virtual public gr::block {
public:
  typedef std::shared_ptr<telemetry> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of sidekiq::telemetry.
   *
   * To avoid accidental use of raw pointers, sidekiq::telemetry's
   * constructor is in a private implementation
   * class. sidekiq::telemetry::make is the public interface for
   * creating new instances.
   */
  static sptr make(
          int input_card,
          int temp_enabled,
          int imu_enabled);


};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_TELEMETRY_H */
