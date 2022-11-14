/* -*- c++ -*- */
/*
 * Copyright 2022 epiq.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_RX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_RX_H

#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sidekiq {

/*!
 * \brief <+description of block+>
 * \ingroup sidekiq
 *
 */
class SIDEKIQ_API sidekiq_rx : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<sidekiq_rx> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of sidekiq::sidekiq_rx.
   *
   * To avoid accidental use of raw pointers, sidekiq::sidekiq_rx's
   * constructor is in a private implementation
   * class. sidekiq::sidekiq_rx::make is the public interface for
   * creating new instances.
   */
  static sptr make();
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_RX_H */
