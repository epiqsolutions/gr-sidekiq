/* -*- c++ -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_HANDLE_UTILS_H
#define INCLUDED_SIDEKIQ_HANDLE_UTILS_H

#include <sidekiq_api.h>

#include <string>

namespace gr {
namespace sidekiq {

skiq_tx_hdl_t parse_tx_handle(const std::string& value);
skiq_rx_hdl_t parse_rx_handle(const std::string& value, bool allow_none = false);

int find_tx_param_index(const skiq_param_t& params, skiq_tx_hdl_t handle);
int find_rx_param_index(const skiq_param_t& params, skiq_rx_hdl_t handle);

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_HANDLE_UTILS_H */
