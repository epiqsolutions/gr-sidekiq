/* -*- c++ -*- */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sidekiq_handle_utils.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace gr {
namespace sidekiq {

namespace {

std::string normalize_handle_string(const std::string& value)
{
    std::string normalized;
    normalized.reserve(value.size());

    for (const char ch : value)
    {
        if (!std::isspace(static_cast<unsigned char>(ch)) && ch != '_')
        {
            normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
        }
    }

    if (normalized.rfind("TX", 0) == 0 || normalized.rfind("RX", 0) == 0)
    {
        normalized.erase(0, 2);
    }

    return normalized;
}

} // namespace

skiq_tx_hdl_t parse_tx_handle(const std::string& value)
{
    const std::string normalized = normalize_handle_string(value);

    if (normalized == "0" || normalized == "A1") {
        return skiq_tx_hdl_A1;
    }
    if (normalized == "1" || normalized == "A2") {
        return skiq_tx_hdl_A2;
    }
    if (normalized == "2" || normalized == "B1") {
        return skiq_tx_hdl_B1;
    }
    if (normalized == "3" || normalized == "B2") {
        return skiq_tx_hdl_B2;
    }

    throw std::invalid_argument("invalid TX handle: " + value);
}

skiq_rx_hdl_t parse_rx_handle(const std::string& value, bool allow_none)
{
    const std::string normalized = normalize_handle_string(value);

    if (allow_none && (normalized == "100" || normalized == "NONE")) {
        return skiq_rx_hdl_end;
    }
    if (normalized == "0" || normalized == "A1") {
        return skiq_rx_hdl_A1;
    }
    if (normalized == "1" || normalized == "A2") {
        return skiq_rx_hdl_A2;
    }
    if (normalized == "2" || normalized == "B1") {
        return skiq_rx_hdl_B1;
    }
    if (normalized == "3" || normalized == "B2") {
        return skiq_rx_hdl_B2;
    }
    if (normalized == "4" || normalized == "C1") {
        return skiq_rx_hdl_C1;
    }
    if (normalized == "5" || normalized == "D1") {
        return skiq_rx_hdl_D1;
    }

    throw std::invalid_argument("invalid RX handle: " + value);
}

int find_tx_param_index(const skiq_param_t& params, skiq_tx_hdl_t handle)
{
    for (int idx = 0; idx < params.rf_param.num_tx_channels; ++idx)
    {
        if (params.rf_param.tx_handles[idx] == handle)
        {
            return idx;
        }
    }

    throw std::runtime_error("requested TX handle is not available in the current topology");
}

int find_rx_param_index(const skiq_param_t& params, skiq_rx_hdl_t handle)
{
    for (int idx = 0; idx < params.rf_param.num_rx_channels; ++idx)
    {
        if (params.rf_param.rx_handles[idx] == handle)
        {
            return idx;
        }
    }

    throw std::runtime_error("requested RX handle is not available in the current topology");
}

} // namespace sidekiq
} // namespace gr
