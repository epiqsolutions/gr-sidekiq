// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <pmt/pmt.h>

namespace gr {
namespace sidekiq {
// Shared command protocol for RX and TX; changing these keys changes what
// existing flowgraphs send. Helpers only decode messages, never call the SDK.
namespace detail {
constexpr bool packed_iq = false;
inline const auto command_port = pmt::intern("command");
inline const auto frequency_key = pmt::intern("lo_freq");
inline const auto rate_key = pmt::intern("rate");
inline const auto bandwidth_key = pmt::intern("bandwidth");
inline const auto attenuation_key = pmt::intern("attenuation");
inline const auto gain_key = pmt::intern("gain");

// Dictionaries are also PMT pairs, so test for a dictionary first. Preserve
// other message shapes so the caller can log and ignore them as before.
inline pmt::pmt_t command_dict(pmt::pmt_t message)
{
    if (!pmt::is_dict(message) && pmt::is_pair(message))
        return pmt::dict_add(pmt::make_dict(), pmt::car(message), pmt::cdr(message));
    return message;
}
inline double command_number(pmt::pmt_t message, pmt::pmt_t key)
{
    return pmt::to_double(pmt::dict_ref(message, key, pmt::PMT_NIL));
}
} // namespace detail
} // namespace sidekiq
} // namespace gr
