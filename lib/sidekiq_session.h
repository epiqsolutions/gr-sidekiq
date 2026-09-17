// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace gr {
namespace sidekiq {

// One lease per block. The SDK remains initialized until the last lease ends.
// A library initialized by another component is borrowed, never shut down here.
class sidekiq_session
{
public:
    explicit sidekiq_session(uint8_t card);
    ~sidekiq_session();
    sidekiq_session(const sidekiq_session&) = delete;
    sidekiq_session& operator=(const sidekiq_session&) = delete;
    // True if this acquisition initialized/enabled the card. TX uses this to
    // preserve its policy of leaving rate/bandwidth unchanged on a shared card.
    bool initialized_card() const { return initialized_card_; }

private:
    bool initialized_card_ = false;
};

} // namespace sidekiq
} // namespace gr
