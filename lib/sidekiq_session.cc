// SPDX-License-Identifier: GPL-3.0-or-later
#include "sidekiq_session.h"
#include <sidekiq_api.h>
#include <gnuradio/logger.h>
#include <array>
#include <cerrno>
#include <mutex>
#include <stdexcept>
#include <string>

namespace gr {
namespace sidekiq {
namespace {
struct session_state {
    std::mutex mutex;
    size_t users = 0;
    bool owned = false;
    std::array<bool, 256> enabled{};
    gr::logger logger{"sidekiq_session"};
};
session_state& state()
{
    static session_state instance;
    return instance;
}
}

sidekiq_session::sidekiq_session(uint8_t card)
{
    auto& shared = state();
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (!shared.users) {
        const auto status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
        if (status != 0 && status != -EEXIST)
            throw std::runtime_error("Failure: skiq_init, status " + std::to_string(status));
        shared.owned = status == 0;
        shared.enabled.fill(false);
        shared.enabled[card] = true;
        initialized_card_ = shared.owned;
    } else if (shared.owned && !shared.enabled[card]) {
        const auto status = skiq_enable_cards(&card, 1, skiq_xport_init_level_full);
        if (status != 0)
            throw std::runtime_error("Failure: skiq_enable_cards, status " + std::to_string(status));
        shared.enabled[card] = true;
        initialized_card_ = true;
    }
    ++shared.users;
}

sidekiq_session::~sidekiq_session()
{
    auto& shared = state();
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (--shared.users == 0 && shared.owned) {
        const auto status = skiq_exit();
        if (status != 0) shared.logger.error("Failed to release libsidekiq: {}", status);
        shared.owned = false;
    }
}

} // namespace sidekiq
} // namespace gr
