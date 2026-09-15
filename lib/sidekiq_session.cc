// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Share libsidekiq initialization among this module's RX and TX blocks. A block
 * holds one session for its entire lifetime, including while stopped. The last
 * session shuts down an SDK owned by this module; externally owned SDKs are
 * borrowed. Session ownership does not arbitrate radio configuration changes.
 */

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
// libsidekiq initialization belongs to the application, not to one RX/TX block.
// Each block holds a session member. Count those sessions so destroying the
// first-created block cannot shut down the SDK while another block still uses it.
// The session member also releases ownership if block construction throws.
struct session_state {
    std::mutex mutex;
    size_t users = 0; // Number of live session objects, not streaming handles.
    bool owned = false; // True only when this module successfully called skiq_init.
    // Card IDs are uint8_t. Keep enabled cards until the last session ends.
    std::array<bool, 256> enabled{};
    gr::logger logger{"sidekiq_session"};
};
session_state& state()
{
    // One shared state for all RX/TX blocks in this module.
    static session_state instance;
    return instance;
}
}

sidekiq_session::sidekiq_session(uint8_t card)
{
    auto& shared = state();
    // Serialize session creation and destruction, including SDK lifetime calls.
    // This mutex does not serialize ordinary RX/TX configuration or streaming.
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (!shared.users) {
        const auto status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
        if (status != 0 && status != -EEXIST)
            throw std::runtime_error("Failure: skiq_init, status " + std::to_string(status));
        // -EEXIST means another component initialized the SDK. Borrow it without
        // taking responsibility for enabling its cards or calling skiq_exit.
        shared.owned = status == 0;
        shared.enabled.fill(false);
        shared.enabled[card] = true;
        initialized_card_ = shared.owned;
    } else if (shared.owned && !shared.enabled[card]) {
        // The SDK is already initialized; additional cards use enable_cards,
        // rather than a second skiq_init call.
        const auto status = skiq_enable_cards(&card, 1, skiq_xport_init_level_full);
        if (status != 0)
            throw std::runtime_error("Failure: skiq_enable_cards, status " + std::to_string(status));
        shared.enabled[card] = true;
        initialized_card_ = true;
    }
    // Count only successful acquisitions; failed constructors own no session.
    ++shared.users;
}

sidekiq_session::~sidekiq_session()
{
    auto& shared = state();
    std::lock_guard<std::mutex> lock(shared.mutex);
    // Release the SDK only after the final block releases its session, and only
    // if this module initialized it. Externally owned SDKs are left running.
    if (--shared.users == 0 && shared.owned) {
        const auto status = skiq_exit();
        if (status != 0) shared.logger.error("Failed to release libsidekiq: {}", status);
        shared.owned = false;
    }
}

} // namespace sidekiq
} // namespace gr
