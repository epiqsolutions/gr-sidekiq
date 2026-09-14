// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <sidekiq_api.h>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Test controls only. This backend is linked exclusively into the QA executable.
namespace fake_sidekiq {
struct call {
    std::string name;
    int handle;
    uint64_t value;
};
struct tx_packet {
    skiq_tx_hdl_t handle;
    uint64_t timestamp;
    std::vector<int16_t> iq;
};
struct rx_packet {
    skiq_rx_hdl_t handle;
    uint64_t timestamp;
    std::vector<int16_t> iq;
};
// Call only when all blocks and scheduler threads from the previous test are gone.
void reset();
std::vector<call> calls();
std::vector<tx_packet> transmitted();
// Queue-full rejection is deterministic; pending packets retain the original
// buffer pointer until complete_one(), just as async callers must retain data.
void set_async_capacity(size_t capacity);
void set_auto_complete(bool enabled);
bool complete_one(int32_t status = 0);
size_t buffer_reuse_count();
size_t pending_count();
// Script is replayed cyclically to keep source work() able to return while a
// downstream Head terminates the graph. Timestamps repeat with the script.
void set_rx_script(const std::vector<rx_packet>& packets);
void fail_next(const std::string& function, int32_t status);
} // namespace fake_sidekiq
