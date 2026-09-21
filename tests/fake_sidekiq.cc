// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Hardware-independent implementation of the SDK calls used by the QA blocks.
 * Models selected behaviors and failure paths, not FPGA timing or RF operation.
 * Deferred TX retains caller buffers until completion so ownership bugs remain
 * visible; RX reuses one storage area to exercise the SDK buffer-lifetime rule.
 */

#include "fake_sidekiq.h"
#include <array>
#include <cerrno>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace {
struct radio_rate { uint32_t rate = 1000000, bandwidth = 800000; };
struct pending_packet {
    skiq_tx_hdl_t handle;
    skiq_tx_block_t* block;
    void* user;
    size_t words;
    skiq_tx_callback_t callback;
};
struct state {
    bool initialized = false;
    bool rx_started = false;
    bool auto_complete = true;
    size_t capacity = 20;
    size_t reused_buffers = 0;
    skiq_chan_mode_t channel_mode = skiq_chan_mode_single;
    skiq_tx_callback_t callback = nullptr;
    std::array<bool, skiq_tx_hdl_end> tx_started{};
    std::array<uint16_t, skiq_tx_hdl_end> block_words{};
    std::array<skiq_tx_transfer_mode_t, skiq_tx_hdl_end> transfer{};
    std::array<skiq_tx_flow_mode_t, skiq_tx_hdl_end> flow{};
    std::array<uint32_t, skiq_tx_hdl_end> late_counts{};
    skiq_tx_timestamp_base_t timestamp_base = skiq_tx_rf_timestamp;
    std::array<radio_rate, skiq_tx_hdl_end> tx_rates{};
    std::array<radio_rate, skiq_rx_hdl_end> rx_rates{};
    std::vector<fake_sidekiq::call> calls;
    std::vector<fake_sidekiq::tx_packet> transmitted;
    std::deque<pending_packet> pending;
    std::vector<fake_sidekiq::rx_packet> rx_script;
    size_t rx_index = 0;
    bool rx_repeat = true;
    std::map<skiq_rx_hdl_t, uint32_t> rx_cal_available;
    std::map<std::string, int32_t> failures;
};
state s;
std::mutex mutex;
alignas(4096) std::array<uint8_t, SKIQ_MAX_RX_BLOCK_SIZE_IN_BYTES> rx_storage{};
int32_t record(const char* name, int handle = -1, uint64_t value = 0)
{
    s.calls.push_back({name, handle, value});
    const auto it = s.failures.find(name);
    if (it == s.failures.end()) return 0;
    const auto result = it->second;
    s.failures.erase(it);
    return result;
}
void capture(const pending_packet& p)
{
    s.transmitted.push_back({p.handle, skiq_tx_get_block_timestamp(p.block),
                            {p.block->data, p.block->data + p.words * 2}});
}
} // namespace

namespace fake_sidekiq {
void set_rx_cal_available(skiq_rx_hdl_t handle, uint32_t mask)
{ std::lock_guard<std::mutex> lock(mutex); s.rx_cal_available[handle] = mask; }
void reset() { std::lock_guard<std::mutex> lock(mutex); s = state{}; }
std::vector<call> calls() { std::lock_guard<std::mutex> lock(mutex); return s.calls; }
std::vector<tx_packet> transmitted() { std::lock_guard<std::mutex> lock(mutex); return s.transmitted; }
void set_async_capacity(size_t value) { std::lock_guard<std::mutex> lock(mutex); s.capacity = value; }
void set_auto_complete(bool value) { std::lock_guard<std::mutex> lock(mutex); s.auto_complete = value; }
void set_tx_late_count(skiq_tx_hdl_t handle, uint32_t count)
{ std::lock_guard<std::mutex> lock(mutex); s.late_counts.at(handle) = count; }
size_t pending_count() { std::lock_guard<std::mutex> lock(mutex); return s.pending.size(); }
void fail_next(const std::string& name, int32_t status)
{
    std::lock_guard<std::mutex> lock(mutex);
    s.failures[name] = status;
}
void set_rx_script(const std::vector<rx_packet>& packets, bool repeat)
{
    constexpr size_t shorts_per_packet = 2 * (SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS);
    if (packets.empty()) throw std::invalid_argument("RX script must not be empty");
    for (const auto& p : packets) {
        if (p.iq.size() != shorts_per_packet || p.handle < 0 || p.handle >= skiq_rx_hdl_end)
            throw std::invalid_argument("RX script requires valid handles and full unpacked packets");
    }
    std::lock_guard<std::mutex> lock(mutex);
    s.rx_script = packets;
    s.rx_index = 0;
    s.rx_repeat = repeat;
}
size_t buffer_reuse_count() { std::lock_guard<std::mutex> lock(mutex); return s.reused_buffers; }
bool complete_one(int32_t status)
{
    pending_packet p;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (s.pending.empty()) return false;
        p = s.pending.front();
        s.pending.pop_front();
        if (status == 0) capture(p);
    }
    if (p.callback) p.callback(status, p.block, p.user);
    return true;
}
} // namespace fake_sidekiq

// Only the functions used by the production blocks are supplied. Missing new
// SDK dependencies fail at link time instead of falling through to real hardware.

int32_t skiq_init(skiq_xport_type_t type, skiq_xport_init_level_t level, uint8_t *p_card_nums, uint8_t num_cards)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_init", -1, 0);
    if (status) return status;
    if (num_cards != 1 || !p_card_nums || p_card_nums[0] != 0) return -ENODEV;
    if (s.initialized) return -EEXIST;
    s.initialized = true;
    return 0;
}

int32_t skiq_enable_cards(const uint8_t cards[], uint8_t count, skiq_xport_init_level_t)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_enable_cards", count ? cards[0] : -1);
    if (status) return status;
    if (!s.initialized) return -EPERM;
    return count == 1 && cards[0] < 2 ? 0 : -EINVAL;
}

int32_t skiq_read_parameters(uint8_t card, skiq_param_t* p_param)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_parameters", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_param = {};
    p_param->card_param.part_type = skiq_m2;
    p_param->rf_param.num_tx_channels = 2;
    p_param->rf_param.num_rx_channels = 2;
    for (int i = 0; i < 2; ++i) {
        p_param->rf_param.tx_handles[i] = static_cast<skiq_tx_hdl_t>(i);
        p_param->rf_param.rx_handles[i] = static_cast<skiq_rx_hdl_t>(i);
        p_param->tx_param[i].sample_rate_min = 1000;
        p_param->tx_param[i].sample_rate_max = 61440000;
        p_param->rx_param[i].sample_rate_min = 1000;
        p_param->rx_param[i].sample_rate_max = 61440000;
    }
    return 0;
}

int32_t skiq_exit(void)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_exit", -1, 0);
    if (status) return status;
    s.initialized = false;
    return 0;
}

int32_t skiq_start_rx_streaming_multi_on_trigger(uint8_t card, skiq_rx_hdl_t handles[], uint8_t nr_handles, skiq_trigger_src_t trigger, uint64_t sys_timestamp)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_start_rx_streaming_multi_on_trigger", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.rx_started = true;
    return 0;
}

int32_t skiq_start_tx_streaming(uint8_t card, skiq_tx_hdl_t hdl)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_start_tx_streaming", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    if (!s.block_words.at(hdl)) return -EINVAL;
    s.tx_started.at(hdl) = true;
    return 0;
}

int32_t skiq_stop_rx_streaming_multi_on_trigger(uint8_t card, skiq_rx_hdl_t handles[], uint8_t nr_handles, skiq_trigger_src_t trigger, uint64_t sys_timestamp)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_stop_rx_streaming_multi_on_trigger", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.rx_started = false;
    return 0;
}

int32_t skiq_stop_tx_streaming(uint8_t card, skiq_tx_hdl_t hdl)
{
    std::deque<pending_packet> cancelled;
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto status = record("skiq_stop_tx_streaming", hdl);
        if (status) return status;
        if (card != 0) return -ENODEV;
        s.tx_started.at(hdl) = false;
        s.late_counts.at(hdl) = 0;
        for (auto it = s.pending.begin(); it != s.pending.end();) {
            if (it->handle == hdl) {
                cancelled.push_back(*it);
                it = s.pending.erase(it);
            } else ++it;
        }
    }
    for (const auto& p : cancelled)
        if (p.callback) p.callback(-2, p.block, p.user);
    return 0;
}

int32_t skiq_write_tx_data_flow_mode(uint8_t card, skiq_tx_hdl_t hdl, skiq_tx_flow_mode_t mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_data_flow_mode", hdl, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    if (mode != skiq_tx_immediate_data_flow_mode &&
        mode != skiq_tx_with_timestamps_data_flow_mode) return -ENOTSUP;
    s.flow.at(hdl) = mode;
    return 0;
}

int32_t skiq_write_tx_timestamp_base(uint8_t card, skiq_tx_timestamp_base_t timestamp_base)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_timestamp_base", -1, timestamp_base);
    if (status) return status;
    if (card != 0) return -ENODEV;
    if (timestamp_base != skiq_tx_rf_timestamp) return -ENOTSUP;
    s.timestamp_base = timestamp_base;
    return 0;
}

int32_t skiq_write_tx_transfer_mode(uint8_t card, skiq_tx_hdl_t hdl, skiq_tx_transfer_mode_t transfer_mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_transfer_mode", hdl, transfer_mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.transfer.at(hdl) = transfer_mode;
    return 0;
}

int32_t skiq_register_tx_complete_callback(uint8_t card, skiq_tx_callback_t tx_complete)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_register_tx_complete_callback", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.callback = tx_complete;
    return 0;
}

int32_t skiq_write_chan_mode(uint8_t card, skiq_chan_mode_t mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_chan_mode", -1, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.channel_mode = mode;
    return 0;
}

int32_t skiq_write_rx_LO_freq(uint8_t card, skiq_rx_hdl_t hdl, uint64_t freq)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_LO_freq", hdl, freq);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_rx_sample_rate_and_bandwidth(uint8_t card, skiq_rx_hdl_t hdl, uint32_t rate, uint32_t bandwidth)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_sample_rate_and_bandwidth", hdl, rate);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.rx_rates.at(hdl) = {rate, bandwidth};
    return 0;
}

int32_t skiq_read_rx_sample_rate_and_bandwidth(uint8_t card, skiq_rx_hdl_t hdl, uint32_t *p_rate, double *p_actual_rate, uint32_t *p_bandwidth, uint32_t *p_actual_bandwidth)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_rx_sample_rate_and_bandwidth", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    const auto rate = s.rx_rates.at(hdl);
    *p_rate = rate.rate; *p_actual_rate = rate.rate;
    *p_bandwidth = rate.bandwidth; *p_actual_bandwidth = rate.bandwidth;
    return 0;
}

int32_t skiq_write_tx_sample_rate_and_bandwidth(uint8_t card, skiq_tx_hdl_t hdl, uint32_t rate, uint32_t bandwidth)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_sample_rate_and_bandwidth", hdl, rate);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.tx_rates.at(hdl) = {rate, bandwidth};
    return 0;
}

int32_t skiq_read_tx_sample_rate_and_bandwidth(uint8_t card, skiq_tx_hdl_t hdl, uint32_t *p_rate, double *p_actual_rate, uint32_t *p_bandwidth, uint32_t *p_actual_bandwidth)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_tx_sample_rate_and_bandwidth", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    const auto rate = s.tx_rates.at(hdl);
    *p_rate = rate.rate; *p_actual_rate = rate.rate;
    *p_bandwidth = rate.bandwidth; *p_actual_bandwidth = rate.bandwidth;
    return 0;
}

int32_t skiq_read_rx_gain_index_range(uint8_t card, skiq_rx_hdl_t hdl, uint8_t* p_gain_index_min, uint8_t* p_gain_index_max)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_rx_gain_index_range", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_gain_index_min = 0; *p_gain_index_max = 76;
    return 0;
}

int32_t skiq_write_rx_gain(uint8_t card, skiq_rx_hdl_t hdl, uint8_t gain_index)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_gain", hdl, gain_index);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_rx_gain_mode(uint8_t card, skiq_rx_hdl_t hdl, skiq_rx_gain_t gain_mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_gain_mode", hdl, gain_mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_tx_LO_freq(uint8_t card, skiq_tx_hdl_t hdl, uint64_t freq)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_LO_freq", hdl, freq);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_tx_attenuation(uint8_t card, skiq_tx_hdl_t hdl, uint16_t attenuation)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_attenuation", hdl, attenuation);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_tx_block_size(uint8_t card, skiq_tx_hdl_t hdl, uint16_t block_size_in_words)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_block_size", hdl, block_size_in_words);
    if (status) return status;
    if (card != 0) return -ENODEV;
    s.block_words.at(hdl) = block_size_in_words;
    return 0;
}

int32_t skiq_read_tx_num_underruns(uint8_t card, skiq_tx_hdl_t hdl, uint32_t *p_num_underrun)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_tx_num_underruns", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_num_underrun = 0;
    return 0;
}

int32_t skiq_read_tx_num_late_timestamps(uint8_t card, skiq_tx_hdl_t hdl,
                                         uint32_t *p_num_late)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_tx_num_late_timestamps", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_num_late = s.late_counts.at(hdl);
    return 0;
}

int32_t skiq_write_iq_pack_mode(uint8_t card, bool mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_iq_pack_mode", -1, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    if (mode) return -ENOTSUP;
    return 0;
}

int32_t skiq_write_iq_order_mode(uint8_t card, skiq_iq_order_t mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_iq_order_mode", -1, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    if (mode != skiq_iq_order_iq) return -ENOTSUP;
    return 0;
}

int32_t skiq_write_rx_data_src(uint8_t card, skiq_rx_hdl_t hdl, skiq_data_src_t src)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_data_src", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_reset_timestamps(uint8_t card)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_reset_timestamps", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_num_tx_threads(uint8_t card, uint8_t num_threads)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_num_tx_threads", -1, num_threads);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_read_rx_iq_resolution(uint8_t card, uint8_t *p_adc_res)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_rx_iq_resolution", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_adc_res = 12;
    return 0;
}

int32_t skiq_read_tx_iq_resolution(uint8_t card, uint8_t *p_dac_res)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_tx_iq_resolution", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    *p_dac_res = 12;
    return 0;
}

int32_t skiq_write_tx_quadcal_mode(uint8_t card, skiq_tx_hdl_t hdl, skiq_tx_quadcal_mode_t mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_tx_quadcal_mode", hdl, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_run_tx_quadcal(uint8_t card, skiq_tx_hdl_t hdl)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_run_tx_quadcal", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_rx_cal_mode(uint8_t card, skiq_rx_hdl_t hdl, skiq_rx_cal_mode_t mode)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_cal_mode", hdl, mode);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_run_rx_cal(uint8_t card, skiq_rx_hdl_t hdl)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_run_rx_cal", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_write_rx_cal_type_mask(uint8_t card, skiq_rx_hdl_t hdl, uint32_t cal_mask)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_rx_cal_type_mask", hdl, cal_mask);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

int32_t skiq_read_rx_cal_types_avail(uint8_t card, skiq_rx_hdl_t hdl, uint32_t *p_cal_mask)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_read_rx_cal_types_avail", hdl, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    const auto it = s.rx_cal_available.find(hdl);
    *p_cal_mask = it == s.rx_cal_available.end() ?
        skiq_rx_cal_type_dc_offset | skiq_rx_cal_type_quadrature : it->second;
    return 0;
}

int32_t skiq_write_1pps_source(uint8_t card, skiq_1pps_source_t pps_source)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_write_1pps_source", -1, pps_source);
    if (status) return status;
    if (card != 0) return -ENODEV;
    // Configuration call recorded for assertions.
    return 0;
}

bool skiq_is_topology_supported(uint8_t card)
{
    std::lock_guard<std::mutex> lock(mutex);
    record("skiq_is_topology_supported");
    return false;
}

int32_t skiq_apply_topology(uint8_t card, uint8_t topology_id)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto status = record("skiq_apply_topology", -1, 0);
    if (status) return status;
    if (card != 0) return -ENODEV;
    return -ENOTSUP;
}

skiq_rx_status_t skiq_receive(uint8_t card, skiq_rx_hdl_t* handle,
                              skiq_rx_block_t** block, uint32_t* bytes)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (card != 0 || !s.rx_started || s.rx_script.empty()) return skiq_rx_status_no_data;
    if (!s.rx_repeat && s.rx_index >= s.rx_script.size()) return skiq_rx_status_no_data;
    const auto& p = s.rx_script[s.rx_index++ % s.rx_script.size()];
    auto* result = reinterpret_cast<skiq_rx_block_t*>(rx_storage.data());
    result->rf_timestamp = p.timestamp;
    std::memcpy(rx_storage.data() + SKIQ_RX_HEADER_SIZE_IN_BYTES,
                p.iq.data(), p.iq.size() * sizeof(int16_t));
    *handle = p.handle;
    *block = result;
    *bytes = SKIQ_RX_HEADER_SIZE_IN_BYTES + p.iq.size() * sizeof(int16_t);
    return skiq_rx_status_success;
}

int32_t skiq_transmit(uint8_t card, skiq_tx_hdl_t hdl, skiq_tx_block_t* block, void* user)
{
    pending_packet p;
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto status = record("skiq_transmit", hdl);
        if (status) return status;
        if (card != 0 || !s.tx_started.at(hdl)) return -EINVAL;
        const bool async = s.transfer.at(hdl) == skiq_tx_transfer_mode_async;
        if (async && s.pending.size() >= s.capacity) return SKIQ_TX_ASYNC_SEND_QUEUE_FULL;
        const size_t words = s.block_words.at(hdl) * (s.channel_mode == skiq_chan_mode_dual ? 2 : 1);
        p = {hdl, block, user, words, async ? s.callback : nullptr};
        if (async && !s.auto_complete) {
            for (const auto& queued : s.pending)
                if (queued.block == block) ++s.reused_buffers;
            s.pending.push_back(p);
            return 0;
        }
        capture(p);
    }
    // Never invoke foreign callbacks while holding the fake backend's mutex.
    if (p.callback) p.callback(0, block, user);
    return 0;
}
