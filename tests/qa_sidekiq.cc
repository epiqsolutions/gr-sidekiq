// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Regression tests for the actual GNU Radio blocks linked to the fake SDK.
 * Flowgraphs check sample/tag behavior; focused tests check lifecycle and error
 * paths. Tests run sequentially with fresh fake state and require no radio.
 * Passing results do not establish hardware throughput, timing, or RF quality.
 */

#include "fake_sidekiq.h"
#include "sidekiq_rx_impl.h"
#include "sidekiq_tx_impl.h"
#include "tx_buffer_pool.h"
#include "sidekiq_handle_utils.h"
#include <gnuradio/sidekiq/sidekiq_rx.h>
#include <gnuradio/sidekiq/sidekiq_tx.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>
#include <sstream>
#include <spdlog/sinks/ostream_sink.h>
#include <cerrno>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace {
constexpr int tx_samples = 4092;
constexpr int rx_samples = SKIQ_MAX_RX_BLOCK_SIZE_IN_WORDS - SKIQ_RX_HEADER_SIZE_IN_WORDS;
struct fixture { fixture() { fake_sidekiq::reset(); } };
auto make_tx(int threads = 1)
{
    return gr::sidekiq::sidekiq_tx::make(0, "A1", 1e6, 800e3, 915e6, 100, "", threads, tx_samples, 1);
}
size_t count_calls(const std::string& name)
{
    const auto calls = fake_sidekiq::calls();
    return std::count_if(calls.begin(), calls.end(), [&](const auto& c) { return c.name == name; });
}
void run_tx_flowgraph(int threads)
{
    std::vector<gr_complex> samples(4 * tx_samples);
    for (size_t i = 0; i < samples.size(); ++i)
        samples[i] = gr_complex((int(i % 17) - 8) / 16.0f, (int(i % 11) - 5) / 16.0f);
    auto graph = gr::make_top_block("qa_tx");
    auto source = gr::blocks::vector_source_c::make(samples, false);
    auto sink = make_tx(threads);
    graph->connect(source, 0, sink, 0);
    graph->run(tx_samples);
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), 4);
    size_t offset = 0;
    for (const auto& p : packets) {
        BOOST_REQUIRE_EQUAL(p.iq.size(), 2 * tx_samples);
        BOOST_CHECK_EQUAL(p.handle, skiq_tx_hdl_A1);
        for (int i = 0; i < tx_samples; ++i, ++offset) {
            BOOST_CHECK_LE(std::abs(p.iq[2 * i] - samples[offset].real() * 2047.0f), 1.0f);
            BOOST_CHECK_LE(std::abs(p.iq[2 * i + 1] - samples[offset].imag() * 2047.0f), 1.0f);
        }
    }
    BOOST_CHECK_EQUAL(count_calls("skiq_start_tx_streaming"), 1);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 1);
    const auto calls = fake_sidekiq::calls();
    BOOST_CHECK(std::any_of(calls.begin(), calls.end(), [](const auto& c) {
        return c.name == "skiq_write_tx_data_flow_mode" && c.value == skiq_tx_immediate_data_flow_mode;
    }));
}
} // namespace

BOOST_FIXTURE_TEST_SUITE(baseline, fixture)
BOOST_AUTO_TEST_CASE(tx_immediate_sync) { run_tx_flowgraph(1); }
// Immediate callbacks are only the baseline. Deferred ownership regressions
// are covered separately in tx_safety.
BOOST_AUTO_TEST_CASE(tx_immediate_async) { run_tx_flowgraph(2); }

BOOST_AUTO_TEST_CASE(rx_single_channel_samples)
{
    std::vector<int16_t> iq(2 * rx_samples);
    for (int i = 0; i < rx_samples; ++i) {
        iq[2 * i] = (i % 100) - 50;
        iq[2 * i + 1] = 200 - (i % 200);
    }
    fake_sidekiq::set_rx_script({{skiq_rx_hdl_A1, 10000, iq}});
    auto graph = gr::make_top_block("qa_rx");
    auto source = gr::sidekiq::sidekiq_rx::make(
        0, "A1", "none", 1e6, 800e3, 915e6, 0, 10, 0, 0, 0, 2, 0);
    auto head = gr::blocks::head::make(sizeof(gr_complex), 2 * rx_samples);
    auto sink = gr::blocks::vector_sink_c::make();
    graph->connect(source, 0, head, 0);
    graph->connect(head, 0, sink, 0);
    graph->run(rx_samples);
    const auto data = sink->data();
    BOOST_REQUIRE_EQUAL(data.size(), 2 * rx_samples);
    for (size_t i = 0; i < data.size(); ++i) {
        BOOST_CHECK_SMALL(data[i].real() - iq[2 * (i % rx_samples)] / 2047.0f, 1e-6f);
        BOOST_CHECK_SMALL(data[i].imag() - iq[2 * (i % rx_samples) + 1] / 2047.0f, 1e-6f);
    }
    BOOST_CHECK_EQUAL(count_calls("skiq_start_rx_streaming_multi_on_trigger"), 1);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_rx_streaming_multi_on_trigger"), 1);
}

BOOST_AUTO_TEST_CASE(tx_setters_forward_values)
{
    auto sink = make_tx();
    sink->set_tx_frequency(920e6);
    sink->set_tx_attenuation(120);
    sink->set_tx_sample_rate(2e6);
    sink->set_tx_bandwidth(1.5e6);
    sink->run_tx_cal(1);
    const auto calls = fake_sidekiq::calls();
    for (const auto& expected : std::vector<fake_sidekiq::call>{
             {"skiq_write_tx_LO_freq", 0, 920000000},
             {"skiq_write_tx_attenuation", 0, 120},
             {"skiq_write_tx_sample_rate_and_bandwidth", 0, 2000000},
             {"skiq_run_tx_quadcal", 0, 0}}) {
        BOOST_CHECK(std::any_of(calls.begin(), calls.end(), [&](const auto& c) {
            return c.name == expected.name && c.handle == expected.handle && c.value == expected.value;
        }));
    }
    uint32_t requested, bw, actual_bw;
    double actual;
    skiq_read_tx_sample_rate_and_bandwidth(0, skiq_tx_hdl_A1, &requested, &actual, &bw, &actual_bw);
    BOOST_CHECK_EQUAL(actual, 2000000);
    BOOST_CHECK_EQUAL(actual_bw, 1500000);
}

BOOST_AUTO_TEST_CASE(sdk_failure_is_reported)
{
    auto sink = make_tx();
    fake_sidekiq::fail_next("skiq_write_tx_LO_freq", -EIO);
    BOOST_CHECK_THROW(sink->set_tx_frequency(920e6), std::runtime_error);
    BOOST_CHECK_NO_THROW(sink->set_tx_frequency(920e6));
}

BOOST_AUTO_TEST_CASE(handle_names_and_parameter_order)
{
    using namespace gr::sidekiq;
    BOOST_CHECK_EQUAL(parse_tx_handle(" tx_a1 "), skiq_tx_hdl_A1);
    BOOST_CHECK_EQUAL(parse_rx_handle("RxB1"), skiq_rx_hdl_B1);
    BOOST_CHECK_EQUAL(parse_rx_handle("none", true), skiq_rx_hdl_end);
    BOOST_CHECK_THROW(parse_rx_handle("none"), std::invalid_argument);
    BOOST_CHECK_THROW(parse_tx_handle("bad"), std::invalid_argument);
    skiq_param_t params{};
    params.rf_param.num_tx_channels = 2;
    params.rf_param.tx_handles[0] = skiq_tx_hdl_B1;
    params.rf_param.tx_handles[1] = skiq_tx_hdl_A1;
    BOOST_CHECK_EQUAL(find_tx_param_index(params, skiq_tx_hdl_A1), 1);
    BOOST_CHECK_THROW(find_tx_param_index(params, skiq_tx_hdl_A2), std::runtime_error);
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_FIXTURE_TEST_SUITE(backend, fixture)
BOOST_AUTO_TEST_CASE(deferred_completion_and_queue_full)
{
    uint8_t card = 0;
    BOOST_REQUIRE_EQUAL(skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1), 0);
    skiq_write_tx_block_size(0, skiq_tx_hdl_A1, 252);
    skiq_write_tx_transfer_mode(0, skiq_tx_hdl_A1, skiq_tx_transfer_mode_async);
    skiq_register_tx_complete_callback(0, [](int32_t status, skiq_tx_block_t*, void* user) {
        *static_cast<int*>(user) = status == 0 ? 1 : -1;
    });
    skiq_start_tx_streaming(0, skiq_tx_hdl_A1);
    fake_sidekiq::set_auto_complete(false);
    fake_sidekiq::set_async_capacity(1);
    std::unique_ptr<skiq_tx_block_t, decltype(&skiq_tx_block_free)> block(
        skiq_tx_block_allocate(252), &skiq_tx_block_free);
    BOOST_REQUIRE(block);
    block->data[0] = 123;
    skiq_tx_set_block_timestamp(block.get(), 456);
    int completed = 0;
    BOOST_CHECK_EQUAL(skiq_transmit(0, skiq_tx_hdl_A1, block.get(), &completed), 0);
    BOOST_CHECK_EQUAL(completed, 0);
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 1);
    BOOST_CHECK(fake_sidekiq::transmitted().empty());
    BOOST_CHECK_EQUAL(skiq_transmit(0, skiq_tx_hdl_A1, block.get(), &completed), SKIQ_TX_ASYNC_SEND_QUEUE_FULL);
    // Deliberately violate caller ownership to prove the fake reads the original
    // buffer on completion rather than hiding reuse bugs with an early copy.
    block->data[0] = 124;
    BOOST_CHECK(fake_sidekiq::complete_one());
    BOOST_CHECK_EQUAL(completed, 1);
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 0);
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), 1);
    BOOST_CHECK_EQUAL(packets[0].iq[0], 124);
    BOOST_CHECK_EQUAL(packets[0].timestamp, 456);
    BOOST_CHECK(!fake_sidekiq::complete_one());
    BOOST_CHECK_EQUAL(skiq_stop_tx_streaming(0, skiq_tx_hdl_A1), 0);
    skiq_exit();
}
BOOST_AUTO_TEST_CASE(stop_failure_preserves_state_and_cancellation_is_per_handle)
{
    uint8_t card = 0;
    BOOST_REQUIRE_EQUAL(skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1), 0);
    for (auto handle : {skiq_tx_hdl_A1, skiq_tx_hdl_A2}) {
        BOOST_REQUIRE_EQUAL(skiq_write_tx_block_size(card, handle, 252), 0);
        BOOST_REQUIRE_EQUAL(skiq_write_tx_transfer_mode(card, handle, skiq_tx_transfer_mode_async), 0);
        BOOST_REQUIRE_EQUAL(skiq_start_tx_streaming(card, handle), 0);
    }
    fake_sidekiq::set_auto_complete(false);
    std::unique_ptr<skiq_tx_block_t, decltype(&skiq_tx_block_free)> block(
        skiq_tx_block_allocate(252), &skiq_tx_block_free);
    BOOST_REQUIRE(block);
    BOOST_REQUIRE_EQUAL(skiq_transmit(card, skiq_tx_hdl_A1, block.get(), nullptr), 0);
    // A2 can stop even though A1 still owns a pending transfer.
    BOOST_CHECK_EQUAL(skiq_stop_tx_streaming(card, skiq_tx_hdl_A2), 0);
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 1);
    BOOST_CHECK_EQUAL(skiq_transmit(card, skiq_tx_hdl_A2, block.get(), nullptr), -EINVAL);
    // This branch cancels transfers on successful stop. Inject a failure to
    // verify that an unsuccessful stop still preserves streaming state.
    fake_sidekiq::fail_next("skiq_stop_tx_streaming", -EIO);
    BOOST_CHECK_EQUAL(skiq_stop_tx_streaming(card, skiq_tx_hdl_A1), -EIO);
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 1);
    BOOST_CHECK(fake_sidekiq::complete_one());
    BOOST_CHECK_EQUAL(skiq_transmit(card, skiq_tx_hdl_A1, block.get(), nullptr), 0);
    BOOST_CHECK(fake_sidekiq::complete_one());
    BOOST_CHECK_EQUAL(skiq_transmit(card, skiq_tx_hdl_A1, block.get(), nullptr), 0);
    BOOST_CHECK_EQUAL(skiq_stop_tx_streaming(card, skiq_tx_hdl_A1), 0);
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 0);
    BOOST_CHECK_EQUAL(fake_sidekiq::transmitted().size(), 2);
    BOOST_CHECK_EQUAL(skiq_exit(), 0);
}

BOOST_AUTO_TEST_CASE(rx_script_validates_and_preserves_metadata)
{
    BOOST_CHECK_THROW(fake_sidekiq::set_rx_script({}), std::invalid_argument);
    BOOST_CHECK_THROW(fake_sidekiq::set_rx_script({{skiq_rx_hdl_A1, 0, {1, 2}}}), std::invalid_argument);
    std::vector<int16_t> iq(2 * rx_samples, 321);
    fake_sidekiq::set_rx_script({{skiq_rx_hdl_A1, 100, iq}, {skiq_rx_hdl_A2, 900, iq}});
    skiq_rx_hdl_t handles[] = {skiq_rx_hdl_A1, skiq_rx_hdl_A2};
    skiq_start_rx_streaming_multi_on_trigger(0, handles, 2, skiq_trigger_src_immediate, 0);
    for (const auto expected : {100, 900, 100}) {
        skiq_rx_hdl_t handle;
        skiq_rx_block_t* block;
        uint32_t bytes;
        BOOST_REQUIRE_EQUAL(skiq_receive(0, &handle, &block, &bytes), skiq_rx_status_success);
        BOOST_CHECK_EQUAL(uint64_t(block->rf_timestamp), expected);
        BOOST_CHECK_EQUAL(handle, expected == 900 ? skiq_rx_hdl_A2 : skiq_rx_hdl_A1);
        BOOST_CHECK_EQUAL(bytes, SKIQ_MAX_RX_BLOCK_SIZE_IN_BYTES);
        BOOST_CHECK_EQUAL(int16_t(block->data[0]), 321);
    }
}
BOOST_AUTO_TEST_SUITE_END()

namespace {
bool wait_until(const std::function<bool()>& predicate)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}
}

BOOST_FIXTURE_TEST_SUITE(tx_safety, fixture)
BOOST_AUTO_TEST_CASE(secondary_channel_payload)
{
    // A2 selects a dual-channel packet with contiguous A1 and A2 payloads.
    constexpr size_t per_channel = tx_samples + 2;
    std::vector<gr_complex> samples(per_channel, {0.5f, -0.25f});
    auto graph = gr::make_top_block("qa_tx_a2");
    auto source = gr::blocks::vector_source_c::make(samples, false);
    auto sink = gr::sidekiq::sidekiq_tx::make(
        0, "A2", 1e6, 800e3, 915e6, 100, "", 1, tx_samples, 1);
    graph->connect(source, 0, sink, 0);
    graph->run(per_channel);
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), 1);
    BOOST_REQUIRE_EQUAL(packets[0].iq.size(), 4 * per_channel);
    for (size_t i = 0; i < 2 * per_channel; ++i) BOOST_CHECK_EQUAL(packets[0].iq[i], 0);
    for (size_t i = 0; i < per_channel; ++i) {
        BOOST_CHECK_LE(std::abs(packets[0].iq[2 * per_channel + 2 * i] - 1023.5f), 1.0f);
        BOOST_CHECK_LE(std::abs(packets[0].iq[2 * per_channel + 2 * i + 1] + 511.75f), 1.0f);
    }
}

BOOST_AUTO_TEST_CASE(queue_full_without_a_pending_callback)
{
    // A shared SDK queue may be full without an outstanding packet owned by
    // this sink. Waiting only for our own completion would deadlock.
    fake_sidekiq::fail_next("skiq_transmit", SKIQ_TX_ASYNC_SEND_QUEUE_FULL);
    run_tx_flowgraph(2);
    BOOST_CHECK_EQUAL(count_calls("skiq_transmit"), 5);
}

BOOST_AUTO_TEST_CASE(deferred_buffers_are_not_reused)
{
    fake_sidekiq::set_auto_complete(false);
    fake_sidekiq::set_async_capacity(64);
    std::vector<gr_complex> samples(24 * tx_samples);
    for (size_t i = 0; i < samples.size(); ++i) samples[i] = {float(i / tx_samples) / 32, 0};
    auto graph = gr::make_top_block("qa_deferred");
    auto source = gr::blocks::vector_source_c::make(samples, false);
    auto sink = make_tx(2);
    graph->connect(source, 0, sink, 0);
    graph->start(tx_samples);
    BOOST_CHECK(wait_until([] { return count_calls("skiq_transmit") >= 20; }));
    // Give a broken implementation the opportunity to submit a 21st packet;
    // the fixed implementation must be blocked on its 20 occupied buffers.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (count_calls("skiq_transmit") == 20 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    BOOST_CHECK_EQUAL(fake_sidekiq::buffer_reuse_count(), 0);
    BOOST_CHECK_EQUAL(count_calls("skiq_transmit"), 20);
    fake_sidekiq::set_auto_complete(true);
    while (fake_sidekiq::complete_one()) {}
    graph->wait();
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), 24);
    // Transfers completing inline may interleave with the manually completed
    // ones; payload identity must still be preserved exactly once.
    std::vector<int> identities;
    for (const auto& packet : packets) {
        BOOST_REQUIRE_EQUAL(packet.iq.size(), 2 * tx_samples);
        const int id = std::lround(packet.iq[0] * 32.0 / 2047);
        identities.push_back(id);
        for (int i = 0; i < tx_samples; ++i)
            BOOST_CHECK_LE(std::abs(packet.iq[2 * i] - id * 2047.0f / 32), 1.0f);
    }
    std::sort(identities.begin(), identities.end());
    for (int i = 0; i < 24; ++i) BOOST_CHECK_EQUAL(identities[i], i);
}

BOOST_AUTO_TEST_CASE(stop_interrupts_a_full_pool)
{
    fake_sidekiq::set_auto_complete(false);
    fake_sidekiq::set_async_capacity(64);
    auto graph = gr::make_top_block("qa_stop");
    auto source = gr::blocks::vector_source_c::make(std::vector<gr_complex>(tx_samples, {0.25f, 0}), true);
    auto sink = make_tx(2);
    graph->connect(source, 0, sink, 0);
    graph->start(tx_samples);
    BOOST_CHECK(wait_until([] { return fake_sidekiq::pending_count() >= 20; }));
    graph->stop();
    graph->wait();
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 0);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 1);
}
BOOST_AUTO_TEST_CASE(callback_storage_survives_owner)
{
    auto pool = std::make_shared<gr::sidekiq::tx_buffer_pool>(1, 32);
    std::weak_ptr<gr::sidekiq::tx_buffer_pool> lifetime = pool;
    void* context;
    skiq_tx_block_t* block;
    {
        auto buffer = pool->acquire(0);
        context = buffer.context();
        block = buffer.block();
        buffer.handoff();
    }
    pool->cancel();
    pool.reset();
    BOOST_CHECK(!lifetime.expired());
    gr::sidekiq::tx_buffer_pool::complete(-2, block, context);
    BOOST_CHECK(lifetime.expired());
}

BOOST_AUTO_TEST_CASE(completion_failure_and_restart)
{
    auto pool = std::make_shared<gr::sidekiq::tx_buffer_pool>(1, 32);
    {
        auto buffer = pool->acquire(0);
        BOOST_CHECK_THROW(pool->restart(), std::runtime_error);
        buffer.handoff();
        gr::sidekiq::tx_buffer_pool::complete(-EIO, buffer.block(), buffer.context());
    }
    BOOST_CHECK_EQUAL(pool->error(), -EIO);
    BOOST_CHECK_THROW(pool->acquire(0), std::runtime_error);
    pool->cancel();
    pool->restart();
    auto buffer = pool->acquire(0);
    BOOST_CHECK(bool(buffer));
    BOOST_CHECK_EQUAL(pool->error(), 0);
}

BOOST_AUTO_TEST_SUITE_END()

namespace {
void run_burst_completion_test(int threads, bool abort_burst)
{
    auto graph = gr::make_top_block("qa_burst_completion");
    gr::tag_t tag;
    tag.offset = 0;
    tag.key = pmt::intern("burst");
    tag.value = pmt::from_uint64(2 * tx_samples);
    auto source = gr::blocks::vector_source_c::make(
        std::vector<gr_complex>(3 * tx_samples, {0.25f, -0.25f}), false, 1,
        std::vector<gr::tag_t>{tag});
    auto sink = gr::sidekiq::sidekiq_tx::make(
        0, "A1", 1e6, 800e3, 915e6, 100, "burst", threads, tx_samples, 1);
    graph->connect(source, 0, sink, 0);
    if (threads > 1) fake_sidekiq::set_auto_complete(false);
    graph->start(tx_samples);
    if (threads > 1) {
        BOOST_CHECK(wait_until([] { return count_calls("skiq_transmit") == 2; }));
        // Give premature stop a chance to run before inspecting the queue.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 0);
        BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 2);
        if (abort_burst) {
            graph->stop();
        } else {
            BOOST_CHECK(fake_sidekiq::complete_one());
            BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 0);
            BOOST_CHECK(fake_sidekiq::complete_one());
        }
    }
    graph->wait();
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 0);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 1);
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), abort_burst ? 0 : 2);
    for (const auto& packet : packets)
        for (int i = 0; i < tx_samples; ++i) {
            BOOST_CHECK_LE(std::abs(packet.iq[2 * i] - 511.75f), 1.0f);
            BOOST_CHECK_LE(std::abs(packet.iq[2 * i + 1] + 511.75f), 1.0f);
        }
}
}
BOOST_FIXTURE_TEST_SUITE(burst_completion, fixture)
BOOST_AUTO_TEST_CASE(async_waits_for_all_callbacks) { run_burst_completion_test(2, false); }
BOOST_AUTO_TEST_CASE(sync_releases_reservation_before_drain) { run_burst_completion_test(1, false); }
BOOST_AUTO_TEST_CASE(explicit_stop_interrupts_drain) { run_burst_completion_test(2, true); }
BOOST_AUTO_TEST_SUITE_END()

namespace {
auto make_burst_tx(int threads = 1)
{
    return gr::sidekiq::sidekiq_tx::make(0, "A1", 1e6, 800e3, 915e6,
                                      100, "burst", threads, tx_samples, 1);
}
std::vector<gr::tag_t> burst_tags(const std::vector<std::pair<uint64_t, uint64_t>>& bursts)
{
    std::vector<gr::tag_t> tags;
    for (const auto& b : bursts) {
        gr::tag_t tag;
        tag.offset = b.first;
        tag.key = pmt::intern("burst");
        tag.value = pmt::from_uint64(b.second);
        tags.push_back(tag);
    }
    return tags;
}
void check_bursts(const std::vector<std::pair<uint64_t, uint64_t>>& bursts,
                  size_t total, int chunk, int threads = 1)
{
    std::vector<gr_complex> samples(total);
    for (size_t i = 0; i < total; ++i)
        samples[i] = {float(int(i % 31) - 15) / 32, float(int(i % 19) - 9) / 32};
    auto graph = gr::make_top_block("qa_bursts");
    auto source = gr::blocks::vector_source_c::make(samples, false, 1, burst_tags(bursts));
    auto sink = make_burst_tx(threads);
    graph->connect(source, 0, sink, 0);
    graph->run(chunk);
    const auto packets = fake_sidekiq::transmitted();
    size_t packet = 0;
    for (const auto& burst : bursts) {
        for (uint64_t sent = 0; sent < burst.second; sent += tx_samples) {
            BOOST_REQUIRE_LT(packet, packets.size());
            const auto& iq = packets[packet++].iq;
            BOOST_REQUIRE_EQUAL(iq.size(), 2 * tx_samples);
            const auto valid = std::min<uint64_t>(tx_samples, burst.second - sent);
            for (size_t i = 0; i < tx_samples; ++i) {
                const auto expected = i < valid ? samples[burst.first + sent + i] : gr_complex{};
                BOOST_CHECK_LE(std::abs(iq[2 * i] - expected.real() * 2047), 1.0f);
                BOOST_CHECK_LE(std::abs(iq[2 * i + 1] - expected.imag() * 2047), 1.0f);
            }
        }
    }
    BOOST_CHECK_EQUAL(packet, packets.size());
    BOOST_CHECK_EQUAL(count_calls("skiq_start_tx_streaming"), bursts.size());
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), bursts.size());
}
}
BOOST_FIXTURE_TEST_SUITE(tx_bursts, fixture)
BOOST_AUTO_TEST_CASE(offset_and_multiple_tags)
{
    check_bursts({{17, 53}, {70, 31}, {201, 97}}, tx_samples, tx_samples);
}
BOOST_AUTO_TEST_CASE(fragmented_burst_and_short_input)
{
    check_bursts({{11, 2 * tx_samples + 23}}, 2 * tx_samples + 34, 257);
}
BOOST_AUTO_TEST_CASE(reused_final_packet_is_zero_padded)
{
    check_bursts({{0, 20 * tx_samples + 7}}, 21 * tx_samples, tx_samples);
}
BOOST_AUTO_TEST_CASE(queue_full_preserves_burst_samples)
{
    fake_sidekiq::fail_next("skiq_transmit", SKIQ_TX_ASYNC_SEND_QUEUE_FULL);
    check_bursts({{0, tx_samples + 11}}, 2 * tx_samples, tx_samples, 2);
    BOOST_CHECK_EQUAL(count_calls("skiq_transmit"), 3);
}
BOOST_AUTO_TEST_CASE(no_tag_transmits_nothing)
{
    check_bursts({}, 19, 7);
}
BOOST_AUTO_TEST_CASE(async_burst_waits_for_completion)
{
    fake_sidekiq::set_auto_complete(false);
    auto graph = gr::make_top_block("qa_burst_drain");
    auto source = gr::blocks::vector_source_c::make(
        std::vector<gr_complex>(tx_samples, {0.25f, 0}), false, 1,
        burst_tags({{0, tx_samples}}));
    auto sink = make_burst_tx(2);
    graph->connect(source, 0, sink, 0);
    graph->start(tx_samples);
    BOOST_CHECK(wait_until([] { return count_calls("skiq_transmit") != 0; }));
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 0);
    BOOST_CHECK(fake_sidekiq::complete_one());
    graph->wait();
    BOOST_CHECK_EQUAL(fake_sidekiq::transmitted().size(), 1);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 1);
}
BOOST_AUTO_TEST_CASE(stop_interrupts_burst_drain)
{
    fake_sidekiq::set_auto_complete(false);
    auto graph = gr::make_top_block("qa_burst_abort");
    auto source = gr::blocks::vector_source_c::make(
        std::vector<gr_complex>(tx_samples, {0.25f, 0}), false, 1,
        burst_tags({{0, tx_samples}}));
    auto sink = make_burst_tx(2);
    graph->connect(source, 0, sink, 0);
    graph->start(tx_samples);
    BOOST_CHECK(wait_until([] { return fake_sidekiq::pending_count() == 1; }));
    graph->stop();
    graph->wait();
    BOOST_CHECK_EQUAL(fake_sidekiq::pending_count(), 0);
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_tx_streaming"), 1);
}

BOOST_AUTO_TEST_CASE(signed_length_and_unrelated_tag)
{
    auto tags = burst_tags({{5, 7}});
    tags[0].value = pmt::from_long(7);
    auto unrelated = tags[0];
    unrelated.offset = 0;
    unrelated.key = pmt::intern("unrelated");
    unrelated.value = pmt::PMT_NIL;
    tags.insert(tags.begin(), unrelated);
    auto graph = gr::make_top_block("qa_signed_burst");
    auto source = gr::blocks::vector_source_c::make(
        std::vector<gr_complex>(12, {0.25f, -0.25f}), false, 1, tags);
    auto sink = make_burst_tx();
    graph->connect(source, 0, sink, 0);
    graph->run(3);
    const auto packets = fake_sidekiq::transmitted();
    BOOST_REQUIRE_EQUAL(packets.size(), 1);
    for (size_t i = 0; i < packets[0].iq.size(); ++i) {
        const float expected = i < 14 ? (i % 2 ? -511.75f : 511.75f) : 0;
        BOOST_CHECK_LE(std::abs(packets[0].iq[i] - expected), 1.0f);
    }
}

BOOST_AUTO_TEST_CASE(restart_after_incomplete_burst)
{
    auto sink = make_burst_tx();
    {
        auto graph = gr::make_top_block("qa_truncated_burst");
        auto source = gr::blocks::vector_source_c::make(
            std::vector<gr_complex>(7, {0.25f, 0}), false, 1, burst_tags({{0, 100}}));
        graph->connect(source, 0, sink, 0);
        graph->run(3);
        graph->disconnect_all();
    }
    BOOST_CHECK(fake_sidekiq::transmitted().empty());
    const auto starts = count_calls("skiq_start_tx_streaming");
    auto graph = gr::make_top_block("qa_restart_idle");
    auto source = gr::blocks::vector_source_c::make(std::vector<gr_complex>(11), false);
    graph->connect(source, 0, sink, 0);
    graph->run(3);
    BOOST_CHECK_EQUAL(count_calls("skiq_start_tx_streaming"), starts);
    BOOST_CHECK(fake_sidekiq::transmitted().empty());
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_FIXTURE_TEST_SUITE(rx_correctness, fixture)
BOOST_AUTO_TEST_CASE(uneven_handles_preserve_samples_and_tags)
{
    std::vector<fake_sidekiq::rx_packet> script;
    for (int i = 0; i < 24; ++i) {
        const bool secondary = i >= 12;
        script.push_back({secondary ? skiq_rx_hdl_A2 : skiq_rx_hdl_A1,
                          uint64_t((secondary ? 9000 : 1000) + (i % 12) * rx_samples),
                          std::vector<int16_t>(2 * rx_samples, 100 + i)});
    }
    fake_sidekiq::set_rx_script(script);
    auto graph = gr::make_top_block("qa_rx_uneven");
    auto source = gr::sidekiq::sidekiq_rx::make(
        0, "A1", "A2", 1e6, 800e3, 915e6, 0, 10, 1, 0, 0, 2, 0);
    std::vector<gr::blocks::vector_sink_c::sptr> sinks;
    for (int port = 0; port < 2; ++port) {
        auto head = gr::blocks::head::make(sizeof(gr_complex), 12 * rx_samples);
        auto sink = gr::blocks::vector_sink_c::make();
        graph->connect(source, port, head, 0);
        graph->connect(head, 0, sink, 0);
        sinks.push_back(sink);
    }
    graph->run(rx_samples);
    for (int port = 0; port < 2; ++port) {
        const auto data = sinks[port]->data();
        BOOST_REQUIRE_EQUAL(data.size(), 12 * rx_samples);
        for (size_t i = 0; i < data.size(); ++i) {
            const float expected = (100 + port * 12 + i / rx_samples) / 2047.0f;
            BOOST_CHECK_SMALL(data[i].real() - expected, 1e-6f);
            BOOST_CHECK_SMALL(data[i].imag() - expected, 1e-6f);
        }
        const auto tags = sinks[port]->tags();
        BOOST_REQUIRE_EQUAL(tags.size(), 12);
        for (size_t i = 0; i < tags.size(); ++i) {
            BOOST_CHECK_EQUAL(tags[i].offset, i * rx_samples);
            BOOST_CHECK(pmt::eq(tags[i].key, pmt::intern("rf_timestamp")));
            BOOST_CHECK_EQUAL(pmt::to_uint64(tags[i].value),
                              (port ? 9000 : 1000) + i * rx_samples);
        }
    }
}
BOOST_AUTO_TEST_CASE(stop_without_rx_data)
{
    auto graph = gr::make_top_block("qa_rx_idle_stop");
    auto source = gr::sidekiq::sidekiq_rx::make(
        0, "A1", "none", 1e6, 800e3, 915e6, 0, 10, 1, 0, 0, 2, 0);
    auto sink = gr::blocks::vector_sink_c::make();
    graph->connect(source, 0, sink, 0);
    graph->start(rx_samples);
    BOOST_CHECK(wait_until([] { return count_calls("skiq_start_rx_streaming_multi_on_trigger") == 1; }));
    graph->stop();
    graph->wait();
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_rx_streaming_multi_on_trigger"), 1);
}
BOOST_AUTO_TEST_CASE(discontinuity_and_restart)
{
    // Capture the block's externally visible diagnostic, without adding a
    // testing-only accessor to the production block.
    std::ostringstream messages;
    auto capture = std::make_shared<spdlog::sinks::ostream_sink_mt>(messages);
    auto backend = std::dynamic_pointer_cast<spdlog::sinks::dist_sink_mt>(
        gr::logging::singleton().default_backend());
    BOOST_REQUIRE(backend);
    backend->add_sink(capture);
    struct detach {
        std::shared_ptr<spdlog::sinks::dist_sink_mt> backend;
        spdlog::sink_ptr sink;
        ~detach() { backend->remove_sink(sink); }
    } cleanup{backend, capture};
    auto source = gr::sidekiq::sidekiq_rx::make(
        0, "A1", "none", 1e6, 800e3, 915e6, 0, 10, 1, 0, 0, 2, 0);
    const std::vector<int16_t> iq(2 * rx_samples, 123);
    auto run = [&](const std::vector<fake_sidekiq::rx_packet>& script) {
        auto padded_script = script;
        // Supply contiguous lookahead for scheduler batching and Head shutdown.
        auto trailing = script.back();
        for (int i = 0; i < 32; ++i) {
            trailing.timestamp += rx_samples;
            padded_script.push_back(trailing);
        }
        fake_sidekiq::set_rx_script(padded_script, false);
        auto graph = gr::make_top_block("qa_rx_gap");
        auto head = gr::blocks::head::make(sizeof(gr_complex), script.size() * rx_samples);
        auto sink = gr::blocks::vector_sink_c::make();
        graph->connect(source, 0, head, 0);
        graph->connect(head, 0, sink, 0);
        graph->run(rx_samples);
        const auto tags = sink->tags();
        BOOST_REQUIRE_EQUAL(tags.size(), script.size());
        for (size_t i = 0; i < tags.size(); ++i) {
            BOOST_CHECK_EQUAL(tags[i].offset, i * rx_samples);
            BOOST_CHECK_EQUAL(pmt::to_uint64(tags[i].value), script[i].timestamp);
        }
        graph->disconnect_all();
    };
    run({{skiq_rx_hdl_A1, 100, iq}, {skiq_rx_hdl_A1, 100 + rx_samples + 17, iq}});
    BOOST_CHECK(messages.str().find("RX timestamp discontinuity") != std::string::npos);
    messages.str("");
    messages.clear();
    run({{skiq_rx_hdl_A1, 90000, iq}, {skiq_rx_hdl_A1, 90000 + rx_samples, iq}});
    BOOST_CHECK(messages.str().find("RX timestamp discontinuity") == std::string::npos);
}

BOOST_AUTO_TEST_CASE(stop_error_is_reported_without_throwing)
{
    auto source = gr::sidekiq::sidekiq_rx::make(
        0, "A1", "none", 1e6, 800e3, 915e6, 0, 10, 0, 0, 0, 2, 0);
    BOOST_CHECK(source->start());
    BOOST_CHECK(source->start());
    BOOST_CHECK_EQUAL(count_calls("skiq_start_rx_streaming_multi_on_trigger"), 1);
    fake_sidekiq::fail_next("skiq_stop_rx_streaming_multi_on_trigger", -EIO);
    BOOST_CHECK(!source->stop());
    BOOST_CHECK(source->stop());
    BOOST_CHECK(source->stop());
    BOOST_CHECK_EQUAL(count_calls("skiq_stop_rx_streaming_multi_on_trigger"), 2);
}
BOOST_AUTO_TEST_SUITE_END()

namespace {
auto make_cal_rx(const std::string& second = "A2")
{
    return gr::sidekiq::sidekiq_rx::make(
        0, "A1", second, 1e6, 800e3, 915e6, 0, 10, 0, 0, 0, 2, 0);
}
std::vector<fake_sidekiq::call> calls_after(const std::string& name, size_t begin)
{
    const auto all = fake_sidekiq::calls();
    std::vector<fake_sidekiq::call> result;
    for (size_t i = begin; i < all.size(); ++i)
        if (all[i].name == name) result.push_back(all[i]);
    return result;
}
}
BOOST_FIXTURE_TEST_SUITE(calibration, fixture)
BOOST_AUTO_TEST_CASE(manual_runs_each_selected_handle_once)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    auto begin = fake_sidekiq::calls().size();
    source->run_rx_cal(1);
    const auto calls = calls_after("skiq_run_rx_cal", begin);
    BOOST_REQUIRE_EQUAL(calls.size(), 2);
    BOOST_CHECK_EQUAL(calls[0].handle, skiq_rx_hdl_A1);
    BOOST_CHECK_EQUAL(calls[1].handle, skiq_rx_hdl_A2);
}
BOOST_AUTO_TEST_CASE(requested_subset_is_preserved)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    for (int type : {0, 1}) {
        const auto begin = fake_sidekiq::calls().size();
        source->set_rx_cal_type(type);
        const auto calls = calls_after("skiq_write_rx_cal_type_mask", begin);
        BOOST_REQUIRE_EQUAL(calls.size(), 2);
        for (const auto& call : calls)
            BOOST_CHECK_EQUAL(call.value, type == 0 ? skiq_rx_cal_type_dc_offset : skiq_rx_cal_type_quadrature);
    }
}
BOOST_AUTO_TEST_CASE(capabilities_are_checked_per_handle)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    fake_sidekiq::set_rx_cal_available(skiq_rx_hdl_A1, skiq_rx_cal_type_dc_offset);
    fake_sidekiq::set_rx_cal_available(skiq_rx_hdl_A2, skiq_rx_cal_type_quadrature);
    const auto begin = fake_sidekiq::calls().size();
    source->set_rx_cal_type(2);
    const auto calls = calls_after("skiq_write_rx_cal_type_mask", begin);
    BOOST_REQUIRE_EQUAL(calls.size(), 2);
    BOOST_CHECK_EQUAL(calls[0].value, skiq_rx_cal_type_dc_offset);
    BOOST_CHECK_EQUAL(calls[1].value, skiq_rx_cal_type_quadrature);
    BOOST_CHECK_EQUAL(calls_after("skiq_read_rx_cal_types_avail", begin).size(), 2);
}
BOOST_AUTO_TEST_CASE(capability_failure_does_not_write_unverified_mask)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    const auto begin = fake_sidekiq::calls().size();
    fake_sidekiq::fail_next("skiq_read_rx_cal_types_avail", -EIO);
    BOOST_CHECK_THROW(source->set_rx_cal_type(2), std::runtime_error);
    BOOST_CHECK(calls_after("skiq_write_rx_cal_type_mask", begin).empty());
}
BOOST_AUTO_TEST_CASE(manual_trigger_gating)
{
    auto source = make_cal_rx("none");
    const auto begin = fake_sidekiq::calls().size();
    source->run_rx_cal(1); // Off.
    source->set_rx_cal_mode(skiq_rx_cal_mode_auto);
    source->run_rx_cal(1);
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    source->run_rx_cal(0);
    BOOST_CHECK(calls_after("skiq_run_rx_cal", begin).empty());
    source->run_rx_cal(1);
    BOOST_CHECK_EQUAL(calls_after("skiq_run_rx_cal", begin).size(), 1);
}
BOOST_AUTO_TEST_CASE(unsupported_request_does_not_enable_other_algorithms)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    fake_sidekiq::set_rx_cal_available(skiq_rx_hdl_A2, skiq_rx_cal_type_quadrature);
    const auto begin = fake_sidekiq::calls().size();
    BOOST_CHECK_THROW(source->set_rx_cal_type(0), std::runtime_error);
    BOOST_CHECK(calls_after("skiq_write_rx_cal_type_mask", begin).empty());
    BOOST_CHECK_THROW(source->set_rx_cal_type(99), std::invalid_argument);
}
BOOST_AUTO_TEST_CASE(calibration_sdk_failures_are_reported)
{
    auto source = make_cal_rx();
    source->set_rx_cal_mode(skiq_rx_cal_mode_manual);
    fake_sidekiq::fail_next("skiq_write_rx_cal_type_mask", -EIO);
    BOOST_CHECK_THROW(source->set_rx_cal_type(0), std::runtime_error);
    BOOST_CHECK_NO_THROW(source->set_rx_cal_type(0));
    fake_sidekiq::fail_next("skiq_run_rx_cal", -EIO);
    BOOST_CHECK_THROW(source->run_rx_cal(1), std::runtime_error);
    BOOST_CHECK_NO_THROW(source->run_rx_cal(1));
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_FIXTURE_TEST_SUITE(sdk_lifetime, fixture)
BOOST_AUTO_TEST_CASE(rx_owner_can_be_destroyed_before_tx)
{
    auto rx = make_cal_rx("none");
    auto tx = make_tx();
    rx.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    BOOST_CHECK_NO_THROW(tx->set_tx_frequency(920e6));
    tx.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
}
BOOST_AUTO_TEST_CASE(tx_owner_can_be_destroyed_before_rx)
{
    auto tx = make_tx();
    auto rx = make_cal_rx("none");
    tx.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    BOOST_CHECK_NO_THROW(rx->set_rx_frequency(920e6));
    rx.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
}
BOOST_AUTO_TEST_CASE(failed_constructor_releases_sdk)
{
    fake_sidekiq::fail_next("skiq_read_parameters", -EIO);
    BOOST_CHECK_THROW(make_tx(), std::runtime_error);
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
    auto tx = make_tx();
    tx.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 2);
}
BOOST_AUTO_TEST_CASE(external_sdk_owner_is_preserved)
{
    uint8_t card = 0;
    BOOST_REQUIRE_EQUAL(skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1), 0);
    { auto tx = make_tx(); auto rx = make_cal_rx("none"); }
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    BOOST_CHECK_EQUAL(skiq_exit(), 0);
}
BOOST_AUTO_TEST_CASE(additional_card_uses_enable_and_preserves_owner_on_failure)
{
    auto first = std::make_unique<gr::sidekiq::sidekiq_session>(0);
    BOOST_CHECK(first->initialized_card());
    fake_sidekiq::fail_next("skiq_enable_cards", -EIO);
    BOOST_CHECK_THROW(gr::sidekiq::sidekiq_session(1), std::runtime_error);
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    {
        gr::sidekiq::sidekiq_session second(1);
        BOOST_CHECK(second.initialized_card());
        gr::sidekiq::sidekiq_session shared(1);
        BOOST_CHECK(!shared.initialized_card());
        BOOST_CHECK_EQUAL(count_calls("skiq_init"), 1);
        BOOST_CHECK_EQUAL(count_calls("skiq_enable_cards"), 2);
        first.reset();
        BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    }
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
}
BOOST_AUTO_TEST_CASE(concurrent_session_acquisition)
{
    auto owner = std::make_unique<gr::sidekiq::sidekiq_session>(0);
    std::vector<std::thread> threads;
    for (int i = 0; i < 8; ++i)
        threads.emplace_back([] { gr::sidekiq::sidekiq_session shared(0); });
    for (auto& thread : threads) thread.join();
    BOOST_CHECK_EQUAL(count_calls("skiq_init"), 1);
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    owner.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
}
BOOST_AUTO_TEST_CASE(rx_constructor_failure_does_not_release_other_block)
{
    auto owner = make_tx();
    fake_sidekiq::fail_next("skiq_read_parameters", -EIO);
    BOOST_CHECK_THROW(make_cal_rx("none"), std::runtime_error);
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 0);
    owner.reset();
    BOOST_CHECK_EQUAL(count_calls("skiq_exit"), 1);
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_FIXTURE_TEST_SUITE(command_refactor, fixture)
BOOST_AUTO_TEST_CASE(pair_and_dictionary_commands_keep_their_behavior)
{
    auto tx = std::dynamic_pointer_cast<gr::sidekiq::sidekiq_tx_impl>(make_tx());
    auto rx = std::dynamic_pointer_cast<gr::sidekiq::sidekiq_rx_impl>(make_cal_rx("none"));
    BOOST_REQUIRE(tx);
    BOOST_REQUIRE(rx);
    const auto begin = fake_sidekiq::calls().size();
    tx->handle_control_message(pmt::cons(pmt::intern("lo_freq"), pmt::from_double(920e6)));
    rx->handle_control_message(pmt::dict_add(pmt::make_dict(), pmt::intern("gain"), pmt::from_double(12)));
    const auto tx_calls = calls_after("skiq_write_tx_LO_freq", begin);
    const auto rx_calls = calls_after("skiq_write_rx_gain", begin);
    BOOST_REQUIRE_EQUAL(tx_calls.size(), 1);
    BOOST_CHECK_EQUAL(tx_calls[0].value, 920000000);
    BOOST_REQUIRE_EQUAL(rx_calls.size(), 1);
    BOOST_CHECK_EQUAL(rx_calls[0].value, 12);
    const auto end = fake_sidekiq::calls().size();
    BOOST_CHECK_NO_THROW(tx->handle_control_message(pmt::PMT_NIL));
    BOOST_CHECK_NO_THROW(rx->handle_control_message(pmt::PMT_NIL));
    BOOST_CHECK_EQUAL(fake_sidekiq::calls().size(), end);
}
BOOST_AUTO_TEST_SUITE_END()
