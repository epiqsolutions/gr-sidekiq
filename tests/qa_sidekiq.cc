// SPDX-License-Identifier: GPL-3.0-or-later
#include "fake_sidekiq.h"
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
// belong to the next branch; this test does not claim that async is fixed.
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
