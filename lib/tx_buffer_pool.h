// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <sidekiq_api.h>
#include <boost/thread.hpp>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace gr {
namespace sidekiq {

// The callback owns only pool state, never the GNU Radio block. Slots and their
// SDK-aligned storage have stable addresses for the entire transfer lifetime.
class tx_buffer_pool : public std::enable_shared_from_this<tx_buffer_pool>
{
    struct block_deleter {
        void operator()(skiq_tx_block_t* p) const { skiq_tx_block_free(p); }
    };
    struct slot {
        tx_buffer_pool* owner;
        std::unique_ptr<skiq_tx_block_t, block_deleter> block;
        bool busy = false;
        slot(tx_buffer_pool* pool, uint32_t words)
            : owner(pool), block(skiq_tx_block_allocate(words))
        {
            if (!block) throw std::bad_alloc();
        }
    };

public:
    // An unsubmitted reservation is returned on every exit path, including
    // exceptions and GNU Radio thread interruption. Accepted packets are released
    // by the SDK callback (or synchronously when no callback is used).
    class lease {
    public:
        lease() = default;
        lease(std::shared_ptr<tx_buffer_pool> pool, slot* buffer)
            : pool_(std::move(pool)), slot_(buffer) {}
        lease(lease&& other) noexcept
            : pool_(std::move(other.pool_)), slot_(other.slot_), submitted_(other.submitted_) {}
        lease(const lease&) = delete;
        lease& operator=(const lease&) = delete;
        ~lease() { if (pool_ && !submitted_) pool_->release(slot_, 0); }
        explicit operator bool() const { return bool(pool_); }
        skiq_tx_block_t* block() const { return slot_->block.get(); }
        void* context() const { return slot_; }
        void handoff() { submitted_ = true; }
    private:
        std::shared_ptr<tx_buffer_pool> pool_;
        slot* slot_ = nullptr;
        bool submitted_ = false;
    };

    tx_buffer_pool(size_t count, uint32_t words)
    {
        slots_.reserve(count);
        for (size_t i = 0; i < count; ++i)
            slots_.emplace_back(new slot(this, words));
    }

    lease acquire(size_t index)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        auto* buffer = slots_.at(index).get();
        while (buffer->busy && !stopping_) {
            check_error();
            interruptible_wait(lock);
        }
        boost::this_thread::interruption_point();
        check_error();
        if (stopping_) return {};
        buffer->busy = true;
        ++occupied_;
        // No allocation per packet. Keep storage/context alive even if stop
        // fails and a callback arrives after the sink has been destroyed.
        keepalive_ = shared_from_this();
        return lease(shared_from_this(), buffer);
    }

    uint64_t generation()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return generation_;
    }

    void wait_for_completion(uint64_t previous)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        check_error();
        // A completion can precede the queue-full return. Compare against the
        // generation captured BEFORE submission so that notification isn't lost.
        // Also retry periodically: another sink may occupy the SDK's shared queue.
        if (!stopping_ && generation_ == previous) interruptible_wait(lock);
        check_error();
    }

    bool stopping()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopping_;
    }
    void cancel()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        available_.notify_all();
    }
    void restart()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (occupied_) throw std::runtime_error("TX callbacks still outstanding at restart");
        stopping_ = false;
        error_ = 0;
    }
    int32_t error()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_;
    }
    static void complete(int32_t status, skiq_tx_block_t*, void* context) noexcept
    {
        if (!context) return;
        auto* buffer = static_cast<slot*>(context);
        buffer->owner->release(buffer, status);
    }

private:
    void interruptible_wait(std::unique_lock<std::mutex>& lock)
    {
        boost::this_thread::interruption_point();
        available_.wait_for(lock, std::chrono::milliseconds(10));
        boost::this_thread::interruption_point();
    }
    void check_error() const
    {
        if (error_) throw std::runtime_error("Async TX completion failed: " + std::to_string(error_));
    }
    void release(slot* buffer, int32_t status)
    {
        std::shared_ptr<tx_buffer_pool> retired;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (status != 0 && status != -2 && error_ == 0) error_ = status;
            if (buffer->busy) {
                buffer->busy = false;
                --occupied_;
                ++generation_;
            }
            if (occupied_ == 0) retired = std::move(keepalive_);
            available_.notify_all();
        }
        // Last callback may own the last reference. Release it after unlocking.
    }
    std::vector<std::unique_ptr<slot>> slots_;
    std::mutex mutex_;
    std::condition_variable available_;
    size_t occupied_ = 0;
    uint64_t generation_ = 0;
    bool stopping_ = false;
    int32_t error_ = 0;
    std::shared_ptr<tx_buffer_pool> keepalive_;
};

} // namespace sidekiq
} // namespace gr
