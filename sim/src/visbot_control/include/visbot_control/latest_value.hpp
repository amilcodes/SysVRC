// visbot_control/latest_value.hpp — single-writer / single-reader mailbox for
// sensor samples. The subscription callback (executor thread) writes, the RT
// thread reads the newest sample without taking a lock or allocating.
//
// Also stamps each write with the monotonic receive time so the RT tick can
// reject stale data ("the IMU stopped 80 ms ago" must not look like "the
// robot stopped turning").
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "visbot_control/rt_loop.hpp"

namespace visbot_control {

template <typename T>
class LatestValue {
public:
    struct Stamped {
        T value{};
        int64_t rxNs = 0;    // monotonic time of arrival
        uint64_t seq = 0;    // increments per write
    };

    void write(const T& v) {
        Stamped s;
        s.value = v;
        s.rxNs = monotonicNs();
        s.seq = ++writes_;
        lock_.write(s);
    }

    Stamped read() const { return lock_.read(); }

    /// Age of the newest sample in nanoseconds, or INT64_MAX if none yet.
    int64_t ageNs() const {
        const Stamped s = lock_.read();
        return s.seq == 0 ? INT64_MAX : monotonicNs() - s.rxNs;
    }

private:
    SeqLock<Stamped> lock_;
    uint64_t writes_ = 0;
};

/// RT -> non-RT handoff for outbound messages (the realtime_tools pattern):
/// the RT thread try-locks and copies; if the publisher thread happens to be
/// mid-copy the RT thread simply skips this tick's update rather than block.
template <typename T>
class RealtimeBox {
public:
    bool tryStore(const T& v) {
        if (busy_.exchange(true, std::memory_order_acquire)) return false;
        std::memcpy(&data_, &v, sizeof(T));
        seq_.fetch_add(1, std::memory_order_release);
        busy_.store(false, std::memory_order_release);
        return true;
    }
    /// Returns true if a new value has arrived since the last take().
    bool take(T& out) {
        const uint64_t s = seq_.load(std::memory_order_acquire);
        if (s == taken_) return false;
        while (busy_.exchange(true, std::memory_order_acquire)) {}
        std::memcpy(&out, &data_, sizeof(T));
        busy_.store(false, std::memory_order_release);
        taken_ = s;
        return true;
    }
private:
    std::atomic<bool> busy_{false};
    std::atomic<uint64_t> seq_{0};
    uint64_t taken_ = 0;
    alignas(64) T data_{};
};

}  // namespace visbot_control
