// SpscRing — single-producer/single-consumer lock-free ring buffer.
//
// For discrete events crossing cores (config commands, bind, telemetry frames) where every
// item matters. For channel/stats data — where the consumer wants the freshest value, not
// a backlog — use util/Mailbox.h instead.
//
// Capacity is N-1 usable slots; N must be a power of two. Cross-core ordering uses C++11
// acquire/release on the head/tail indices (not bare volatile): the release store of an
// index publishes the slot write before the index becomes visible, and the acquire load
// pairs with it. Compiles to `dmb` on Cortex-M0+/M33 and stays host-testable.
#pragma once
#include <stddef.h>
#include <atomic>

namespace xlrs {

template <typename T, size_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
public:
    bool push(const T& v) {                                   // producer side
        size_t h = _head.load(std::memory_order_relaxed);
        size_t n = (h + 1) & (N - 1);
        if (n == _tail.load(std::memory_order_acquire)) { ++_dropped; return false; } // full
        _buf[h] = v;
        _head.store(n, std::memory_order_release);            // publishes _buf[h]
        return true;
    }

    // Overflow policy: push() drops the NEWEST item when full and counts it. Callers that
    // cannot tolerate drops (config/bind commands) must size N so it never overflows and
    // treat a rising dropped() as a fault; lossy streams (telemetry) may ignore it. See §3.9.
    size_t dropped() const { return _dropped; }             // producer-owned counter

    bool pop(T& out) {                                        // consumer side
        size_t t = _tail.load(std::memory_order_relaxed);
        if (t == _head.load(std::memory_order_acquire)) return false;  // empty
        out = _buf[t];
        _tail.store((t + 1) & (N - 1), std::memory_order_release);
        return true;
    }

    bool empty() const {
        return _head.load(std::memory_order_acquire) == _tail.load(std::memory_order_acquire);
    }

private:
    T _buf[N] = {};
    std::atomic<size_t> _head{0};
    std::atomic<size_t> _tail{0};
    size_t _dropped = 0;        // producer-owned overflow counter (benign read race for stats)
};

} // namespace xlrs
