// Mailbox (LatestValue) — single-value, overwrite, "freshest wins" handoff.
//
// Use this — NOT a FIFO ring — for channel data and link stats crossing cores: if the
// consumer falls behind, it must see the newest value, never a backlog of stale ones.
// FIFO rings (util/RingBuffer.h) are for discrete events (config, bind, telemetry frames)
// where every item matters.
//
// Implemented as a seqlock: the reader retries if a write straddled its read, so it never
// observes a torn value. Cross-core correctness relies on C++11 acquire/release fences
// (NOT bare volatile — volatile blocks compiler reordering but emits no hardware barrier,
// so Core 0 could publish the sequence counter before the payload is visible to Core 1).
// std::atomic + thread fences compile to `dmb` on Cortex-M0+/M33 and stay host-testable.
//
// Single-producer / single-consumer. The whole T is copied as one snapshot, so the reader
// gets an all-or-nothing frame (no mixed-age fields — which per-element atomics would allow).
// Channel-handoff pattern (NO mutex — a lock here would block the RF slot deadline): the RF
// task copies the snapshot out ONCE per tick via loadOrKeep() into a private local, then
// packs from that copy; nothing is shared during packing. See loadOrKeep().
#pragma once
#include <stdint.h>
#include <atomic>
#include <type_traits>

namespace xlrs {

// T must be trivially copyable and small: the seqlock copies T whole, so a large T widens
// the reader's copy window and raises retry churn under contention. Keep it to a small POD
// (e.g. a channel-set struct or a LinkStats snapshot).
template <typename T>
class LatestValue {
    static_assert(std::is_trivially_copyable<T>::value,
                  "LatestValue<T>: T must be trivially copyable");
public:
    void store(const T& v) {                              // producer: wait-free
        uint32_t s = _seq.load(std::memory_order_relaxed);
        _seq.store(s + 1, std::memory_order_relaxed);     // odd => write in progress
        std::atomic_thread_fence(std::memory_order_release);
        _val = v;
        std::atomic_thread_fence(std::memory_order_release);
        _seq.store(s + 2, std::memory_order_relaxed);     // even => stable
    }

    bool load(T& out) const {                             // consumer: retry on straddle
        for (int tries = 0; tries < 8; ++tries) {
            uint32_t s1 = _seq.load(std::memory_order_relaxed);
            if (s1 & 1) continue;                         // writer active
            std::atomic_thread_fence(std::memory_order_acquire);
            T tmp = _val;
            std::atomic_thread_fence(std::memory_order_acquire);
            uint32_t s2 = _seq.load(std::memory_order_relaxed);
            if (s1 == s2) { out = tmp; return true; }
        }
        return false;
    }

    // Channel-handoff helper. Refresh `out` with the freshest value; on the (rare) failed
    // read under heavy contention, leave `out` unchanged so the consumer transparently
    // reuses its last-good snapshot (stale by one frame, never a stall). Returns true if
    // `out` was refreshed.
    //
    //   static ChannelSet local;                 // private to the RF task (consumer)
    //   mailbox.loadOrKeep(local);               // copy-out ONCE per tick (lock-free)
    //   ChannelPack::pack(local.ch, N, otaBuf);  // pack from the private copy — nothing shared
    bool loadOrKeep(T& out) const {
        T tmp;
        if (load(tmp)) { out = tmp; return true; }
        return false;  // keep prior value
    }

    bool loadIfNew(T& out, uint32_t& lastSeq) const {
        for (int tries = 0; tries < 8; ++tries) {
            uint32_t s1 = _seq.load(std::memory_order_relaxed);
            if (s1 & 1) continue;
            if (s1 == lastSeq) return false;
            std::atomic_thread_fence(std::memory_order_acquire);
            T tmp = _val;
            std::atomic_thread_fence(std::memory_order_acquire);
            uint32_t s2 = _seq.load(std::memory_order_relaxed);
            if (s1 == s2) { out = tmp; lastSeq = s1; return true; }
        }
        return false;
    }

private:
    std::atomic<uint32_t> _seq{0};
    T _val{};
};

} // namespace xlrs
