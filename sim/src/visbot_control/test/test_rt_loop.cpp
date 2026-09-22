#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "visbot_control/latest_value.hpp"
#include "visbot_control/rt_loop.hpp"

namespace vc = visbot_control;

TEST(SeqLock, NeverTearsUnderConcurrentWrites) {
    // Every state the reader can observe must satisfy a == ~b. Seed the lock
    // first so the default-constructed {0, 0} is never one of them, and never
    // ASSERT out of the loop: that would return with the writer still running
    // and the thread unjoined.
    struct Pair { uint64_t a, b; };
    vc::SeqLock<Pair> lock;
    lock.write({0, ~uint64_t(0)});

    std::atomic<bool> run{true};
    std::thread writer([&] {
        uint64_t i = 0;
        while (run) { lock.write({i, ~i}); ++i; }
    });

    uint64_t torn = 0, reads = 0;
    for (int i = 0; i < 200000; ++i) {
        const Pair p = lock.read();
        ++reads;
        if (p.a != ~p.b) ++torn;
    }
    run = false;
    writer.join();

    EXPECT_EQ(torn, 0u) << torn << " torn reads out of " << reads;
}

TEST(LatestValue, TracksAge) {
    vc::LatestValue<int> v;
    EXPECT_EQ(v.ageNs(), INT64_MAX);
    v.write(42);
    EXPECT_EQ(v.read().value, 42);
    EXPECT_LT(v.ageNs(), 50'000'000);
}

TEST(RealtimeBox, DeliversNewestOnly) {
    vc::RealtimeBox<int> box;
    int out = 0;
    EXPECT_FALSE(box.take(out));
    box.tryStore(1); box.tryStore(2);
    EXPECT_TRUE(box.take(out)); EXPECT_EQ(out, 2);
    EXPECT_FALSE(box.take(out));
}

TEST(Histogram, Percentiles) {
    vc::Histogram<100> h;
    h.bucketNs = 1000;
    for (int i = 0; i < 100; ++i) h.add(i * 1000);
    EXPECT_NEAR(h.percentile(0.5), 50'000, 1000);
    EXPECT_NEAR(h.percentile(0.99), 99'000, 1000);
    EXPECT_EQ(h.max, 99'000);
}

TEST(RtLoop, HoldsRateWithoutPrivileges) {
    // SCHED_OTHER, unpinned: must still hit the tick count and keep the grid
    // (no drift), even if individual wake latencies are loose.
    vc::RtConfig cfg;
    cfg.rateHz = 500;
    cfg.schedPolicy = SCHED_OTHER;
    cfg.lockMemory = false;
    std::atomic<uint64_t> ticks{0};
    vc::RtLoop loop(cfg);
    loop.start([&](const vc::TickContext&) { ++ticks; });
    std::this_thread::sleep_for(std::chrono::seconds(1));
    loop.stop();
    const vc::RtStats st = loop.stats();
    EXPECT_NEAR(double(ticks.load()), 500.0, 25.0);
    EXPECT_EQ(st.ticks, ticks.load());
    // Absolute-deadline sleeping: mean period error stays near zero.
    const double elapsed = double(st.lastTickNs - st.startNs) * 1e-9;
    EXPECT_NEAR(elapsed / double(st.ticks), 1.0 / 500.0, 2e-4);
    EXPECT_LT(st.wake.percentile(0.99), 20'000'000.0);  // < 20 ms even on a busy CI box
}
