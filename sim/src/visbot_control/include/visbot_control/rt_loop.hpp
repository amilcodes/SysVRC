// visbot_control/rt_loop.hpp — a fixed-rate control thread with bounded wake
// latency, and the instrumentation to prove it.
//
// What makes this different from `create_wall_timer`:
//   * its own pthread, elevated to SCHED_FIFO and pinned to one CPU, so the
//     executor's DDS traffic, logging and parameter services never preempt it
//   * mlockall + a pre-faulted stack: no page faults once running
//   * sleeps to an *absolute* deadline (clock_nanosleep TIMER_ABSTIME) on a
//     fixed grid, so drift never accumulates and a late tick doesn't shift
//     every later tick
//   * every tick records wake latency and execution time into lock-free
//     histograms that the non-RT side reads through a seqlock
//
// No ROS types in here: the benchmark binary links this with nothing else.
#pragma once

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

namespace visbot_control {

inline int64_t monotonicNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

struct RtConfig {
    double rateHz = 120.0;
    int schedPolicy = SCHED_FIFO;   // SCHED_FIFO | SCHED_RR | SCHED_OTHER
    int priority = 80;              // 1..99 for FIFO/RR
    int cpu = -1;                   // pin to this CPU, -1 = don't pin
    bool lockMemory = true;
    size_t stackPrefaultBytes = 256 * 1024;
    /// If we wake this many periods late, snap the grid to now instead of
    /// trying to catch up with a burst of back-to-back ticks.
    int resyncAfterPeriods = 2;
    /// Hybrid sleep: return from clock_nanosleep this long before the
    /// deadline and spin on the clock for the remainder. Trades CPU for
    /// wake precision on kernels/hypervisors with coarse timer wakeups.
    int64_t spinBeforeDeadlineNs = 0;
    /// Keep the pinned RT core from entering an idle state by running a
    /// nice-19 busy thread on it (userspace stand-in for idle=poll or
    /// /dev/cpu_dma_latency=0, which VMs and containers usually lack).
    /// Requires cpu >= 0. The FIFO control thread preempts it instantly.
    bool pollIdleCompanion = false;
};

/// Fixed-bucket histogram with percentile queries. Single writer.
template <size_t N>
struct Histogram {
    static constexpr size_t kBuckets = N;
    double bucketNs = 10'000.0;  // 10 us
    std::array<uint32_t, N> counts{};
    uint64_t total = 0;
    double sum = 0, sumSq = 0;
    int64_t min = INT64_MAX, max = 0;

    void add(int64_t v) {
        const int64_t c = std::max<int64_t>(v, 0);
        size_t i = static_cast<size_t>(c / bucketNs);
        if (i >= N) i = N - 1;  // overflow bucket
        ++counts[i];
        ++total;
        sum += double(v);
        sumSq += double(v) * double(v);
        min = std::min(min, v);
        max = std::max(max, v);
    }
    double mean() const { return total ? sum / double(total) : 0.0; }
    double rms() const { return total ? std::sqrt(sumSq / double(total)) : 0.0; }
    /// Upper edge of the bucket containing the p-th percentile (0..1).
    double percentile(double p) const {
        if (!total) return 0.0;
        const uint64_t target = uint64_t(std::ceil(p * double(total)));
        uint64_t acc = 0;
        for (size_t i = 0; i < N; ++i) {
            acc += counts[i];
            if (acc >= target) return double(i + 1) * bucketNs;
        }
        return double(N) * bucketNs;
    }
    void clear() { *this = Histogram{}; }
};

/// Snapshot handed to the non-RT side.
struct RtStats {
    static constexpr size_t kHistBuckets = 512;
    Histogram<kHistBuckets> wake;       // wake latency (ns)          20 us buckets -> 10.24 ms span
    Histogram<kHistBuckets> exec;       // callback duration (ns)      2 us buckets ->  1.02 ms span
    Histogram<kHistBuckets> period;     // |actual - nominal period|  20 us buckets
    RtStats() { wake.bucketNs = 20'000; exec.bucketNs = 2'000; period.bucketNs = 20'000; }
    uint64_t ticks = 0;
    uint64_t overruns = 0;
    uint64_t missedDeadlines = 0;
    int64_t startNs = 0;
    int64_t lastTickNs = 0;
    bool memoryLocked = false;
    int effectivePolicy = SCHED_OTHER;
    int effectivePriority = 0;
    int effectiveCpu = -1;
};

struct TickContext {
    uint64_t seq;
    int64_t scheduledNs;   // when this tick was supposed to start
    int64_t wakeNs;        // when it actually started
    double dtSeconds;      // nominal period
    double actualDtSeconds;
};

/// Single-writer seqlock so the RT thread can publish a snapshot of a
/// trivially-copyable struct without ever blocking; readers retry if they
/// observe a torn write.
template <typename T>
class SeqLock {
public:
    void write(const T& v) {
        seq_.fetch_add(1, std::memory_order_release);   // odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(&data_, &v, sizeof(T));
        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);   // even: stable
    }
    T read() const {
        T out;
        for (;;) {
            const uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u) { std::this_thread::yield(); continue; }
            std::atomic_thread_fence(std::memory_order_acquire);
            std::memcpy(&out, &data_, sizeof(T));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_acquire) == s1) return out;
        }
    }
private:
    std::atomic<uint32_t> seq_{0};
    alignas(64) T data_{};
};

class RtLoop {
public:
    using Callback = std::function<void(const TickContext&)>;

    explicit RtLoop(RtConfig cfg) : cfg_(cfg) {}
    ~RtLoop() { stop(); }

    /// Start the loop. Returns a human-readable note if scheduling could not
    /// be elevated (e.g. missing CAP_SYS_NICE) — the loop still runs.
    std::string start(Callback cb) {
        cb_ = std::move(cb);
        running_.store(true);
        if (cfg_.pollIdleCompanion && cfg_.cpu >= 0) {
            companion_ = std::thread([this] {
                pthread_setname_np(pthread_self(), "visbot_pollidl");
                cpu_set_t set; CPU_ZERO(&set); CPU_SET(cfg_.cpu, &set);
                pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
                setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
                volatile uint64_t x = 0;
                while (running_.load(std::memory_order_relaxed)) { x = x + 1; }
            });
        }
        thread_ = std::thread([this] { run(); });
        // Wait until the thread has applied its scheduling so callers can log it.
        while (!setupDone_.load(std::memory_order_acquire)) std::this_thread::yield();
        return setupNote_;
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (thread_.joinable()) thread_.join();
        if (companion_.joinable()) companion_.join();
    }

    RtStats stats() const { return statsLock_.read(); }
    void resetStats() { resetRequested_.store(true); }
    const RtConfig& config() const { return cfg_; }

    static const char* policyName(int p) {
        switch (p) {
            case SCHED_FIFO: return "SCHED_FIFO";
            case SCHED_RR: return "SCHED_RR";
            default: return "SCHED_OTHER";
        }
    }
    static int policyFromName(const std::string& s) {
        if (s == "fifo" || s == "SCHED_FIFO") return SCHED_FIFO;
        if (s == "rr" || s == "SCHED_RR") return SCHED_RR;
        return SCHED_OTHER;
    }

private:
    void applyScheduling() {
        std::string note;
        if (cfg_.lockMemory) {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) stats_.memoryLocked = true;
            else note += "mlockall failed (" + std::string(strerror(errno)) + "); ";
        }
        if (cfg_.schedPolicy != SCHED_OTHER) {
            sched_param sp{};
            sp.sched_priority = cfg_.priority;
            const int rc = pthread_setschedparam(pthread_self(), cfg_.schedPolicy, &sp);
            if (rc != 0) note += "pthread_setschedparam(" + std::string(policyName(cfg_.schedPolicy)) +
                                 ") failed (" + strerror(rc) + ") — run with CAP_SYS_NICE / rtprio ulimit; ";
        }
        prctl(PR_SET_TIMERSLACK, 1);  // don't let the kernel coalesce our wakeups
        if (cfg_.cpu >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(cfg_.cpu, &set);
            if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
                note += "cpu pin failed (" + std::string(strerror(errno)) + "); ";
        }
        // Pre-fault the stack so the first deep call doesn't page-fault under RT.
        volatile char* buf = static_cast<volatile char*>(alloca(cfg_.stackPrefaultBytes));
        for (size_t i = 0; i < cfg_.stackPrefaultBytes; i += 4096) buf[i] = 0;

        int policy = 0;
        sched_param got{};
        pthread_getschedparam(pthread_self(), &policy, &got);
        stats_.effectivePolicy = policy;
        stats_.effectivePriority = got.sched_priority;
        stats_.effectiveCpu = cfg_.cpu >= 0 ? cfg_.cpu : sched_getcpu();
        setupNote_ = note;
    }

    void run() {
        pthread_setname_np(pthread_self(), "visbot_rt");
        applyScheduling();
        statsLock_.write(stats_);  // so stats() reflects the scheduling before the first tick
        setupDone_.store(true, std::memory_order_release);

        const int64_t periodNs = int64_t(std::llround(1e9 / cfg_.rateHz));
        const double dt = 1.0 / cfg_.rateHz;
        stats_.startNs = monotonicNs();
        int64_t next = stats_.startNs + periodNs;
        int64_t prevWake = stats_.startNs;
        uint64_t seq = 0;

        while (running_.load(std::memory_order_relaxed)) {
            const int64_t sleepUntil = next - cfg_.spinBeforeDeadlineNs;
            timespec ts{sleepUntil / 1'000'000'000LL, sleepUntil % 1'000'000'000LL};
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
            if (cfg_.spinBeforeDeadlineNs > 0)
                while (monotonicNs() < next) { __asm__ __volatile__("" ::: "memory"); }

            const int64_t wake = monotonicNs();
            const int64_t late = wake - next;

            if (late >= cfg_.resyncAfterPeriods * periodNs) {
                // Hopelessly late (someone stole the CPU). Count it, drop the
                // missed slots and re-anchor the grid rather than storming.
                const int64_t skipped = late / periodNs;
                stats_.missedDeadlines += uint64_t(skipped);
                next = wake - (late % periodNs);
            }

            if (resetRequested_.exchange(false)) {
                const RtStats fresh;
                stats_.wake = fresh.wake; stats_.exec = fresh.exec; stats_.period = fresh.period;
                stats_.ticks = stats_.overruns = stats_.missedDeadlines = 0;
                stats_.startNs = wake;
            }

            const TickContext ctx{seq++, next, wake, dt, double(wake - prevWake) * 1e-9};
            cb_(ctx);
            const int64_t done = monotonicNs();

            stats_.wake.add(late);
            stats_.exec.add(done - wake);
            stats_.period.add(std::llabs((wake - prevWake) - periodNs));
            if (done - wake > periodNs) ++stats_.overruns;
            ++stats_.ticks;
            stats_.lastTickNs = wake;
            prevWake = wake;

            statsLock_.write(stats_);
            next += periodNs;
        }
        if (stats_.memoryLocked) munlockall();
    }

    RtConfig cfg_;
    Callback cb_;
    std::thread thread_;
    std::thread companion_;
    std::atomic<bool> running_{false};
    std::atomic<bool> setupDone_{false};
    std::atomic<bool> resetRequested_{false};
    std::string setupNote_;
    RtStats stats_{};             // RT-thread private
    SeqLock<RtStats> statsLock_;  // published copy
};

}  // namespace visbot_control
