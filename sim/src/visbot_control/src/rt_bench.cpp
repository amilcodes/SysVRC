// rt_bench — measures the control loop's wake latency and execution time
// with no ROS in the process. The tick runs the real controller against the
// kinematic plant so the execution-time numbers are representative.
//
//   rt_bench --policy fifo --rate 120 --seconds 10 --cpu 2 --stress 4 --csv out.csv
//
// Prints a JSON summary to stdout. --stress N spawns N busy-loop threads to
// contend for CPU, which is where SCHED_OTHER falls apart and SCHED_FIFO
// doesn't.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "visbot/visbot.hpp"
#include "visbot_control/rt_loop.hpp"

namespace vc = visbot_control;

int main(int argc, char** argv) {
    vc::RtConfig cfg;
    double seconds = 10.0;
    int stress = 0;
    std::string csv;
    std::string policy = "fifo";
    for (int i = 1; i < argc; ++i) {
        auto next = [&](double& v) { if (i + 1 < argc) v = std::atof(argv[++i]); };
        if (!std::strcmp(argv[i], "--policy") && i + 1 < argc) policy = argv[++i];
        else if (!std::strcmp(argv[i], "--rate")) next(cfg.rateHz);
        else if (!std::strcmp(argv[i], "--seconds")) next(seconds);
        else if (!std::strcmp(argv[i], "--priority")) { double v = cfg.priority; next(v); cfg.priority = int(v); }
        else if (!std::strcmp(argv[i], "--cpu")) { double v = cfg.cpu; next(v); cfg.cpu = int(v); }
        else if (!std::strcmp(argv[i], "--stress")) { double v = 0; next(v); stress = int(v); }
        else if (!std::strcmp(argv[i], "--no-mlock")) cfg.lockMemory = false;
        else if (!std::strcmp(argv[i], "--poll-idle")) cfg.pollIdleCompanion = true;
        else if (!std::strcmp(argv[i], "--spin-us")) { double v = 0; next(v); cfg.spinBeforeDeadlineNs = int64_t(v * 1000); }
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv = argv[++i];
        else { std::fprintf(stderr, "unknown arg %s\n", argv[i]); return 2; }
    }
    cfg.schedPolicy = vc::RtLoop::policyFromName(policy);

    // CPU hogs: unpinned SCHED_OTHER busy loops.
    std::atomic<bool> stressRun{true};
    std::vector<std::thread> hogs;
    for (int i = 0; i < stress; ++i)
        hogs.emplace_back([&] { volatile double x = 1; while (stressRun) x = x * 1.000001 + 1e-9; });

    // Realistic workload: the controller + plant tick.
    visbot::RobotParams rp;
    visbot::DiffDrivePlant plant(rp);
    visbot::MotionController ctrl(rp);
    plant.reset({0, 0, 0});
    ctrl.resetPose({0, 0, 0}, plant.sensors());
    ctrl.setMission(visbot::missions::skillsLoop());

    struct Row { uint64_t seq; int64_t lateNs, execNs, dtNs; };
    std::vector<Row> rows;
    rows.reserve(size_t(cfg.rateHz * seconds) + 1024);  // preallocated: no malloc in the tick
    int64_t prevWake = 0;

    vc::RtLoop loop(cfg);
    const std::string note = loop.start([&](const vc::TickContext& c) {
        const int64_t t0 = vc::monotonicNs();
        visbot::WheelCmd cmd = ctrl.tick(plant.sensors(), c.dtSeconds);
        plant.step(cmd, c.dtSeconds);
        if (ctrl.status().done) ctrl.setMission(visbot::missions::skillsLoop());
        const int64_t t1 = vc::monotonicNs();
        if (rows.size() < rows.capacity())
            rows.push_back({c.seq, c.wakeNs - c.scheduledNs, t1 - t0, prevWake ? c.wakeNs - prevWake : 0});
        prevWake = c.wakeNs;
    });
    if (!note.empty()) std::fprintf(stderr, "warning: %s\n", note.c_str());

    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    loop.stop();
    stressRun = false;
    for (auto& h : hogs) h.join();

    const vc::RtStats st = loop.stats();
    const double periodUs = 1e6 / cfg.rateHz;
    std::printf("{\n");
    std::printf("  \"policy\": \"%s\", \"priority\": %d, \"cpu\": %d, \"mlock\": %s, \"stress_threads\": %d, \"poll_idle\": %s, \"spin_us\": %.0f,\n",
                vc::RtLoop::policyName(st.effectivePolicy), st.effectivePriority, st.effectiveCpu,
                st.memoryLocked ? "true" : "false", stress, cfg.pollIdleCompanion ? "true" : "false", cfg.spinBeforeDeadlineNs * 1e-3);
    std::printf("  \"rate_hz\": %.1f, \"period_us\": %.1f, \"seconds\": %.1f, \"ticks\": %llu,\n",
                cfg.rateHz, periodUs, seconds, (unsigned long long)st.ticks);
    std::printf("  \"wake_latency_us\": {\"mean\": %.1f, \"p50\": %.1f, \"p99\": %.1f, \"p999\": %.1f, \"max\": %.1f},\n",
                st.wake.mean() * 1e-3, st.wake.percentile(0.5) * 1e-3, st.wake.percentile(0.99) * 1e-3,
                st.wake.percentile(0.999) * 1e-3, st.wake.max * 1e-3);
    std::printf("  \"exec_us\": {\"mean\": %.1f, \"p50\": %.1f, \"p99\": %.1f, \"max\": %.1f},\n",
                st.exec.mean() * 1e-3, st.exec.percentile(0.5) * 1e-3, st.exec.percentile(0.99) * 1e-3, st.exec.max * 1e-3);
    std::printf("  \"period_jitter_rms_us\": %.1f, \"overruns\": %llu, \"missed_deadlines\": %llu,\n",
                st.period.rms() * 1e-3, (unsigned long long)st.overruns, (unsigned long long)st.missedDeadlines);
    std::printf("  \"deadline_budget_us\": %.1f, \"worst_case_response_us\": %.1f\n", periodUs,
                (st.wake.max + st.exec.max) * 1e-3);
    std::printf("}\n");

    if (!csv.empty()) {
        FILE* f = std::fopen(csv.c_str(), "w");
        if (!f) { std::perror("csv"); return 1; }
        std::fprintf(f, "seq,wake_latency_us,exec_us,period_us\n");
        for (const Row& r : rows) std::fprintf(f, "%llu,%.2f,%.2f,%.2f\n", (unsigned long long)r.seq, r.lateNs * 1e-3, r.execNs * 1e-3, r.dtNs * 1e-3);
        std::fclose(f);
    }
    return 0;
}
