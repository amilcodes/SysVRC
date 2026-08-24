#!/usr/bin/env python3
"""Turn docs/bench/*.csv|json into docs/latency.png and docs/latency.md."""
import glob
import json
import os
import sys

import numpy as np

root = os.path.join(os.path.dirname(__file__), "..", "..", "docs")
bench = os.path.join(root, "bench")
order = ["other_idle", "other_stress", "fifo_idle", "fifo_stress", "fifo_pin_pollidle",
         "fifo_pin_pollidle_stress", "fifo_pin_pollidle_spin", "fifo_spin3000"]
labels = {
    "other_idle": "SCHED_OTHER",
    "other_stress": "SCHED_OTHER + 8 CPU hogs",
    "fifo_idle": "SCHED_FIFO 80",
    "fifo_stress": "SCHED_FIFO 80 + 8 hogs",
    "fifo_pin_pollidle": "FIFO + pinned + poll-idle",
    "fifo_pin_pollidle_stress": "FIFO + pinned + poll-idle + 8 hogs",
    "fifo_pin_pollidle_spin": "FIFO + pinned + poll-idle + 500 µs spin",
    "fifo_spin3000": "FIFO + 3 ms hybrid spin",
}
runs = {}
for name in order:
    j = os.path.join(bench, name + ".json")
    c = os.path.join(bench, name + ".csv")
    if os.path.exists(j) and os.path.exists(c):
        runs[name] = (json.load(open(j)), np.genfromtxt(c, delimiter=",", names=True))
if not runs:
    sys.exit("no bench results in docs/bench — run sim/tools/bench.sh first")

# ---- markdown table ----
lines = ["| configuration | p50 | p99 | p99.9 | max | jitter rms | missed deadlines | overruns |",
         "|---|---:|---:|---:|---:|---:|---:|---:|"]
for name, (j, _) in runs.items():
    w = j["wake_latency_us"]
    lines.append(f"| {labels[name]} | {w['p50']:.0f} µs | {w['p99']:.0f} µs | {w['p999']:.0f} µs | {w['max']:.0f} µs | "
                 f"{j['period_jitter_rms_us']:.0f} µs | {j['missed_deadlines']} | {j['overruns']} |")
kernel = open(os.path.join(bench, "kernel.txt")).read().split() if os.path.exists(os.path.join(bench, "kernel.txt")) else ["?", "?"]
first = next(iter(runs.values()))[0]
md = ["# Control-loop latency", "",
      f"Wake latency of the {first['rate_hz']:.0f} Hz control thread (period {first['period_us']:.0f} µs), "
      f"{first['seconds']:.0f} s per configuration, measured by `rt_bench` with the real controller + plant as the workload.", "",
      f"Host: Linux {kernel[0]} in a Docker Desktop VM on Apple Silicon ({kernel[1]} vCPUs). "
      "A VM halts idle vCPUs, so bare timer wakeups are coarse (~3 ms); on bare-metal Linux the same code "
      "lands in the tens of microseconds. The point of the table is the *relative* effect of each mitigation.", "",
      *lines, "", "![latency](latency.png)", ""]
open(os.path.join(root, "latency.md"), "w").write("\n".join(md))
print("\n".join(lines))

# ---- figure ----
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib missing; wrote latency.md only")

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9})
fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), dpi=150)
colors = plt.cm.viridis(np.linspace(0.1, 0.95, len(runs)))

ax = axes[0]
for (name, (j, d)), col in zip(runs.items(), colors):
    lat = np.sort(d["wake_latency_us"])
    ax.plot(lat, 1 - np.arange(len(lat)) / len(lat), label=labels[name], color=col, lw=1.4)
ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlabel("wake latency (µs)"); ax.set_ylabel("P(latency > x)")
ax.axvline(first["period_us"], color="#ef476f", ls="--", lw=1); ax.text(first["period_us"], 0.5, " period", color="#ef476f", va="center")
ax.set_title("Complementary CDF of wake latency"); ax.grid(alpha=.3, which="both"); ax.legend(fontsize=7, loc="lower left")

ax = axes[1]
worst = "other_stress" if "other_stress" in runs else next(iter(runs))
best = "fifo_pin_pollidle" if "fifo_pin_pollidle" in runs else list(runs)[-1]
for name, col in ((worst, "#ef476f"), (best, "#4cc9f0")):
    d = runs[name][1]
    t = np.arange(len(d)) / first["rate_hz"]
    ax.plot(t[:600], d["wake_latency_us"][:600], color=col, lw=0.8, label=labels[name])
ax.set_xlabel("time (s)"); ax.set_ylabel("wake latency (µs)"); ax.set_title("First 5 s, tick by tick")
ax.grid(alpha=.3); ax.legend(fontsize=7)
fig.tight_layout()
fig.savefig(os.path.join(root, "latency.png"))
print("wrote docs/latency.png, docs/latency.md")
