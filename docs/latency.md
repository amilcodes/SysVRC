# Control-loop latency

Wake latency of the 120 Hz control thread (period 8333 µs), 10 s per configuration, measured by `rt_bench` with the real controller + plant as the workload.

Host: Linux 6.10.14-linuxkit in a Docker Desktop VM on Apple Silicon (10 vCPUs). A VM halts idle vCPUs, so bare timer wakeups are coarse (~3 ms); on bare-metal Linux the same code lands in the tens of microseconds. The point of the table is the *relative* effect of each mitigation.

| configuration | p50 | p99 | p99.9 | max | jitter rms | missed deadlines | overruns |
|---|---:|---:|---:|---:|---:|---:|---:|
| SCHED_OTHER | 2640 µs | 4880 µs | 5160 µs | 5182 µs | 1657 µs | 0 | 0 |
| SCHED_OTHER + 8 CPU hogs | 2720 µs | 6060 µs | 10240 µs | 32783 µs | 2310 µs | 8 | 1 |
| SCHED_FIFO 80 | 2660 µs | 5000 µs | 5300 µs | 5362 µs | 1822 µs | 0 | 0 |
| SCHED_FIFO 80 + 8 hogs | 2640 µs | 5160 µs | 5340 µs | 7763 µs | 1891 µs | 0 | 0 |
| FIFO + pinned + poll-idle | 360 µs | 700 µs | 720 µs | 1070 µs | 472 µs | 0 | 0 |
| FIFO + pinned + poll-idle + 8 hogs | 380 µs | 1260 µs | 5680 µs | 5994 µs | 640 µs | 0 | 0 |
| FIFO + pinned + poll-idle + 500 µs spin | 20 µs | 260 µs | 300 µs | 659 µs | 194 µs | 0 | 0 |
| FIFO + 3 ms hybrid spin | 40 µs | 1240 µs | 5320 µs | 6134 µs | 774 µs | 0 | 0 |

![latency](latency.png)
