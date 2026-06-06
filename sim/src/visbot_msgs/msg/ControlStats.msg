# Timing statistics for the real-time control loop, published at ~10 Hz.
# All latencies are measured with CLOCK_MONOTONIC inside the RT thread.
std_msgs/Header header

float64 rate_hz              # configured loop rate
float64 period_us            # 1e6 / rate_hz
string  sched_policy         # "SCHED_FIFO" | "SCHED_RR" | "SCHED_OTHER"
int32   priority
int32   cpu                  # pinned CPU, -1 = unpinned
bool    memory_locked

uint64  ticks
uint64  overruns             # callback ran longer than one period
uint64  missed_deadlines     # woke >= one full period late (grid resynced)
uint64  stale_sensor_ticks   # ticks that ran on sensor data older than the limit

# wake latency = actual wake time - scheduled wake time
float64 wake_latency_mean_us
float64 wake_latency_p50_us
float64 wake_latency_p99_us
float64 wake_latency_max_us
float64 jitter_rms_us        # RMS of (actual period - nominal period)

# callback execution time
float64 exec_mean_us
float64 exec_p50_us
float64 exec_p99_us
float64 exec_max_us

float64 sensor_age_mean_us   # age of the freshest IMU sample at tick start
float64 window_seconds       # how long the stats have been accumulating

# wake-latency histogram: bucket i counts latencies in [i*bucket_us, (i+1)*bucket_us)
float64 hist_bucket_us
uint32[] wake_latency_hist
float64 exec_hist_bucket_us
uint32[] exec_hist
