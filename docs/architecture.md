# Architecture

```
                 ┌──────────────────────────── core/ (header-only C++17, no deps) ───────────────────────────┐
                 │  pid.hpp  slew.hpp  cheesy_drive.hpp  odometry.hpp  motion.hpp  plant.hpp  missions.hpp   │
                 └──────────────┬─────────────────────────────────────────────────────┬──────────────────────┘
                                │ #include                                            │ #include
              ┌─────────────────▼──────────────┐                    ┌─────────────────▼──────────────────────┐
              │ v5/  (PROS, V5 brain)          │                    │ sim/src/visbot_control                 │
              │ autons.cpp · drive.cpp · ...   │                    │ controller_node  ─ RT thread @120 Hz   │
              │ EZ-Template gains → constants  │                    │ plant_node       ─ fast backend        │
              └────────────────────────────────┘                    │ rt_bench         ─ latency benchmark   │
                                                                    └───┬──────────────────────▲─────────────┘
                                                        /visbot/cmd_vel │                      │ /visbot/imu (200 Hz)
                                                        (Twist)         │                      │ /visbot/joint_states (100 Hz)
                                                                        ▼                      │ /visbot/odom (50 Hz, truth)
                                            ┌──────────── one of ────────────┐                 │
                                            │ Gazebo Harmonic + ros_gz_bridge│─────────────────┘
                                            │   vrc_field.sdf, visbot.urdf   │
                                            ├────────────────────────────────┤
                                            │ plant_node (kinematic, 1 kHz)  │
                                            └────────────────────────────────┘
                                                                        │ /visbot/control_state (30 Hz)
                                                                        │ /visbot/control_stats (10 Hz)
                                                                        ▼
                                                           visbot_dash → ws://:8081 → browser
```

## The control thread

`visbot_control/rt_loop.hpp` is the whole point of the repo. `controller_node`
does not use a ROS timer; it owns a pthread that:

1. **elevates itself** to `SCHED_FIFO` (default priority 80) and optionally
   pins to one CPU, so DDS discovery, parameter services, logging and the
   executor can never preempt a tick;
2. **locks memory** (`mlockall(MCL_CURRENT|MCL_FUTURE)`) and pre-faults its
   stack so there are no page faults once running;
3. **sleeps to absolute deadlines** on a fixed grid with
   `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` — a late tick does not
   shift every later tick, and drift cannot accumulate the way it does with
   relative sleeps or `create_wall_timer`;
4. **re-anchors instead of storming** if it ever wakes ≥ 2 periods late
   (counted as missed deadlines), rather than firing a burst of catch-up ticks;
5. **records every tick**: wake latency (actual − scheduled), execution time
   and period error into fixed-bucket histograms, published through a
   seqlock so the non-RT side never blocks the RT side.

Two optional mitigations exist for hosts with coarse timer wakeups
(hypervisors, containers without `/dev/cpu_dma_latency`):

* `poll_idle`: a nice-19 companion thread pinned to the RT core keeps the
  core out of idle states; the FIFO thread preempts it instantly. This is the
  userspace equivalent of `idle=poll`.
* `spin_before_deadline_us`: hybrid sleep — return from `nanosleep` early and
  spin on the clock for the final stretch.

[latency.md](latency.md) has the measured effect of each.

## Data flow into and out of the RT thread

Nothing in the tick calls rclcpp.

| direction | mechanism | why |
|---|---|---|
| sensors → tick | `LatestValue<T>` (single-writer seqlock, stamped with arrival time) | subscription callbacks on the executor thread write; the tick reads the newest sample without locks or allocation and knows its age |
| tick → publisher | `RealtimeBox<T>` (try-lock + copy) | the tick never blocks; a dedicated publisher thread is woken with `notify_one` and does the rclcpp work |
| tick → dashboard | `SeqLock<StateSnapshot>` | 30 Hz timer reads a consistent snapshot of pose, mission and command |
| stats | `SeqLock<RtStats>` | 10 Hz timer reads the histograms |

Stale-sensor gating: if the freshest IMU or encoder sample is older than
`stale_limit_ms` (50 ms), the tick outputs zero and counts a stale tick. A
sensor dropout must not look like "the robot stopped turning".

## Controllers are shared, not mirrored

`core/` contains the actual algorithms from the competition code with the
hardware calls removed:

* `CheesyDrive` is the operator mixer from `v5/src/subsystemFiles/drive.cpp`
  (turn remapping, negative inertia, quick-stop accumulators);
* `Pid` implements EZ-Template semantics (`start_i` gating, sign-flip
  integral reset, small/big/velocity exit conditions);
* `DriveGains` in `constants.hpp` are the literal numbers from
  `default_constants()` in `autons.cpp`;
* `MotionController` provides `driveDistance` / `turnTo` / `driveToPoint` /
  `wait` steps with the same exit-condition behaviour as
  `chassis.pid_drive_set` / `pid_turn_set` / `pid_odom_set`.

EZ runs at 100 Hz and its kD is per-tick. `Pid` rescales the derivative and
integral to a 10 ms tick from whatever real `dt` it is given, so the same
gains behave identically in the 120 Hz sim loop — `test_pid.cpp` checks this.

## Timing model vs. the V5

| | V5 brain | sim |
|---|---|---|
| control loop | 10 ms PROS task (`pros::delay(10)`) | 8.33 ms RT thread |
| IMU | 10 ms nominal | 200 Hz Gazebo IMU / plant |
| motor encoders | 10 ms poll | 100 Hz joint states |
| motor response | ~100–150 ms to speed | 120 ms first-order lag (plant) / accel limits (Gazebo diff-drive) |
| physics | — | 1 kHz ODE, RTF 1.0 |

## Backends

The controller subscribes to the same three topics from either backend, so
swapping Gazebo for the kinematic plant is a launch-file choice, not a code
change. The plant is deterministic (xorshift noise, fixed seed) and runs in CI.
