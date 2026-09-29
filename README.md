# SysVRC

Sim testbed for our VRC robot. ROS 2 Jazzy + Gazebo Harmonic, running a port of the same EZ-Template drive code the V5 brain runs, on a 120 Hz control loop that actually holds its timing.

The idea: tuning autons on the real robot costs a field, a charged battery and a teammate. Here you can run the drive PID, odometry and motion sequencing against physics first, with the same gains and exit conditions, and see the loop timing as numbers instead of guessing why the robot wobbled.

![dashboard](docs/dashboard.png)

## What lives where

```text
SysVRC/
  core/    the drive controllers, header-only C++, no deps
  sim/     ROS 2 workspace (control loop, gazebo model + field, dashboard, tools)
  v5/      our PROS competition code, same as it always was
  docs/    how it works, latency numbers, a recorded run
```

### `core/`

The controllers with the hardware calls ripped out: EZ-Template's PID, slew, and motion tasks, cheesy drive from `drive.cpp`, encoder + IMU odometry, and a simple kinematic drivebase model for testing.

It's a port of EZ-Template v3.2.2, not a rewrite from the docs. EZ has some weird behavior (the integral reset compares against the wrong variable, big_error exits can never fire if you set small_error, etc). The port keeps all of it on purpose, because that's what the robot actually does. [docs/fidelity.md](docs/fidelity.md) lists every quirk, the test that pins it, and what isn't ported yet.

Gains are in `core/include/visbot/constants.hpp`. They're copied from `default_constants()` in `v5/src/autons.cpp`, so if you retune on the robot, update both. The firmware has `core/include` on its include path but doesn't use any of it yet.

### `sim/`

Five ROS 2 packages. The important one is `visbot_control`:

* `controller_node` runs the controller on its own SCHED_FIFO thread, not a ROS timer. Absolute-deadline sleeps, locked memory, lock-free handoff to/from the ROS side. Every tick's wake latency goes into a histogram on `/visbot/control_stats`.
* `plant_node` is a fast fake robot for when you don't need Gazebo. Deterministic, which makes it good for CI.
* `rt_bench` measures loop latency with no ROS at all.

The rest: `visbot_description` (robot model), `visbot_gazebo` (12 ft field + launch), `visbot_msgs`, and `visbot_dash` (the browser dashboard). [docs/architecture.md](docs/architecture.md) goes through how the pieces connect.

### `v5/`

Mostly the old repo, moved into a folder. See [v5/README.md](v5/README.md). Heads up: `v5/include/` and `project.pros` were never committed, so `pros make` won't work from a fresh clone until those are added back.

## Getting started

You need Docker. Everything runs in a Linux container, and it works natively on Apple Silicon.

```bash
git clone https://github.com/amilcodes/SysVRC && cd SysVRC
docker compose -f sim/docker/compose.yaml build
sim/tools/build.sh                                   # colcon build inside the container
```

The first image build takes a while. The build output lives in docker volumes, so rebuilds after that are quick.

Inside the container you start in `/ws` and the repo is mounted at `/ws/src/sysvrc`, which is why the tool commands below start with `src/sysvrc/`.

## Running it

Fake robot, no Gazebo (fast, what CI uses):

```bash
docker compose -f sim/docker/compose.yaml run --rm --service-ports sim \
  ros2 launch visbot_control plant.launch.py mission:=skills cpu:=3 poll_idle:=true spin_us:=500
```

Gazebo (headless by default):

```bash
docker compose -f sim/docker/compose.yaml run --rm --service-ports sim \
  ros2 launch visbot_gazebo sim.launch.py mission:=skills cpu:=3 poll_idle:=true spin_us:=500
```

Then open http://localhost:8080. You get the field with the controller's odometry drawn over ground truth, the mission instruction list with how each motion exited, mechanism calls with timestamps, and live loop latency.

Missions:

* `mogo_rush` is `worldsMogoRush()` from `autons.cpp`, transcribed line for line (58 instructions)
* `skills` hits every motion type (drive, turn, swing, odom point, chaining)
* `square` is good for eyeballing odometry drift
* `chain` is back-to-back chained motions

Other launch args: `sched:=fifo|rr|other`, `cpu:=N` (pin the loop to a core), `poll_idle:=true`, `spin_us:=N`, `dash:=false`, `record:=file.jsonl`. Gazebo also takes `gui:=true`, but that needs an X display. On a Mac it's easier to just use the dashboard.

## Writing a mission

Missions are in `core/include/visbot/missions.hpp`. They read like an auton, because the instruction list mirrors how EZ works (set a motion, then block on a wait):

```cpp
Instr::driveSet(36, 127),          // chassis.pid_drive_set(36, 127)
Instr::waitUntil(6),               // chassis.pid_wait_until(12 - 6)
Instr::act(ActionId::DoinkerRight),// rightDoinker.toggle()
Instr::speedMax(70),               // chassis.pid_speed_max_set(70)
Instr::wait(),                     // chassis.pid_wait()
```

Add yours to `byName()` and `names()` in the same file, rebuild, and launch it with `mission:=yourname`.

Rule of thumb: if you're porting a real auton, go line by line and keep the original math in a comment (`// set_drive(32 + 4, ...)`). It makes the two much easier to compare later.

Mechanism calls (intake, clamp, doinkers, ladybrown) get logged with timestamps but not simulated. The timing is real, the effect isn't.

## Tests

Core tests, no ROS needed:

```bash
cmake -S core -B build/core && cmake --build build/core && ctest --test-dir build/core --output-on-failure
```

50 tests covering closed-loop convergence for every motion type, the EZ quirks, kD/kI giving the same result at 100 Hz and 120 Hz, and `mogo_rush` running start to finish inside the field.

Everything, plus the end to end checks:

```bash
docker compose -f sim/docker/compose.yaml run --rm sim bash -lc "colcon test && colcon test-result --all"
docker compose -f sim/docker/compose.yaml run --rm sim src/sysvrc/sim/tools/smoke_test.sh 22 mogo_rush fifo 3 true 500
docker compose -f sim/docker/compose.yaml run --rm sim src/sysvrc/sim/tools/gazebo_smoke.sh 30 square 6.0
```

CI (`.github/workflows/ci.yml`) runs all of this on push to main and on PRs.

## Latency

`rt_bench` runs the real controller + plant tick under different scheduling setups, 10 s each at 120 Hz. These were measured in Docker Desktop on a Mac. The VM halts idle vCPUs, so plain timer wakeups are coarse (~3 ms); what matters is the difference each fix makes.

| setup | p50 | p99 | max | missed deadlines |
|---|---:|---:|---:|---:|
| SCHED_OTHER + 8 CPU hogs | 2720 µs | 6060 µs | 32783 µs | 8 |
| SCHED_FIFO 80 + 8 hogs | 2640 µs | 5160 µs | 7763 µs | 0 |
| FIFO + pinned + poll-idle | 360 µs | 700 µs | 1070 µs | 0 |
| FIFO + pinned + poll-idle + 500 µs spin | 20 µs | 260 µs | 659 µs | 0 |

Full table and plot in [docs/latency.md](docs/latency.md). To rerun:

```bash
docker compose -f sim/docker/compose.yaml run --rm sim src/sysvrc/sim/tools/bench.sh 10 3
docker compose -f sim/docker/compose.yaml run --rm sim python3 src/sysvrc/sim/tools/plot_latency.py
```

## Watching a recorded run

There's a Gazebo run of `mogo_rush` in `docs/replay/`. You can play it back without ROS:

```bash
cp docs/replay/mogo_rush_gazebo.jsonl sim/src/visbot_dash/web/ && (cd sim/src/visbot_dash/web && python3 -m http.server 8080)
# then open http://localhost:8080/?replay=mogo_rush_gazebo.jsonl
```

## Quick troubleshooting

* "Cannot connect to the Docker daemon": Docker Desktop isn't running. Open it and wait a few seconds.
* Controller log says `pthread_setschedparam ... failed`: the container doesn't have CAP_SYS_NICE. The compose file adds it; if you're using plain `docker run`, add `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1`. The loop still runs without it, just not realtime.
* Latency in the milliseconds on a Mac: that's the VM. Use `cpu:=3 poll_idle:=true spin_us:=500`.
* Dashboard won't load: you probably left off `--service-ports`. It needs 8080 (page) and 8081 (websocket).
* Mission never starts: the controller waits for `/visbot/imu` and `/visbot/joint_states` before arming, so check that the backend is up.
* Odometry drifts a lot in Gazebo on `mogo_rush` (about 20 in, vs a few inches on `square`): expected. Encoders can't see wheel slip, and that auton is full speed with hard reversals. More in [docs/fidelity.md](docs/fidelity.md).
