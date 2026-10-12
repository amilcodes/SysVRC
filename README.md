# SysVRC

Sim testbed for our VRC robot. It runs our real autons, imported from `autons.cpp`, through a port of the EZ-Template drive code the brain runs, hundreds of times with placement, battery, slip and gyro drift varied, and shows what breaks. Plus a ROS 2 + Gazebo sim on a 120 Hz real-time control loop, and a studio that turns a robot's CAD into a sim model.

![auton_check report](docs/img/report.gif)

## Check your autons (no ROS needed)

Import, then check:

```bash
python3 tools/ez_import.py v5/src/autons.cpp v5/src/skills.cpp --all -o autons
cmake -S core -B build/core && cmake --build build/core
build/core/auton_check autons/state_solo_awp_center_get.blue.auton --html report.html
```

No cmake? It's one file: `c++ -std=c++17 -O2 -Icore/include -Icore/tools core/tools/auton_check.cpp -o auton_check`.

The importer handles EZ calls, wrappers like our `set_drive`, `if (isBlue)` branches, timed `drive_set` pushes and helpers in the same file. Anything that depends on the robot at runtime gets marked `NOT IMPORTED` instead of guessed. Red side is `--bind isBlue=false`; other wrappers go in `ALIASES`. 16 of our 18 routines import cleanly.

`auton_check` runs a routine once clean, then a few hundred times varied: does it finish in time, how far does each mechanism call land from where it should, and which disturbance is to blame. The report is the field with every run on it, your code down the side, and whatever's still running at the buzzer marked. If a routine starts at (0, 0), drag the robot to its real start.

What it found in our code:

* `stateSoloAwpCenterGet` is on time in only 81% of runs. The last drive toward the ladder has no wait and it's what's still going at the buzzer, mostly on low batteries (100% on time at full charge, 36% at 85%).
* `safeFourRing` has a `pros::delay(20000)` at line 1232 with code after it that never runs in a match.
* `positiveSideQuals` swings at speed `-100` (lines 1383, 1387). EZ drops the sign, so those were never backwards swings.

All routines, one line each: `for f in autons/*.auton; do build/core/auton_check "$f" --brief; done`. More in [docs/auton_check.md](docs/auton_check.md).

## Your robot in the sim

By default the sim drives a generic 15 in box. The robot studio reads your STEP export (motors, cartridges, wheels, gear teeth, pistons, sensors) and your code (chassis constructor, motor names, helpers), and works out the drivetrain and where each mechanism grabs. With an `ANTHROPIC_API_KEY`, Claude reads renders of the robot plus the code to figure out things like what `ChangeLBState(EXTENDED)` does. Fix what's wrong, save.

![robot studio](docs/img/studio.gif)

```bash
python3 -m pip install -r tools/robot_studio/requirements.txt
python3 tools/robot_studio serve                    # http://localhost:8090
build/core/auton_check autons/state_solo_awp.blue.auton --robot robots/ours/robot.json --html report.html
```

With `--robot`, the report plays out the game: rings picked up, goals clamped or missed (and by how much), lady brown scores, points at the buzzer and the AWP checklist, across every run. path.jerryio paths import with `tools/path_import.py`. More in [docs/robot_studio.md](docs/robot_studio.md).

## Full sim (ROS 2 + Gazebo)

Needs Docker; it all runs in a Linux container, Apple Silicon included. The repo is mounted at `/ws/src/sysvrc`.

```bash
docker compose -f sim/docker/compose.yaml build
sim/tools/build.sh                                   # colcon build inside the container
```

Fake robot, no Gazebo (fast, what CI uses), or Gazebo:

```bash
docker compose -f sim/docker/compose.yaml run --rm --service-ports sim \
  ros2 launch visbot_control plant.launch.py mission:=state_solo_awp.blue cpu:=3 poll_idle:=true spin_us:=500
docker compose -f sim/docker/compose.yaml run --rm --service-ports sim \
  ros2 launch visbot_gazebo sim.launch.py mission:=worlds_mogo_rush.blue cpu:=3 poll_idle:=true spin_us:=500
```

The dashboard at http://localhost:8080 shows the robot, a dashed outline where odometry thinks it is, how each wait exited, the match clock and the loop's wake latency. Add `robot:=robots/<name>` to run a studio robot (its CAD in Gazebo, live scoring on the dashboard).

![dashboard](docs/img/dash.gif)

That's a recorded Gazebo run of our skills routine. Watch it without ROS:

```bash
cp docs/replay/skills_gazebo.jsonl sim/src/visbot_dash/web/ && (cd sim/src/visbot_dash/web && python3 -m http.server 8080)
# then open http://localhost:8080/?replay=skills_gazebo.jsonl
```

The odometry outline falls behind after the corner pushes because the wheels keep counting while the robot is pinned on the wall. It was recorded in Docker on a Mac, so ignore its loop latency.

## What's where

```text
SysVRC/
  core/    the drive controllers (header-only C++, no deps) + auton_check
  autons/  our routines as .auton files, generated from v5/src by the importer
  robots/  robot.json for each robot the sim knows
  tools/   ez_import.py, path_import.py, build_web.py, robot_studio/
  web/     the report page and field drawing
  sim/     ROS 2 workspace: control loop, Gazebo model + field, dashboard
  v5/      our PROS competition code
  docs/    how it works, what matches EZ, latency
```

* `core/` is EZ-Template v3.2.2 checked line by line against EZ's source, quirks included, because that's what the robot does ([docs/fidelity.md](docs/fidelity.md)). Gains are copied from `default_constants()` into `constants.hpp`, so retune both.
* `autons/` is generated but committed so you can read a routine without the C++. Each line keeps its source line (`@1977`). CI fails if it drifts from `v5/src`.
* `sim/`'s `controller_node` runs the loop on its own SCHED_FIFO thread, not a ROS timer: absolute-deadline sleeps, locked memory, lock-free handoff to ROS, every tick's latency logged ([docs/architecture.md](docs/architecture.md)).
* `v5/` is missing `include/` and `project.pros`, so `pros make` won't work from a fresh clone.

## Tests

```bash
cmake -S core -B build/core && cmake --build build/core && ctest --test-dir build/core --output-on-failure
python3 -m unittest discover -s tools/tests
python3 -m unittest discover -s tools/robot_studio/tests
node --test web/tests/*.test.js
docker compose -f sim/docker/compose.yaml run --rm sim bash -lc "colcon test && colcon test-result --all"
docker compose -f sim/docker/compose.yaml run --rm sim src/sysvrc/sim/tools/smoke_test.sh 24 state_solo_awp.blue fifo 3 true 500
docker compose -f sim/docker/compose.yaml run --rm sim src/sysvrc/sim/tools/gazebo_smoke.sh 30 square 6.0
```

76 core, 25 tool, 26 studio and 9 game-sim tests, plus end to end over ROS and in Gazebo. CI runs all of it on every push.

## Latency

Wake latency of the 120 Hz loop under different scheduling, from `rt_bench`. Measured in Docker Desktop on a Mac, where the VM's ~3 ms timer wakeups dominate, so read it for what each fix changes:

| setup | p50 | p99 | max | missed deadlines |
|---|---:|---:|---:|---:|
| SCHED_OTHER + 8 CPU hogs | 2720 µs | 6060 µs | 32783 µs | 8 |
| SCHED_FIFO 80 + 8 hogs | 2640 µs | 5160 µs | 7763 µs | 0 |
| FIFO + pinned + poll-idle | 360 µs | 700 µs | 1070 µs | 0 |
| FIFO + pinned + poll-idle + 500 µs spin | 20 µs | 260 µs | 659 µs | 0 |

Rerun with `sim/tools/bench.sh 10 3` in the container. Full table in [docs/latency.md](docs/latency.md).

## When it breaks

* "pushes with drive_set" warning: the routine starts at (0, 0) so the sim doesn't know where the walls are. Drag the robot to its start in the report and rerun with the `--field-start X,Y,HEADING` it gives you.
* `NOT IMPORTED` in a routine: that line needed the robot at runtime. Don't trust results after it.
* "Cannot connect to the Docker daemon": start Docker Desktop.
* `pthread_setschedparam ... failed`: the container needs `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1` (compose adds these). It still runs, just not realtime.
* Millisecond latency on a Mac: that's the VM. Use `cpu:=3 poll_idle:=true spin_us:=500`.
* Dashboard won't load: add `--service-ports` (8080 and 8081).
* Mission never starts: the controller waits for `/visbot/imu` and `/visbot/joint_states`. "no routine called ..." means the name isn't in `autons/`.
