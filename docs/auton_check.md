# auton_check

"Same code, different result every run" is probably the most common auton complaint on the VEX forum. The usual suspects in those threads are always the same: where the robot got placed, battery charge, gyro drift, wheel slip, and a drivetrain that doesn't quite drive straight. The usual fix is also the same: run it on the field a lot and hope you notice the pattern.

`auton_check` does the "run it a lot" part without a field. It takes a routine, runs it once with nothing disturbed, then runs it a few hundred more times with a random draw of those things. It reports:

* does it finish inside the auton period, and how often
* where each mechanism call actually happens, compared with the clean run (a clamp that fires 4 in from where it fired in testing probably misses the goal)
* which disturbance is responsible, one factor at a time
* which motions sometimes give up on a velocity or timeout exit instead of reaching their target

It's the Monte Carlo robustness check you'd do on any control system before trusting it on hardware, pointed at a VRC auton.

## Using it

```bash
cmake -S core -B build/core && cmake --build build/core
build/core/auton_check autons/state_solo_awp.blue.auton
build/core/auton_check autons/state_solo_awp.blue.auton --html report.html   # open in a browser
```

Checking every routine at once, one line each:

```bash
for f in autons/*.auton; do build/core/auton_check "$f" --brief; done
```

Skills routines get a 60 s limit automatically (anything with "skills" in the name). Otherwise it's 15 s; change it with `--budget`.

Comparing gains: `--set KEY=VALUE` overrides a constant for that run, so you can check whether a retune makes a routine more or less repeatable before flashing anything:

```bash
build/core/auton_check autons/ring_rush6.blue.auton --brief
build/core/auton_check autons/ring_rush6.blue.auton --brief --set turn.kd=30 --set drive.kp=14
```

Keys are `drive|heading|turn|swing|odom_angular` + `.kp|.ki|.kd|.start_i`, plus `odom_turn_bias`, `odom_look_ahead`, `slew_drive.distance`, `slew_drive.min_speed`. Same `--seed` gives the same report, so the comparison is fair.

## What gets varied

| disturbance | default | flag | what it stands for |
|---|---|---|---|
| placement x/y | 0.5 in, 1σ | `--place-xy` | lining the robot up by hand with a line-up tool |
| placement heading | 1.0°, 1σ | `--place-deg` | same, rotation |
| battery | 0.85 to 1.00 | `--battery` | achievable speed vs a full battery |
| L/R mismatch | 0.02, 1σ | `--mismatch` | one side of the drive a little stronger than the other |
| wheel slip | 0 to 3% | `--slip` | travel lost to the tiles |
| gyro drift | 0.02 °/s, 1σ | `--drift` | IMU heading creep |
| motor response | 0.10 to 0.14 s | `--tau` | how fast the drive gets up to speed |

These are assumptions, not measurements of your robot. They're a reasonable starting point for a team that uses a line-up tool and charges between matches. If you don't use a line-up tool, try `--place-xy 1.5 --place-deg 3`. If you want to know how bad a flat battery is, try `--battery 0.7:0.8`. `--ideal` turns everything off, which is a good sanity check (you should get 100% on time and ~0 spread).

## Reading the output

Here's `stateSoloAwpCenterGet`:

```
time    nominal 13.65 s   p50 14.47 s   p95 15.44 s   worst 15.87 s   (limit 15 s)
        finishes in time in 81.0% of runs
        when time ran out it was usually on: the last motion, pid_drive_set at line 1566 (nothing waits for it)
```

In a clean run it finishes with 1.3 s to spare. That margin is smaller than the run-to-run variation, so about one run in five the final drive toward the ladder is still going at the buzzer.

The tool ranks the disturbances two ways, because the answers are different:

```
what moves the end point most (p95, one factor at a time):
  wheel slip            1.75 in
  placement (heading)   1.66 in
  motor response        1.44 in
  placement (x/y)       1.16 in
  battery               0.57 in
  ...

what makes it late (p95 finish time, one factor at a time; nominal 13.65 s):
  battery              +1.42 s   in time  92.0%
  motor response       +0.64 s   in time 100.0%
  L/R mismatch         +0.04 s   in time 100.0%
  ...
```

Each row varies one thing and holds the rest perfect. For this routine, where the robot ends up depends on slip and on how it was placed, so a better line-up tool helps accuracy. Whether it *finishes* depends almost entirely on the battery: with every battery full it's on time in 100% of runs, and at 85% charge in 36%. So charge between matches, or buy back time elsewhere in the route. Tuning the last drive wouldn't help.

"Least repeatable mechanism calls" is sorted by the p95 distance between where the call fired in a disturbed run and where it fired in the clean run. The HTML report draws every run's position at that moment on the field plot. Click a row in the table to switch which call is shown.

## Limits

Worth knowing before you trust a number:

* **Drivetrain only.** Intakes, clamps and lifts are logged with timestamps, not simulated. The sim can tell you the clamp fired 4 in from where it usually does. It can't tell you whether the goal got clamped.
* **No field elements.** The robot drives through goals and rings as if they weren't there.
* **Walls only with a real start position.** Most routines use `odom_xyt_set(0, 0, heading)`, which says nothing about where on the field the robot is. Routines that shove into a wall with `drive_set` get a warning, and everything after the shove means nothing until you give a field start: `--field-start X,Y,HEADING`, or `# @field_start X Y HEADING` at the top of the .auton file. Field coordinates have the origin at the centre, +y toward the far wall, in inches. If your `odom_xyt_set` already uses field coordinates, copy the same numbers.
* **Kinematic drivetrain model.** Speed follows the command with a first-order lag; there's no motor torque curve, current limit, or VEXos 2.5 A to 2 A current drop. Short motions are the most trustworthy.
* **The plant isn't fit to your robot yet.** Top speed comes from the geometry in `constants.hpp` (3.25 in wheels at 450 rpm), and the response time is the default above. If the sim's timings don't line up with your robot's, those two numbers are the first thing to check.
* **What the importer skipped isn't there.** If a routine has `NOT IMPORTED` lines (sensor-dependent `if`s, live-pose math), the sim runs without them.
