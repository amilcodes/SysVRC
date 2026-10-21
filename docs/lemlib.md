# LemLib

`core/include/visbot/lemlib.hpp` is a port of LemLib v0.5.6's chassis: odometry, the PID and exit conditions, and every auton motion (`moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint`, `swingToHeading`, `swingToPoint`, `follow`). `auton_check`, the report and the ROS sim run LemLib routines through it the same way they run EZ-Template routines through `motion.hpp`.

## Importing

Pass the importer the file that builds `lemlib::Chassis` (usually `main.cpp`) along with the routines:

```bash
python3 tools/auton_import.py src/main.cpp src/autons.cpp --all -o autons
build/core/auton_check autons/solo_awp.blue.auton --html report.html
```

It reads the `Drivetrain`, both `ControllerSettings`, the `OdomSensors` (tracking wheels and IMU) and the drive curves, and writes them into the `.auton` header, so the sim runs your tuning. `follow()` paths are read from `static/` via the `ASSET()` name and copied into the file. `examples/lemlib` is a small project with its imported routines.

If the importer can't find the chassis it falls back to the values from LemLib's example project and says so at the top of the file.

## Timing

On the brain a LemLib auton is three kinds of task: odometry every 10 ms, a task per motion whose loop ends in `pros::delay(10)`, and the auton itself. The port runs all three on a 1 ms clock, in that order within each millisecond. That is what makes these work as they do on the robot:

* An async motion (the default) returns 10 ms after it's called. Code after it runs while the robot moves.
* Calling a motion while another is running blocks until the first one ends.
* `async = false` blocks until the motion ends.
* `waitUntil(d)` and `waitUntilDone()` check every 10 ms.
* When `autonomous()` returns, the last motion keeps running until it exits.

## Behaviour the tests pin

These are what LemLib does, not what a textbook controller would. Each one has a test in `core/tests/test_lemlib.cpp`.

| LemLib behaviour | Effect on a routine |
|---|---|
| `distTraveled` starts at 0, not -1 | `waitUntil` or `waitUntilDone` before the first motion never returns. `auton_check` warns about it. |
| A turn's `distTraveled` is an `angleError` from its start | It never goes above 180, so `waitUntil(200)` on a turn only returns when the turn ends. |
| Both PIDs are built with `signFlipReset = true`, and `sgn(0)` is 1 | The integral is cleared every time the error changes sign. An error of exactly 0 counts as positive. |
| `MotorGroup::move` takes an `int32_t` | Every drive command is truncated to a whole number. Outputs below 1 are 0. |
| Within 7.5 in of the target, `maxSpeed` becomes `max(abs(last output), 60)` | A `moveToPoint` or `moveToPose` with `maxSpeed` under 60 speeds up for the last 7.5 in. |
| `moveToPoint` only checks its exit conditions once it's within 7.5 in | A target the robot can't reach runs to the timeout. |
| With `minSpeed` set, a motion exits when the robot crosses the line through the target (moved back by `earlyExitRange`) | Reported as `passed`. The robot is still moving at `minSpeed` or more. |
| Turns slew only while the error is over 20° | Short turns start at full PID output. |
| `moveToPose` caps lateral speed at `sqrt(horizontalDrift * radius * 9.8)` and gives angular output priority | Tight curves slow down. `horizontalDrift` is a tuning factor, not a physical unit. |
| Without tracking wheels, odometry takes distance from the left drive side only | Heading comes from the IMU; a slipping left side moves the estimate. |
| `follow` counts its timeout in loop iterations and stops at the first closest point with speed 0 | Path files are read up to `endData`; each line must be `x, y, speed`. |
| `cancelMotion()` waits 10 ms | The next line runs 10 ms later, after the motion has stopped. |

## Where the port differs

* All tasks share one deterministic clock. On the brain, tasks of equal priority that wake in the same tick can run in either order.
* Odometry speed (`getSpeed`, `estimatePose`) isn't computed. No motion uses it.
* The locked side of a swing gets a zero command. On the robot it brakes in `HOLD` mode.
* Tracking wheels read the chassis's true motion, with no slip or quantisation of their own.
* A NaN drive command becomes 0. On the brain it's undefined behaviour.
* Not supported: `curvature()`, `resetLocalPosition()`, changing `lateralPID`/`angularPID` mid-routine, and `pros::Task`s in auton code. The importer marks them `NOT IMPORTED`.

## In the ROS sim

`mission:=` takes a LemLib routine like any other. Without `robot:=`, the controller converts commands to wheel speeds with the routine's own `Drivetrain`. The simulated robot is still `robot.json` or the default drivetrain, so pass a `robot:=` that matches your robot. ROS has no ground truth for tracking wheels, so they read the drive encoders and IMU there.
