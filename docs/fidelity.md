# Fidelity to EZ-Template

The testbed is only worth anything if the controller it runs is the controller
the robot runs. `core/` is therefore a *port* of EZ-Template v3.2.2, not a
reimplementation of the same ideas — where EZ has a quirk, `core/` has the
same quirk.

This page records what was checked against EZ's source, what matches, and what
doesn't. Verified against
[EZ-Template v3.2.2](https://github.com/EZ-Robotics/EZ-Template/tree/v3.2.2):
`src/EZ-Template/PID.cpp`, `slew.cpp`, `util.cpp`, `drive/pid_tasks.cpp`,
`drive/exit_conditions.cpp`, `drive/set_pid/*.cpp`.

## Quirks deliberately preserved

These all look like bugs. They are what runs on the robot, so they are what
runs here. Each has a test that pins it, so a future "cleanup" fails loudly
instead of silently changing how the robot drives.

| # | Behaviour | Textbook version | Pinned by |
|---|---|---|---|
| 1 | **Derivative is taken on the measurement, not the error** (`derivative = cur - prev_current`, then *subtracted*). Moving the target produces no derivative kick. | `d(error)/dt` | `derivative_is_on_measurement_not_error` |
| 2 | **The integral resets when `sgn(error) != sgn(prev_current)`** — the previous *measurement*, not the previous error. When the measurement sits on the opposite side of zero from the error, this is true every tick and `kI` is dead for that motion. | compare against `prev_error` | `ez_quirk_integral_resets_against_previous_measurement` |
| 3 | **The integral accumulates first and resets second**, so the resetting tick's own contribution is discarded too. | reset, then accumulate | same test |
| 4 | **`small_error` and `big_error` are `if / else if`.** Every motion in `autons.cpp` sets `small_error`, so `BIG_EXIT` is unreachable on this robot. | both conditions active | `ez_quirk_big_error_is_dead_when_small_is_set` |
| 5 | **The velocity exit tests `\|derivative\| <= 0.05` alone** — there is no "and still far from the target" guard, so it can fire while settling. | gate on remaining error | `velocity_exit_fires_when_measurement_stops` |
| 6 | **Slew disables itself when the requested max speed is below `min_speed`**, rather than ramping to a cap lower than its own floor. | clamp `min` to `max` | `ez_quirk_disabled_when_max_speed_below_min` |
| 7 | **The sign of a speed is thrown away.** `pid_speed_max_set` does `abs(clamp(speed, 127, -127))`, so `pid_swing_set(..., -100)` is exactly `+100`. Direction only ever comes from the target. `positiveSideQuals` has two swings at `-100` (lines 1383 and 1387) that never swung backwards. | negative speed reverses | `negative_speed_is_the_same_as_positive` |
| 8 | **Slew collapses if the robot is moving the wrong way when a motion starts.** The ramp is a line in error space; roll away from it and the line extrapolates past `min_speed` toward zero. A slewed reverse drive issued while still rolling forward barely moves. Found by the sim after a timed `drive_set` push; on the field the wall usually stops the robot first. | ramp from current speed | `ez_quirk_slew_collapses_if_moving_the_wrong_way` |

## Semantics that a naive port gets wrong

These are not quirks — they are load-bearing details that produce a
plausible-looking but wrong simulation if you skip them. Each was a real bug
in the first version of this port.

**Heading is continuous, never wrapped.** `drive_imu_get()` returns cumulative
rotation. A turn from 170° to −170° is issued as a target of *190*, so the
error is 20°. Wrapping the measurement to ±180 turns that into 370° the
instant the robot crosses 180 and it spins forever. `Pose::theta` is therefore
continuous; `wrappedTheta()` exists for display and bearing comparisons only.

**Outputs are vector-scaled, not clamped.** When the faster side exceeds the
speed cap, EZ scales *both* sides by `cap / faster`. Clamping each side
independently flattens a 300/150 request into 127/127 — the robot drives
straight when it was asking to curve. Done twice per tick in the drive task:
once for the drive PIDs, once after the heading term is mixed in.

**Exit results are latched per side.** `PID::exit_condition` resets its own
timers when it succeeds. Re-polling a side that has already exited restarts
its clock, so on a two-PID motion the sides starve each other and the wait
never returns. EZ latches (`left_exit = left_exit != RUNNING ? left_exit : …`)
and so does this.

**`pid_wait_until` means a heading during a turn.** With a drive running it's
a distance; with a turn or swing running it's an absolute heading. The
first version always treated it as a distance. `worldsMogoRush` depends on
the heading meaning (`pid_turn_set(90 * sgn); pid_wait_until(2)`).

**The last motion keeps running after the routine returns.** A routine that
ends with `set_drive(-30);` and no wait still drives those 30 inches: the
auton function returns, but EZ's PID task keeps going. The first version
stopped the robot on the spot, which made `stateSoloAwpCenterGet` look like it
always finished in time. It doesn't (see below).

**`pid_wait_quick_chain` waits on the *original* target.** It extends the PID
target by the chain constant, then calls `pid_wait_quick`, which waits only
until the robot passes `chain_target_start`. The loop is still pulling toward
the extended target when the wait returns — that is the whole mechanism. Waiting
on the extended target's exit conditions instead just moves the stopping point
and the robot still comes to a halt. Measured at the hand-off between two
24 in drives: **21.0 in/s chained vs 6.6 in/s unchained**, same total distance.

## Rate independence

EZ ticks at `util::DELAY_TIME = 10 ms`. Its `kD` is a raw per-tick delta (not
divided by `dt`) and its integral is a raw per-tick sum, so both are implicitly
scaled by the loop rate. To run the same numbers in a 120 Hz loop, `Pid`
rescales the derivative to a 10 ms tick and accumulates the integral per
10 ms-equivalent. Pass the real loop period as `dtSeconds` and the controller
behaves as if it were the 100 Hz brain loop — pinned by
`derivative_is_rate_invariant` and `integral_is_rate_invariant`, which run the
same physical ramp at both rates and require identical output.

## Known divergences

Listed so nobody has to discover them by being surprised.

| Area | Divergence |
|---|---|
| **Odometry point-to-point** | EZ integrates a synthetic "current" from its own per-tick range change (`xy_delta_fake`). This rebuilds the same quantity from the measured change in range to the target: equal to within odometry noise, not bit-identical. |
| **Boomerang / pure pursuit** | Not implemented. `pid_odom_set` with a carrot point and `odom_boomerang_*` constants have no equivalent here; `DriveGains::boomerang` exists only so the tuned numbers aren't lost. `autons.cpp` uses `pid_odom_set` 4 times, all in `skills.cpp`. |
| **Motor current (mA) exits** | Not modelled — the plant has no current draw, so `mA_timeout` is inert. EZ uses it to detect pushing matches. |
| **Tracking wheels** | This robot runs without them (`globals.cpp` has both trackers commented out), so odometry is encoder + IMU only, matching the real configuration. `ez::tracking_wheel` is not ported. |
| **`interfered` / mA-driven re-runs** | `interfered` is reported but no auton here branches on it. |
| **Timeouts** | `Instr::timeoutMs` is a backstop for headless runs and has no EZ equivalent. It is reported as its own exit reason so it can never be mistaken for robot behaviour. |
| **Mechanism subsystems** | Intake, ladybrown, clamp and colour sort are recorded as timestamped action events (the original call text), not simulated. Their *timing* is preserved; their *effects* are not. |
| **After the routine returns** | EZ holds the last motion until the auton period ends. The sim holds it until it settles, or 5 s, so headless runs terminate. |
| **Turn to a point, reversed** | EZ faces a point in reverse through `find_point_to_face`; this adds 180° to the bearing. Same heading. |
| **Walls** | Only modelled when a routine has a real field start (`# @field_start`). The chassis stops at the wall and the encoders keep counting, which is what makes odometry wrong after a wall push. No rotation from wall contact. |
| **Field elements** | None. The robot drives through goals and rings. |

## Odometry drift against Gazebo

Encoder + IMU dead reckoning cannot see wheel slip. Comparing the
controller's estimate against Gazebo's ground truth at the end of a run:

| routine | drift vs truth |
|---|---:|
| `square` | 0.75 in |
| `worlds_mogo_rush.blue` (full routine, imported) | 2.8 in |

Correction: an earlier version of this page said `mogo_rush` drifted 20.1 in.
That was measured on a hand-copied version of the routine that stopped before
the corner push, running on the controller before the fixes above (wrapped
heading, both exit branches live, and so on). It wasn't a property of the
real routine, and the number is gone.

Single Gazebo runs vary by an inch or two between runs. For spread across many
runs, use `auton_check` (see [auton_check.md](auton_check.md)).
`sim/tools/gazebo_smoke.sh` takes the tolerated drift as its third argument;
CI runs `square` at 6 in.

## What this buys

`tools/ez_import.py` reads `autons.cpp` directly, so the sim runs the
routines as written instead of a hand copy. That matters: the one hand copy
this repo used to have was missing its last third, and got a speed wrong
because `set_drive`'s third argument is `minSpeed`, not speed. Of the 18
routines in `v5/src`, 16 import with nothing left out; the other two branch on
live sensor readings, which the importer reports line by line.

The motion engine mirrors EZ's two-thread structure (a motion is *set*, then
the routine blocks in a `pid_wait*` while the PID task keeps running), so each
imported routine runs to completion unmodified.
