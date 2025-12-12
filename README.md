# SysVRC

Competition code for our VRC robot.
Runs on PROS with EZ-Template + LemLib for chassis control and odometry.
Everything you need to build and flash is in this repo.

## Layout

```text
SysVRC/
  bin/                   # build output from PROS
  firmware/              # firmware / project metadata
  src/
    autons.cpp           # match autonomous routines
    main.cpp             # init, auton, opcontrol
    selection.cpp        # auton selector + LCD
    skills.cpp           # skills run logic
    subsystemFiles/      # intake, lift, clamp, sensors, drivebase, etc.
  static/                # static assets

  EZ-Template.a          # prebuilt EZ-Template
  EZ-Template@3.2.2.zip  # original archive
  LemLib.a               # prebuilt LemLib
  Makefile
  common.mk
```

## What lives where

### `src/main.cpp`

Startup and high level control:

* sets brake modes, IMU, odom, and chassis constants
* spins up background tasks (logger, temp checks, LB loop, color loop)
* wires up the auton selector
* defines `autonomous()` and `opcontrol()` entry points

This is the first place to look if something is weird on boot.

### `src/autons.cpp`

All the actual match autos.

Common pattern:

* reset sensors and odom
* set starting pose
* call drive helpers / mechanism helpers in order
* many functions take `bool isBlue` so the same code works on both sides

There are a bunch of routines here (rush, safe, disrupt, skills-style autos).
Comment out / swap the call in `autonomous()` to change what runs.

### `src/skills.cpp`

A cleaner, isolated version of the skills run:

* uses the same chassis + odom config
* has its own scoring route and motion sequence
* easy to test without touching match autos

If you only care about skills, this file is the one to read.

### `src/subsystemFiles/`

Subsystem logic is broken out to keep autos readable:

* intake control and “unstuck” logic
* ladybrown state machine (REST / PROPPED / EXTENDED / etc.)
* mogo clamp and side doinkers
* drive helpers (`set_drive`, brake helpers, etc.)
* sensor utilities (optical color filter, alignment sensors, temp checks)

Rule of thumb: if it talks to one piece of hardware, it probably lives here.

### `static/`, `bin/`, `firmware/`

Mostly PROS stuff.
You should not need to touch these unless you are doing something weird with the project itself.

## Building and uploading

You need the PROS CLI installed.

Build:

```bash
pros make
```

Upload to the brain:

```bash
pros upload
```

If the build is acting up:

```bash
pros make clean
pros upload
```

## Tuning notes

* Chassis PID and slew settings are configured in `default_constants()` in `autons.cpp`.
* IMU and odom starting pose are set in `autonomous()` and some auton functions.
* If you move the IMU or tracking wheels, fix the offsets there first before touching the gains.

## Adding new code

Guidelines that keep this project sane:

* Put new mechanism logic in `subsystemFiles/`, not inside autos.
* Keep autos as sequences of “do X, wait, do Y” so they can be read top to bottom.
* Avoid random `pros::delay()` if a sensor check would work instead.
* Always reset odom and sensors at the start of a new auton so tests are repeatable.

## Quick troubleshooting

* Robot spins or drives in a curve on straight moves

  * check IMU calibration and tracking wheel directions
* Auton selector does nothing

  * look at `selection.cpp` and the call site in `initialize()`
* Motors brown out or quit mid match

  * watch the temp output from the temp task and maybe back off current / speed

That is basically it. If you are reading this because you are about to change autos, start with `autons.cpp` and work down from there.
