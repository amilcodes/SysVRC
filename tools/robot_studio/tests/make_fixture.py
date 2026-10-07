#!/usr/bin/env python3
"""Build tests/fixtures/hs_bot.step (and .stl): a made-up but believable High
Stakes robot, named the way the Onshape VEX library names parts.

Needs CadQuery (`pip install cadquery`), which the studio itself doesn't. The
generated files are committed, so this only reruns when the fixture changes.

The robot, so tests can check what the studio finds:
  - 6 x 3.25 in wheels (omni outside, traction middle), 3 per side, track 12 in,
    wheelbase 10.2 in; 6 x 11 W blue (600 rpm) motors geared 36:48 -> 450 rpm
  - front intake: flex wheel roller + hook conveyor, one 11 W blue motor
  - rear goal clamp: two pneumatic cylinders
  - lady brown: two 11 W green (200 rpm) motors, 12T driving 36T (1:3)
  - one doinker (pneumatic), brain, battery, IMU, optical, rotation sensor
Modelled in mm, Z up, front toward +X, and deliberately not centred on the
origin, because real exports never are.
"""
import pathlib

import cadquery as cq

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "fixtures"
OFF = (380.0, -120.0, 15.0)     # where the robot sits in the CAD's world


def at(x, y, z, rx=0, ry=0, rz=0):
    loc = cq.Location((OFF[0] + x, OFF[1] + y, OFF[2] + z))
    for axis, deg in (((1, 0, 0), rx), ((0, 1, 0), ry), ((0, 0, 1), rz)):
        if deg:
            loc = loc * cq.Location((0, 0, 0), axis, deg)
    return loc


def box(dx, dy, dz):
    return cq.Workplane("XY").box(dx, dy, dz)


def disc(dia, width):
    """A cylinder whose axis is Y (so wheels and gears face sideways)."""
    return cq.Workplane("XZ").circle(dia / 2).extrude(width / 2, both=True)


def motor(name, cartridge):
    m = cq.Assembly(name=name)
    m.add(box(70, 36, 57), name="Motor Housing")
    m.add(disc(20, 8), name=cartridge, loc=cq.Location((0, 22, 0)))
    return m


def main():
    a = cq.Assembly(name="HS Bot")
    IN = 25.4

    # ---- drivetrain
    for side, y in (("Left", 1), ("Right", -1)):
        d = cq.Assembly(name=f"Drive {side}")
        for i, x in enumerate((-130, 0, 130)):
            wheel = "3.25in Traction Wheel" if x == 0 else "3.25in Omni Wheel"
            d.add(disc(3.25 * IN, 22), name=f"{wheel} <{i + 1}>", loc=cq.Location((x, y * 152, 3.25 * IN / 2)))
            d.add(disc(2.0 * IN, 6), name=f"48T High Strength Gear <{i + 1}>", loc=cq.Location((x, y * 136, 3.25 * IN / 2)))
        for i, x in enumerate((-65, 65)):
            d.add(motor("V5 Smart Motor (11W)", "Blue Cartridge 600 RPM"), name=f"V5 Smart Motor (11W) <{i + 1}>",
                  loc=cq.Location((x, y * 100, 70)))
            d.add(disc(1.5 * IN, 6), name=f"36T High Strength Gear <{i + 1}>", loc=cq.Location((x, y * 128, 70)))
        d.add(motor("V5 Smart Motor (11W)", "Blue Cartridge 600 RPM"), name="V5 Smart Motor (11W) <3>",
              loc=cq.Location((-170, y * 100, 70)))
        d.add(disc(1.5 * IN, 6), name="36T High Strength Gear <3>", loc=cq.Location((-170, y * 128, 70)))
        d.add(box(381, 25, 51), name="C-Channel 1x2x1x25", loc=cq.Location((0, y * 122, 70)))
        a.add(d, name=f"Drive {side}", loc=at(0, 0, 0))

    # ---- chassis electronics
    a.add(box(25, 330, 25), name="C-Channel 1x1x1x15 (front cross)", loc=at(175, 0, 120))
    a.add(box(110, 70, 20), name="V5 Robot Brain", loc=at(-40, 0, 230))
    a.add(box(140, 60, 40), name="V5 Robot Battery", loc=at(-70, 0, 110))
    a.add(box(30, 30, 15), name="V5 Inertial Sensor", loc=at(0, 40, 140))
    a.add(cq.Workplane("XY").circle(25).extrude(150), name="Pneumatic Air Tank", loc=at(-40, -60, 150, ry=90))

    # ---- front intake: flex wheel roller and a hook conveyor up to a goal at the back
    intake = cq.Assembly(name="Intake")
    for i, y in enumerate((-60, 0, 60)):
        intake.add(disc(2.0 * IN, 20), name=f"2in Flex Wheel <{i + 1}>", loc=cq.Location((215, y, 55)))
    intake.add(motor("V5 Smart Motor (11W)", "Blue Cartridge 600 RPM"), name="V5 Smart Motor (11W) <7>",
               loc=cq.Location((175, 120, 95)))
    intake.add(box(330, 90, 6), name="High Strength Chain", loc=cq.Location((70, 0, 200), (0, 1, 0), -38))
    for i, (x, z) in enumerate(((175, 92), (110, 145), (40, 200), (-30, 255))):
        intake.add(box(20, 30, 25), name=f"Conveyor Hook <{i + 1}>", loc=cq.Location((x, 0, z + 18)))
    intake.add(disc(1.2 * IN, 8), name="6T High Strength Sprocket <1>", loc=cq.Location((195, 0, 75)))
    intake.add(disc(1.2 * IN, 8), name="6T High Strength Sprocket <2>", loc=cq.Location((-60, 0, 300)))
    intake.add(box(40, 30, 25), name="V5 Optical Sensor", loc=cq.Location((-20, 55, 270)))
    a.add(intake, name="Intake", loc=at(0, 0, 0))

    # ---- rear mobile goal clamp
    clamp = cq.Assembly(name="Mogo Clamp")
    clamp.add(box(20, 150, 60), name="Clamp Plate", loc=cq.Location((-195, 0, 80)))
    for i, y in enumerate((-55, 55)):
        clamp.add(cq.Workplane("XY").circle(12).extrude(110), name=f"Pneumatic Cylinder 50mm Stroke <{i + 1}>",
                  loc=cq.Location((-165, y, 90)))
    a.add(clamp, name="Mogo Clamp", loc=at(0, 0, 0))

    # ---- lady brown arm (12T -> 36T, so 1:3) at the top front
    lb = cq.Assembly(name="Lady Brown")
    for i, y in enumerate((-90, 90)):
        lb.add(motor("V5 Smart Motor (11W)", "Green Cartridge 200 RPM"), name=f"V5 Smart Motor (11W) <{8 + i}>",
               loc=cq.Location((60, y, 330)))
        lb.add(disc(0.5 * IN, 6), name=f"12T High Strength Gear <{i + 1}>", loc=cq.Location((60, y * 0.7, 330)))
        lb.add(disc(1.5 * IN, 6), name=f"36T High Strength Gear <{4 + i}>", loc=cq.Location((110, y * 0.7, 350)))
    lb.add(box(150, 12, 30), name="Lady Brown Arm", loc=cq.Location((150, 0, 350)))
    lb.add(box(20, 20, 20), name="V5 Rotation Sensor", loc=cq.Location((110, 0, 350)))
    a.add(lb, name="Lady Brown", loc=at(0, 0, 0))

    # ---- doinker, front left corner
    dk = cq.Assembly(name="Doinker")
    dk.add(box(160, 10, 20), name="Doinker Arm", loc=cq.Location((150, 140, 150), (0, 0, 1), 20))
    dk.add(cq.Workplane("XY").circle(10).extrude(80), name="Pneumatic Cylinder 25mm Stroke", loc=cq.Location((90, 140, 150), (0, 1, 0), 90))
    a.add(dk, name="Doinker", loc=at(0, 0, 0))

    OUT.mkdir(exist_ok=True)
    a.export(str(OUT / "hs_bot.step"))
    cq.exporters.export(a.toCompound(), str(OUT / "hs_bot.stl"), tolerance=0.5, angularTolerance=0.5)
    print("wrote", OUT / "hs_bot.step", "and", OUT / "hs_bot.stl")


if __name__ == "__main__":
    main()
