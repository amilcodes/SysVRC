// Closed-loop tests: MotionController driving the kinematic plant at 120 Hz.
#include <string>

#include "check.hpp"
#include "visbot/visbot.hpp"

using namespace visbot;

namespace {

struct Rig {
    RobotParams rp;
    DiffDrivePlant plant;
    MotionController ctrl;
    double t = 0;
    const double dt = 1.0 / 120.0;
    double peakCmd = 0;

    explicit Rig(PlantParams pp = {}) : plant(rp, pp), ctrl(rp) {
        plant.reset({0, 0, 0});
        ctrl.resetPose({0, 0, 0}, plant.sensors());
    }

    void setPose(const Pose& p) {
        plant.reset(p);
        ctrl.resetPose(p, plant.sensors());
    }

    double run(double maxSec) {
        while (!ctrl.status().done && t < maxSec) {
            const WheelCmd c = ctrl.tick(plant.sensors(), dt);
            peakCmd = std::fmax(peakCmd, std::fmax(std::fabs(c.left), std::fabs(c.right)));
            plant.step(c, dt);
            t += dt;
        }
        return t;
    }

    /// Run until the program counter reaches `pc`, then stop.
    bool runToPc(int pc, double maxSec) {
        while (!ctrl.status().done && t < maxSec) {
            if (ctrl.status().pc >= pc) return true;
            plant.step(ctrl.tick(plant.sensors(), dt), dt);
            t += dt;
        }
        return false;
    }
};

}  // namespace

TEST(tick_before_mission_is_safe) {
    Rig r;
    for (int i = 0; i < 50; ++i) {
        const WheelCmd c = r.ctrl.tick(r.plant.sensors(), r.dt);
        EXPECT_NEAR(c.left, 0, 1e-12);
        EXPECT_NEAR(c.right, 0, 1e-12);
    }
    EXPECT_TRUE(!r.ctrl.status().done);
}

TEST(drive_24in_converges) {
    Rig r;
    r.ctrl.setMission({Instr::driveSet(24, 110), Instr::wait()});
    r.run(5);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().y, 24.0, 1.0);
    EXPECT_NEAR(r.plant.truth().x, 0.0, 0.5);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::SmallError);
    EXPECT_TRUE(!r.ctrl.status().interfered);
    EXPECT_TRUE(r.t < 2.5);
}

TEST(drive_reverse_converges) {
    Rig r;
    r.ctrl.setMission({Instr::driveSet(-18, 110), Instr::wait()});
    r.run(5);
    EXPECT_NEAR(r.plant.truth().y, -18.0, 1.0);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::SmallError);
}

TEST(slew_limits_the_launch) {
    // With slew on, the first tick must be capped near min_speed (70), not
    // the full 127 the P term alone would ask for on a 24 in error.
    Rig r;
    r.ctrl.setMission({Instr::driveSet(24, 127, /*slew=*/true), Instr::wait()});
    const WheelCmd first = r.ctrl.tick(r.plant.sensors(), r.dt);
    EXPECT_TRUE(std::fabs(first.left) <= 70.5);
    EXPECT_TRUE(std::fabs(first.left) > 60.0);
}

TEST(no_slew_launches_at_full_speed) {
    // The faster side saturates immediately (16 * 24 in of error). The other
    // side sits just under it because the heading PID is already correcting
    // the IMU noise, which is what vector scaling is supposed to preserve.
    Rig r;
    r.ctrl.setMission({Instr::driveSet(24, 127, /*slew=*/false), Instr::wait()});
    const WheelCmd first = r.ctrl.tick(r.plant.sensors(), r.dt);
    EXPECT_NEAR(std::fmax(std::fabs(first.left), std::fabs(first.right)), 127.0, 1e-9);
}

TEST(turn_90_converges) {
    Rig r;
    r.ctrl.setMission({Instr::turnSet(90, 90), Instr::wait()});
    r.run(4);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().theta, 90.0, 3.0);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::SmallError);
}

TEST(turn_takes_shortest_path) {
    Rig r;
    r.setPose({0, 0, 170});
    r.ctrl.setMission({Instr::turnSet(-170, 90), Instr::wait()});
    r.run(4);
    EXPECT_NEAR(std::fabs(wrapDeg(r.plant.truth().theta - (-170))), 0.0, 3.0);
    EXPECT_TRUE(r.t < 1.5);  // 20 degrees the short way, not 340
}

TEST(turn_longest_goes_the_other_way) {
    Rig r;
    r.setPose({0, 0, 0});
    r.ctrl.setMission({Instr::turnSet(90, 110, AngleBehavior::Longest), Instr::wait(6000)});
    r.run(8);
    // -270 and +90 are the same heading; the robot must have gone negative.
    EXPECT_NEAR(std::fabs(wrapDeg(r.plant.truth().theta - 90.0)), 0.0, 4.0);
    EXPECT_TRUE(r.plant.truth().theta < 95.0);
}

TEST(swing_turn_pivots_on_one_side) {
    Rig r;
    r.ctrl.setMission({Instr::swingSet(SwingSide::Left, 60, 90), Instr::wait(4000)});
    r.run(6);
    EXPECT_NEAR(r.plant.truth().theta, 60.0, 5.0);
    // A swing translates as it rotates; a point turn would not.
    EXPECT_TRUE(r.plant.truth().distanceTo({0, 0, 0}) > 1.0);
}

TEST(drive_to_point_reaches_target) {
    Rig r;
    r.ctrl.setMission({Instr::odomSet(24, 24, 100), Instr::wait(8000)});
    r.run(10);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().x, 24.0, 2.5);
    EXPECT_NEAR(r.plant.truth().y, 24.0, 2.5);
    EXPECT_TRUE(!r.ctrl.status().interfered);
}

TEST(drive_to_point_reverse) {
    Rig r;
    r.ctrl.setMission({Instr::odomSet(0, -24, 90, DriveDirection::Reverse), Instr::wait(8000)});
    r.run(10);
    EXPECT_NEAR(r.plant.truth().y, -24.0, 2.5);
    EXPECT_NEAR(std::fabs(wrapDeg(r.plant.truth().theta)), 0.0, 12.0);  // still facing forward
}

TEST(wait_until_fires_partway_through) {
    Rig r;
    r.ctrl.setMission({
        Instr::driveSet(36, 110),
        Instr::waitUntil(12),
        Instr::act("mogoClamp.toggle()"),
        Instr::wait(),
    });
    // The action must fire while the robot is still short of the full 36 in.
    EXPECT_TRUE(r.runToPc(2, 5));
    const double yAtAction = r.plant.truth().y;
    EXPECT_TRUE(yAtAction > 10.0);
    EXPECT_TRUE(yAtAction < 20.0);
    r.run(6);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().y, 36.0, 1.5);
    EXPECT_TRUE(std::string(r.ctrl.status().lastAction) == "mogoClamp.toggle()");
    EXPECT_TRUE(r.ctrl.status().actionCount == 1);
}

TEST(wait_until_does_not_hang_if_target_never_reached) {
    // pid_wait_until(100) on a 12 in motion must fall through on the
    // motion's own exit conditions rather than blocking forever.
    Rig r;
    r.ctrl.setMission({Instr::driveSet(12, 110), Instr::waitUntil(100, 3000), Instr::wait()});
    r.run(8);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().y, 12.0, 1.5);
}

TEST(speed_max_mid_motion_slows_the_robot) {
    Rig fast, slow;
    fast.ctrl.setMission({Instr::driveSet(40, 127, false), Instr::waitUntil(10), Instr::wait()});
    slow.ctrl.setMission({Instr::driveSet(40, 127, false), Instr::waitUntil(10),
                          Instr::speedMax(40), Instr::wait()});
    fast.run(8);
    slow.run(8);
    EXPECT_TRUE(slow.t > fast.t * 1.15);  // capping the back half costs time
    EXPECT_NEAR(slow.plant.truth().y, 40.0, 1.5);
}

TEST(quick_chain_does_not_stop_at_the_handoff) {
    // The point of pid_wait_quick_chain is that the robot never settles
    // between the two motions. It is NOT that the run is shorter in
    // wall-clock terms: chaining also pushes the target out by the chain
    // constant, so the robot covers 24 + 6 + 24 in instead of 24 + 24.
    // The observable is the speed trough at the hand-off.
    auto lowestMidRunSpeed = [](Rig& r) {
        double lowest = 1e9;
        bool moving = false;
        while (!r.ctrl.status().done && r.t < 15) {
            r.plant.step(r.ctrl.tick(r.plant.sensors(), r.dt), r.dt);
            r.t += r.dt;
            const double sp = 0.5 * (r.plant.leftSpeed() + r.plant.rightSpeed());
            if (sp > 20.0) moving = true;                 // got going
            if (moving && r.plant.truth().y < 40.0)       // before the final stop
                lowest = std::fmin(lowest, sp);
        }
        return lowest;
    };

    Rig chained;
    chained.ctrl.setMission({
        Instr::driveChainConstant(6),
        Instr::driveSet(24, 127), Instr::waitQuickChain(),
        Instr::driveSet(24, 127), Instr::wait(),
    });
    const double chainedLow = lowestMidRunSpeed(chained);

    Rig settled;
    settled.ctrl.setMission({
        Instr::driveSet(24, 127), Instr::wait(),
        Instr::driveSet(24, 127), Instr::wait(),
    });
    const double settledLow = lowestMidRunSpeed(settled);

    EXPECT_TRUE(chained.ctrl.status().done);
    EXPECT_TRUE(settled.ctrl.status().done);
    EXPECT_TRUE(chainedLow > settledLow * 2.0);   // never gives up its speed
    EXPECT_TRUE(settledLow < 15.0);               // the un-chained run does

    // Both cover the same ground: the chain constant changes how hard the
    // loop pulls through the hand-off, not where the robot ends up, because
    // the wait releases at the original target.
    EXPECT_NEAR(chained.plant.truth().y, 48.0, 2.0);
    EXPECT_NEAR(settled.plant.truth().y, 48.0, 2.0);
    EXPECT_TRUE(chained.t < settled.t);  // same distance, no stop in the middle
    std::printf("  handoff speed: chained %.1f in/s vs settled %.1f in/s (%.2fs vs %.2fs)\n",
                chainedLow, settledLow, chained.t, settled.t);
}

TEST(heading_correction_holds_a_straight_line) {
    // Start 6 degrees off and drive: the heading PID must pull it back.
    Rig r;
    r.setPose({0, 0, 6});
    r.ctrl.setMission({Instr::turnSet(0, 90), Instr::wait(), Instr::driveSet(36, 110), Instr::wait()});
    r.run(8);
    EXPECT_NEAR(r.plant.truth().x, 0.0, 2.0);
    EXPECT_NEAR(r.plant.truth().theta, 0.0, 3.0);
}

TEST(vector_scaling_preserves_ratio) {
    // EZ scales both sides together when the faster one exceeds the cap, so
    // the L/R ratio — and therefore path curvature — survives saturation.
    // Clamping each side independently would flatten 300/150 to 127/127 and
    // drive straight when the robot was asking to curve.
    double l = 300, r = 150;
    MotionController::vectorScale(l, r, 127.0);
    EXPECT_NEAR(l, 127.0, 1e-9);
    EXPECT_NEAR(r, 63.5, 1e-9);
    EXPECT_NEAR(l / r, 2.0, 1e-9);

    // Counter-rotation survives too.
    l = 364.5; r = -110.5;
    MotionController::vectorScale(l, r, 127.0);
    EXPECT_NEAR(l, 127.0, 1e-9);
    EXPECT_TRUE(r < 0);
    EXPECT_NEAR(l / r, 364.5 / -110.5, 1e-9);

    // Below the cap nothing is touched.
    l = 50; r = -20;
    MotionController::vectorScale(l, r, 127.0);
    EXPECT_NEAR(l, 50.0, 1e-9);
    EXPECT_NEAR(r, -20.0, 1e-9);
}

TEST(heading_correction_survives_saturation) {
    // A drive issued straight after a turn inherits that turn's heading
    // target (ez: pid_turn_set updates headingPID). Here the robot is at 0
    // with a 90 degree heading target, so the correction is enormous and the
    // output must still be a hard curve, not two clamped-equal sides.
    Rig r;
    r.ctrl.setMission({Instr::turnSet(90, 90), Instr::driveSet(48, 127, false), Instr::wait(6000)});
    const WheelCmd c = r.ctrl.tick(r.plant.sensors(), r.dt);
    EXPECT_NEAR(std::fmax(std::fabs(c.left), std::fabs(c.right)), 127.0, 1e-6);
    EXPECT_TRUE(std::fabs(c.left - c.right) > 100.0);
}

TEST(interfered_is_set_when_stalled) {
    PlantParams pp;
    pp.motorTauSec = 1e9;  // motors never move
    Rig r(pp);
    r.ctrl.setMission({Instr::driveSet(24, 110), Instr::wait(2000)});
    r.run(4);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_TRUE(r.ctrl.status().interfered);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::Velocity ||
                r.ctrl.status().lastExit == ExitReason::Timeout);
    EXPECT_TRUE(r.t < 1.5);
}

TEST(skills_loop_completes) {
    Rig r;
    r.ctrl.setMission(missions::skillsLoop());
    r.run(45);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().x, 0.0, 4.0);
    EXPECT_NEAR(r.plant.truth().y, 0.0, 4.0);
    EXPECT_NEAR(r.ctrl.pose().x, r.plant.truth().x, 2.5);  // odom vs truth
    EXPECT_NEAR(r.ctrl.pose().y, r.plant.truth().y, 2.5);
    std::printf("  skills: %.2f s, odom error %.2f in\n", r.t,
                r.ctrl.pose().distanceTo(r.plant.truth()));
}

TEST(wait_until_during_a_turn_is_a_heading) {
    // ez::pid_wait_until means an absolute heading while turning, not a
    // distance. worldsMogoRush relies on this: pid_turn_set(90); pid_wait_until(2).
    Rig r;
    r.ctrl.setMission({
        Instr::turnSet(90, 90),
        Instr::waitUntil(45),
        Instr::act("halfway"),
        Instr::wait(),
    });
    EXPECT_TRUE(r.runToPc(3, 5));
    EXPECT_NEAR(r.plant.truth().theta, 45.0, 6.0);  // fired as it crossed 45, not at the end
    r.run(5);
    EXPECT_NEAR(r.plant.truth().theta, 90.0, 3.0);
}

TEST(turn_relative_adds_to_the_last_heading_target) {
    Rig r;
    r.ctrl.setMission({
        Instr::turnSet(40, 90), Instr::wait(),
        Instr::turnRelative(30, 90), Instr::wait(),   // 40 + 30
    });
    r.run(6);
    EXPECT_NEAR(r.plant.truth().theta, 70.0, 3.0);
}

TEST(turn_to_point_faces_the_point) {
    Rig r;
    r.ctrl.setMission({Instr::turnToPoint(24, 24, 90), Instr::wait()});
    r.run(4);
    EXPECT_NEAR(r.plant.truth().theta, 45.0, 3.0);  // compass bearing to (24, 24)

    Rig rev;
    rev.ctrl.setMission({Instr::turnToPoint(24, 24, 90, DriveDirection::Reverse), Instr::wait()});
    rev.run(4);
    EXPECT_NEAR(std::fabs(wrapDeg(rev.plant.truth().theta - (-135.0))), 0.0, 3.0);  // back faces it
}

TEST(drive_raw_is_open_loop_until_the_next_motion) {
    // chassis.drive_set(100, 100) then a timed push, as in the corner shove.
    Rig r;
    r.ctrl.setMission({Instr::driveRaw(100, 100), Instr::delay(500), Instr::driveSet(0, 127), Instr::wait()});
    EXPECT_TRUE(r.runToPc(2, 3));
    const double pushed = r.plant.truth().y;
    EXPECT_TRUE(pushed > 10.0);  // ~0.5 s at 100/127 of top speed
    r.run(6);
    EXPECT_TRUE(r.ctrl.status().done);
}

TEST(wait_quick_releases_at_the_target_without_settling) {
    Rig quick, full;
    quick.ctrl.setMission({Instr::driveSet(24, 127), Instr::waitQuick()});
    full.ctrl.setMission({Instr::driveSet(24, 127), Instr::wait()});
    quick.run(5);
    full.run(5);
    EXPECT_TRUE(quick.ctrl.status().done);
    EXPECT_TRUE(quick.t < full.t);  // no 90 ms small-error dwell
}

TEST(odom_reset_mid_routine_moves_only_the_estimate) {
    Rig r;
    r.ctrl.setMission({Instr::driveSet(12, 110), Instr::wait(), Instr::odomReset(-60, -60, 0)});
    r.run(5);
    // The estimate jumps to the new pose. It can drift a hair afterwards: the
    // last drive is still live (EZ keeps holding it after the routine ends).
    EXPECT_NEAR(r.ctrl.pose().x, -60.0, 0.5);
    EXPECT_NEAR(r.ctrl.pose().y, -60.0, 0.5);
    EXPECT_NEAR(r.plant.truth().y, 12.0, 1.0);  // the robot itself didn't jump
}

TEST(negative_speed_is_the_same_as_positive) {
    // ez::pid_speed_max_set takes abs(): a "backwards" -100 swing is just 100.
    Rig pos, neg;
    pos.ctrl.setMission({Instr::swingSet(SwingSide::Left, 45, 100), Instr::wait()});
    neg.ctrl.setMission({Instr::swingSet(SwingSide::Left, 45, -100), Instr::wait()});
    pos.run(5);
    neg.run(5);
    EXPECT_NEAR(pos.plant.truth().theta, neg.plant.truth().theta, 1e-9);
    EXPECT_NEAR(pos.t, neg.t, 1e-9);
    EXPECT_NEAR(neg.plant.truth().theta, 45.0, 5.0);
    EXPECT_TRUE(!neg.ctrl.status().interfered);
}

TEST(ez_quirk_slew_collapses_if_moving_the_wrong_way) {
    // EZ's slew is a line in error space from min_speed at the start. If the
    // robot is still rolling the *other* way when a slewed motion starts, it
    // moves away from the ramp, the line extrapolates past min_speed, and the
    // speed cap collapses toward zero. Found by the sim after a timed
    // drive_set push; on the field the wall usually stops the robot first.
    Rig r;
    r.ctrl.setMission({Instr::driveRaw(127, 127), Instr::delay(800),
                       Instr::driveSet(-10, 127, /*slew=*/true), Instr::wait(3000)});
    EXPECT_TRUE(r.runToPc(3, 3));  // just after the reverse drive is set
    double weakest = 1e9;
    for (int i = 0; i < 40; ++i) {
        const WheelCmd c = r.ctrl.tick(r.plant.sensors(), r.dt);
        r.plant.step(c, r.dt);
        weakest = std::fmin(weakest, std::fmax(std::fabs(c.left), std::fabs(c.right)));
    }
    EXPECT_TRUE(weakest < 20.0);  // nowhere near the 127 a 10 in error asks for

    // Same move without slew drives hard straight away.
    Rig n;
    n.ctrl.setMission({Instr::driveRaw(127, 127), Instr::delay(800),
                       Instr::driveSet(-10, 127, /*slew=*/false), Instr::wait(3000)});
    EXPECT_TRUE(n.runToPc(3, 3));
    const WheelCmd c = n.ctrl.tick(n.plant.sensors(), n.dt);
    EXPECT_TRUE(std::fmax(std::fabs(c.left), std::fabs(c.right)) > 120.0);
}

TEST(walls_stop_the_chassis_but_not_the_encoders) {
    // A timed push into a wall: the robot stops, the wheels keep turning, and
    // odometry runs off. That's why teams re-zero odom after a wall push.
    PlantParams pp;
    pp.walls = true;
    Rig r(pp);
    r.setPose({0, 50, 0});
    r.ctrl.setMission({Instr::driveRaw(100, 100), Instr::delay(1500)});
    r.run(3);
    EXPECT_NEAR(r.plant.truth().y, 72.0 - 7.5, 1e-9);     // flush with the far wall
    EXPECT_TRUE(r.plant.wallContact());
    EXPECT_TRUE(r.ctrl.pose().y > 90.0);                   // odometry thinks it kept going
}

TEST(a_trailing_motion_without_a_wait_still_runs) {
    // `set_drive(-30);` as the last line of an auton: the function returns but
    // EZ's PID task keeps driving it. The robot must still get there.
    Rig r;
    r.ctrl.setMission({Instr::driveSet(20, 127), Instr::wait(), Instr::driveSet(-30, 127)});
    r.run(6);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().y, -10.0, 1.5);
}

TEST(runtime_constant_changes_apply) {
    // slew_drive_constants_set(1_in, 127) effectively turns the launch ramp off.
    Rig r;
    r.ctrl.setMission({Instr::slewDriveConstants(1, 127), Instr::driveSet(24, 127, true), Instr::wait()});
    const WheelCmd first = r.ctrl.tick(r.plant.sensors(), r.dt);
    EXPECT_TRUE(std::fmax(std::fabs(first.left), std::fabs(first.right)) > 120.0);
}

TEST(mission_is_deterministic) {
    // Same seed, same mission, bit-identical trajectory — the property the
    // golden-trace regression test in the sim workspace relies on.
    Rig a, b;
    a.ctrl.setMission(missions::square());
    b.ctrl.setMission(missions::square());
    a.run(30);
    b.run(30);
    EXPECT_NEAR(a.plant.truth().x, b.plant.truth().x, 1e-12);
    EXPECT_NEAR(a.plant.truth().y, b.plant.truth().y, 1e-12);
    EXPECT_NEAR(a.t, b.t, 1e-12);
}

TEST_MAIN()
