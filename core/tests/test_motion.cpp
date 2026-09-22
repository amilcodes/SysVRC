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
        Instr::act(ActionId::MogoClamp),
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
    EXPECT_TRUE(r.ctrl.status().lastAction == ActionId::MogoClamp);
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

TEST(worlds_mogo_rush_runs_and_stays_on_the_field) {
    // The real routine from autons.cpp. It is an open-loop rush with doinker
    // timing, so the check is that it runs to completion, keeps the robot on
    // a 12 ft field, and fires its mechanism actions.
    Rig r;
    const MissionDef def = missions::byName("mogo_rush", /*isBlue=*/true);
    r.setPose(def.start);
    r.ctrl.setMission(def.mission);

    int actions = 0;
    ActionId last = ActionId::None;
    while (!r.ctrl.status().done && r.t < 40) {
        r.plant.step(r.ctrl.tick(r.plant.sensors(), r.dt), r.dt);
        r.t += r.dt;
        if (r.ctrl.status().lastAction != last) {
            last = r.ctrl.status().lastAction;
            ++actions;
        }
        EXPECT_TRUE(std::fabs(r.plant.truth().x) < 72.0);
        EXPECT_TRUE(std::fabs(r.plant.truth().y) < 72.0);
    }
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_TRUE(actions >= 10);
    std::printf("  mogo_rush: %.2f s, %d actions, ended (%.1f, %.1f, %.0f deg)\n",
                r.t, actions, r.plant.truth().x, r.plant.truth().y, r.plant.truth().theta);
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
