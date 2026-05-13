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
    Rig() : plant(rp), ctrl(rp) {
        plant.reset({0, 0, 0});
        ctrl.resetPose({0, 0, 0}, plant.sensors());
    }
    // Run until the mission is done or maxSec elapses.
    double run(double maxSec) {
        while (!ctrl.status().done && t < maxSec) {
            WheelCmd c = ctrl.tick(plant.sensors(), dt);
            plant.step(c, dt);
            t += dt;
        }
        return t;
    }
};
}  // namespace

TEST(tick_before_mission_is_safe) {
    Rig r;
    for (int i = 0; i < 50; ++i) {
        WheelCmd c = r.ctrl.tick(r.plant.sensors(), r.dt);
        EXPECT_NEAR(c.left, 0, 1e-12); EXPECT_NEAR(c.right, 0, 1e-12);
    }
    EXPECT_TRUE(!r.ctrl.status().done);
    EXPECT_TRUE(std::string(r.ctrl.status().stepName) == "idle");
}

TEST(drive_24in_converges) {
    Rig r;
    r.ctrl.setMission({Step::driveDistance(24, 110)});
    r.run(5);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().y, 24.0, 1.0);
    EXPECT_NEAR(r.plant.truth().x, 0.0, 0.5);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::SmallError || r.ctrl.status().lastExit == ExitReason::BigError);
    EXPECT_TRUE(r.t < 2.5);
}

TEST(turn_90_converges) {
    Rig r;
    r.ctrl.setMission({Step::turnTo(90, 90)});
    r.run(4);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().theta, 90.0, 3.0);
    EXPECT_TRUE(r.ctrl.status().lastExit != ExitReason::Timeout);
}

TEST(turn_takes_shortest_path) {
    Rig r;
    r.plant.reset({0, 0, 170});
    r.ctrl.resetPose({0, 0, 170}, r.plant.sensors());
    r.ctrl.setMission({Step::turnTo(-170, 90)});
    r.run(4);
    EXPECT_NEAR(std::fabs(wrapDeg(r.plant.truth().theta - (-170))), 0.0, 3.0);
    EXPECT_TRUE(r.t < 1.5);  // 20 degrees, not 340
}

TEST(drive_to_point_reaches_target) {
    Rig r;
    r.ctrl.setMission({Step::driveToPoint(24, 24, 100)});
    r.run(6);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().x, 24.0, 2.0);
    EXPECT_NEAR(r.plant.truth().y, 24.0, 2.0);
    EXPECT_TRUE(r.ctrl.status().lastExit != ExitReason::Timeout);
}

TEST(drive_to_point_reverse) {
    Rig r;
    r.ctrl.setMission({Step::driveToPoint(0, -24, 90, true)});
    r.run(6);
    EXPECT_NEAR(r.plant.truth().y, -24.0, 2.0);
    EXPECT_NEAR(std::fabs(r.plant.truth().theta), 0.0, 8.0);  // still facing forward
}

TEST(skills_loop_completes) {
    Rig r;
    r.ctrl.setMission(missions::skillsLoop());
    r.run(30);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.plant.truth().x, 0.0, 3.0);
    EXPECT_NEAR(r.plant.truth().y, 0.0, 3.0);
    EXPECT_NEAR(r.ctrl.pose().x, r.plant.truth().x, 2.0);  // odom vs truth
    EXPECT_NEAR(r.ctrl.pose().y, r.plant.truth().y, 2.0);
    std::printf("  skills loop: %.2f s, odom error %.2f in\n", r.t, r.ctrl.pose().distanceTo(r.plant.truth()));
}

TEST(timeout_fires_when_stalled) {
    Rig r;
    PlantParams pp; pp.motorTauSec = 1e9;  // motors never move
    r.plant = DiffDrivePlant(r.rp, pp);
    r.plant.reset({0, 0, 0});
    r.ctrl.setMission({Step::driveDistance(24, 110, 800)});
    r.run(3);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_TRUE(r.ctrl.status().lastExit == ExitReason::Timeout || r.ctrl.status().lastExit == ExitReason::Velocity);
    EXPECT_TRUE(r.t < 1.0);
}

TEST_MAIN()
