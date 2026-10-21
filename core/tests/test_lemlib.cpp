// LemLib port: the task timing autons rely on, and the behaviour that isn't
// what a textbook controller would do. Closed loop against the plant at 120 Hz.
#include <cmath>
#include <string>
#include <vector>

#include "check.hpp"
#include "visbot/auton_file.hpp"
#include "visbot/visbot.hpp"

using namespace visbot;

namespace {

// LemLib's example project, which is also LemConfig's defaults
const char* kHeader =
    "# @lib lemlib\n"
    "# @lemlib drivetrain 10 4 360 2\n"
    "# @lemlib lateral 10 0 3 3 1 100 3 500 20\n"
    "# @lemlib angular 2 0 10 3 1 100 3 500 0\n";

MissionDef parseLem(const std::string& body) {
    const AutonParseResult r = parseAuton(std::string(kHeader) + body, "test");
    for (const auto& e : r.errors) std::printf("  parse: %s\n", e.c_str());
    return r.def;
}

struct Rig {
    MissionDef def;
    DiffDrivePlant plant;
    LemController ctrl;
    double t = 0;
    const double dt = 1.0 / 120.0;
    std::vector<double> actionTimes;
    std::vector<double> exitTimes;
    std::vector<ExitReason> exits;
    std::vector<WheelCmd> cmds;

    explicit Rig(const std::string& body, PlantParams pp = {})
        : def(parseLem(body)), plant(lemDrivetrain(def.lem), pp), ctrl(def.lem) {
        plant.reset(def.start);
        ctrl.resetPose(def.start, plant.sensors());
        ctrl.setMission(def.mission);
    }

    void step() {
        const WheelCmd c = ctrl.tick(plant.sensors(), dt);
        cmds.push_back(c);
        plant.step(c, dt);
        t += dt;
        const MotionStatus& st = ctrl.status();
        while (static_cast<int>(actionTimes.size()) < st.actionCount) actionTimes.push_back(t);
        while (static_cast<int>(exitTimes.size()) < st.exitCount) {
            exitTimes.push_back(t);
            exits.push_back(st.lastExit);
        }
    }
    void run(double maxSec) {
        while (!ctrl.status().done && t < maxSec) step();
    }
    const Pose& truth() const { return plant.truth(); }
};

}  // namespace

// ------------------------------------------------------------------ the math
TEST(pid_resets_integral_on_sign_flip_and_zero_counts_as_positive) {
    lem::Pid p(0, 1, 0, 0, true);  // the chassis builds both PIDs with signFlipReset = true
    EXPECT_NEAR(p.update(2), 2, 1e-6);
    EXPECT_NEAR(p.update(3), 5, 1e-6);
    EXPECT_NEAR(p.update(-1), 0, 1e-6);   // flipped: wiped, this tick's error too
    EXPECT_NEAR(p.update(0), 0, 1e-6);    // sgn(0) == 1 != sgn(-1): wiped again
}

TEST(pid_windup_range_zeroes_integral_outside_it) {
    lem::Pid p(0, 1, 0, 3, true);
    EXPECT_NEAR(p.update(5), 0, 1e-6);    // |5| > 3
    EXPECT_NEAR(p.update(2), 2, 1e-6);
    EXPECT_NEAR(p.update(2), 4, 1e-6);
}

TEST(exit_condition_needs_the_whole_window) {
    lem::ExitCondition e(1, 100);
    EXPECT_TRUE(!e.update(0.5f, 0));
    EXPECT_TRUE(!e.update(0.5f, 99));
    EXPECT_TRUE(e.update(0.5f, 100));
    EXPECT_TRUE(e.update(5.0f, 101));     // stays done until reset
    e.reset();
    EXPECT_TRUE(!e.getExit());
}

TEST(angle_error_directions) {
    EXPECT_NEAR(lem::angleError(10, 350, false), 20, 1e-4);
    EXPECT_NEAR(lem::angleError(350, 10, false), -20, 1e-4);
    EXPECT_NEAR(lem::angleError(10, 20, false, LemDirection::CW), 350, 1e-4);
    EXPECT_NEAR(lem::angleError(20, 10, false, LemDirection::CCW), -350, 1e-4);
}

TEST(odom_without_tracking_wheels_uses_the_left_side) {
    // Chassis::calibrate fills the empty verticals with the drive sides, and
    // update() takes distance from vertical1: the left one. Heading is the IMU's.
    LemConfig cfg;
    cfg.wheelDiameter = 360 / visbot::kPi;   // 1 in per wheel degree
    lem::Odometry odom;
    odom.configure(cfg);
    odom.calibrate();
    lem::Readings r;
    r.leftDeg = 10;   // only the left side turned (it slipped); the IMU says straight
    odom.update(r);
    EXPECT_NEAR(odom.pose().y, 10, 1e-4);
    EXPECT_NEAR(odom.pose().x, 0, 1e-4);
}

// ------------------------------------------------------------------ task timing
TEST(async_motion_returns_after_10ms) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 24, 2000)\naction(\"intake\")\n");
    r.run(5);
    EXPECT_TRUE(r.actionTimes.size() == 1);
    // called at t=0, returns 10 ms later; seen at the next 8.3 ms tick
    EXPECT_TRUE(r.actionTimes[0] > 0.009 && r.actionTimes[0] < 0.02);
    EXPECT_NEAR(r.truth().y, 24, 1.0);
}

TEST(sync_motion_blocks_until_done) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 24, 2000, {}, false)\naction(\"intake\")\n");
    r.run(5);
    EXPECT_TRUE(r.actionTimes.size() == 1 && r.exitTimes.size() == 1);
    EXPECT_NEAR(r.actionTimes[0], r.exitTimes[0], 1e-9);  // the same millisecond
}

TEST(second_motion_waits_for_the_first) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 24, 2000)\nturnToHeading(90, 1000)\naction(\"x\")\n");
    r.run(6);
    EXPECT_TRUE(r.exitTimes.size() == 2 && r.actionTimes.size() == 1);
    // the turn call blocks until the drive ends, then returns 10 ms later: one or two 8.3 ms ticks
    const double gap = r.actionTimes[0] - r.exitTimes[0];
    EXPECT_TRUE(gap > r.dt * 0.5 && gap < r.dt * 2.5);
    EXPECT_NEAR(r.truth().theta, 90, 3);
}

TEST(wait_until_polls_distance_traveled) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 48, 3000)\nwaitUntil(12)\naction(\"clamp\")\n");
    double yAtAction = -1;
    while (!r.ctrl.status().done && r.t < 5) {
        r.step();
        if (yAtAction < 0 && !r.actionTimes.empty()) yAtAction = r.truth().y;
    }
    EXPECT_TRUE(yAtAction > 12 && yAtAction < 16);
}

TEST(wait_until_done_before_any_motion_never_returns) {
    // distTraveled starts at 0, not -1
    Rig r("setPose(0, 0, 0)\nwaitUntilDone()\nmoveToPoint(0, 24, 1000)\n");
    r.run(5);
    EXPECT_TRUE(!r.ctrl.status().done);
    EXPECT_TRUE(r.ctrl.status().pc == 0);
    EXPECT_NEAR(r.truth().y, 0, 1e-9);
}

TEST(cancel_motion_stops_it) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 48, 3000)\ndelay(400)\ncancelMotion()\n");
    r.run(5);
    EXPECT_TRUE(r.exits.size() == 1 && r.exits[0] == ExitReason::Cancelled);
    EXPECT_TRUE(r.truth().y < 30);
}

TEST(routine_can_end_before_its_last_motion) {
    // autonomous() returns; the motion task keeps going
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 36, 3000)\n");
    r.run(5);
    EXPECT_TRUE(r.ctrl.status().done);
    EXPECT_NEAR(r.truth().y, 36, 1.0);
}

// ------------------------------------------------------------------ motions
TEST(move_to_point_settles) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(10, 30, 3000)\n");
    r.run(5);
    EXPECT_TRUE(r.exits.size() == 1);
    EXPECT_TRUE(r.exits[0] == ExitReason::SmallError || r.exits[0] == ExitReason::BigError);
    EXPECT_NEAR(r.truth().x, 10, 1.5);
    EXPECT_NEAR(r.truth().y, 30, 1.5);
}

TEST(motor_commands_are_whole_numbers) {
    // MotorGroup::move takes an int32_t
    Rig r("setPose(0, 0, 0)\nmoveToPoint(10, 30, 3000)\nturnToHeading(200, 1500)\n");
    r.run(6);
    bool whole = true;
    for (const WheelCmd& c : r.cmds) whole = whole && c.left == std::trunc(c.left) && c.right == std::trunc(c.right);
    EXPECT_TRUE(whole);
}

TEST(slow_move_to_point_speeds_up_when_close) {
    // inside 7.5 in maxSpeed becomes max(|last output|, 60), even when it was 30
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 30, 4000, {.maxSpeed = 30})\n");
    double peakFar = 0, peakClose = 0;
    while (!r.ctrl.status().done && r.t < 6) {
        r.step();
        const double c = std::fmax(std::fabs(r.cmds.back().left), std::fabs(r.cmds.back().right));
        // LemLib switches at 7.5 in; leave a margin for the tick it switches on
        double& peak = 30 - r.ctrl.pose().y < 8.0 ? peakClose : peakFar;
        peak = std::fmax(peak, c);
    }
    EXPECT_TRUE(peakFar <= 30);
    EXPECT_TRUE(peakClose > 30);
}

TEST(min_speed_exits_at_the_line_still_moving) {
    Rig r("setPose(0, 0, 0)\nmoveToPoint(0, 24, 3000, {.minSpeed = 60, .earlyExitRange = 4})\n");
    double yAtExit = -1, speedAtExit = 0;
    while (!r.ctrl.status().done && r.t < 5) {
        r.step();
        if (yAtExit < 0 && !r.exits.empty()) {
            yAtExit = r.truth().y;
            speedAtExit = 0.5 * (r.plant.leftSpeed() + r.plant.rightSpeed());
        }
    }
    EXPECT_TRUE(!r.exits.empty() && r.exits[0] == ExitReason::Passed);
    EXPECT_TRUE(yAtExit > 18 && yAtExit < 22);   // 4 in early
    EXPECT_TRUE(speedAtExit > 20);
}

TEST(turn_cw_goes_the_long_way) {
    Rig r("setPose(0, 0, 0)\nturnToHeading(-90, 3000, {.direction = CW_CLOCKWISE})\n");
    r.step();
    r.step();
    EXPECT_TRUE(r.cmds.back().left > 0 && r.cmds.back().right < 0);
    r.run(5);
    EXPECT_NEAR(r.truth().theta, 270, 3);   // continuous heading: it went clockwise
}

TEST(turn_distance_traveled_tops_out_at_180) {
    // distTraveled for a turn is an angleError from the start, so it wraps:
    // waitUntil(200) can only return when the turn is over
    Rig r("setPose(0, 0, 0)\nturnToHeading(270, 3000, {.direction = CW_CLOCKWISE})\nwaitUntil(200)\naction(\"x\")\n");
    r.run(5);
    EXPECT_TRUE(r.exitTimes.size() == 1 && r.actionTimes.size() == 1);
    EXPECT_TRUE(r.actionTimes[0] >= r.exitTimes[0]);
}

TEST(swing_moves_one_side) {
    Rig r("setPose(0, 0, 0)\nswingToHeading(90, LEFT, 2000)\n");
    bool leftStill = true, rightBack = false;
    while (!r.ctrl.status().done && r.t < 4) {
        r.step();
        leftStill = leftStill && r.cmds.back().left == 0;
        rightBack = rightBack || r.cmds.back().right < 0;
    }
    EXPECT_TRUE(leftStill && rightBack);
    EXPECT_NEAR(r.truth().theta, 90, 3);
}

TEST(move_to_pose_arrives_facing_the_heading) {
    Rig r("setPose(0, 0, 0)\nmoveToPose(24, 30, 90, 4000)\n");
    r.run(6);
    EXPECT_NEAR(r.truth().x, 24, 2.0);
    EXPECT_NEAR(r.truth().y, 30, 2.0);
    EXPECT_NEAR(r.truth().theta, 90, 6);
}

TEST(follow_runs_to_the_end_of_the_path) {
    std::string path = "# @path line_txt\n";
    for (int i = 0; i <= 36; i += 2) path += "#   0, " + std::to_string(i) + ", " + (i < 36 ? "80" : "0") + "\n";
    path += "# @end\n";
    Rig r(path + "setPose(0, 0, 0)\nfollow(line_txt, 8, 4000)\n");
    r.run(6);
    EXPECT_TRUE(r.exits.size() == 1 && r.exits[0] == ExitReason::PathEnd);
    EXPECT_TRUE(r.truth().y > 28 && r.truth().y < 38);
    EXPECT_NEAR(r.truth().x, 0, 1.0);
}

TEST(tracking_pod_is_not_fooled_by_slip) {
    PlantParams slippy;
    slippy.slipFraction = 0.2;
    Rig drive("setPose(0, 0, 0)\nmoveToPoint(0, 30, 4000)\n", slippy);
    Rig pod("# @lemlib vertical1 pod 0\nsetPose(0, 0, 0)\nmoveToPoint(0, 30, 4000)\n", slippy);
    drive.run(6);
    pod.run(6);
    EXPECT_TRUE(std::fabs(drive.truth().y - 30) > 4);   // drive odom thinks it's there; it isn't
    EXPECT_NEAR(pod.truth().y, 30, 1.5);
}

// ------------------------------------------------------------------ file format
TEST(lemlib_file_round_trips) {
    const std::string body =
        "# @lemlib vertical1 pod -2.5\n"
        "# @lemlib horizontal1 pod -5.75\n"
        "# @lemlib throttle_curve 3 10 1.019\n"
        "# @path p_txt\n#   0, 0, 100\n#   0, 10, 0\n# @end\n"
        "setPose(-60, -30, 90)\n"
        "moveToPose(20, 15, 90, 4000, {.forwards = false, .lead = 0.3})\n"
        "turnToHeading(90, 1000, {.direction = CW_CLOCKWISE, .minSpeed = 100}, false)\n"
        "swingToPoint(1, 2, RIGHT, 800, {.maxSpeed = 60})\n"
        "follow(p_txt, 15, 4000, false)\n"
        "waitUntil(10)\ncancelMotion()\ntank(127, 127, true)\ndelay(200)\naction(\"clamp.toggle()\")\n";
    const MissionDef a = parseLem(body);
    const AutonParseResult b = parseAuton(toAutonText(a), "x");
    EXPECT_TRUE(b.ok());
    EXPECT_TRUE(b.def.lib == Library::LemLib && b.def.mission.size() == a.mission.size());
    for (size_t i = 0; i < a.mission.size() && i < b.def.mission.size(); ++i)
        EXPECT_TRUE(toAutonLine(a.mission[i]) == toAutonLine(b.def.mission[i]));
    EXPECT_NEAR(b.def.start.x, -60, 1e-9);
    EXPECT_NEAR(b.def.lem.horizontal1.offset, -5.75, 1e-9);
    EXPECT_NEAR(b.def.lem.throttle.gain, 1.019, 1e-9);
    EXPECT_TRUE(b.def.lem.paths.at("p_txt").size() == 2);
    EXPECT_TRUE(toAutonLine(a.mission[0]) == "moveToPose(20, 15, 90, 4000, {.forwards = false, .lead = 0.3})");
    EXPECT_TRUE(toAutonLine(a.mission[1]) ==
                "turnToHeading(90, 1000, {.direction = CW_CLOCKWISE, .minSpeed = 100}, false)");
}

TEST(lemlib_file_errors) {
    auto errors = [](const std::string& body) { return parseAuton(std::string(kHeader) + body).errors; };
    EXPECT_TRUE(!errors("moveToPoint(0, 24, 1000, {.lead = 0.3})\n").empty());   // no .lead on moveToPoint
    EXPECT_TRUE(!errors("follow(missing_txt, 10, 1000)\n").empty());
    EXPECT_TRUE(!errors("pid_drive_set(24, 110)\n").empty());                     // EZ call in a LemLib file
    EXPECT_TRUE(!errors("swingToHeading(90, UP, 1000)\n").empty());
    EXPECT_TRUE(!errors("# @lemlib lateral 1 2 3\nmoveToPoint(0, 1, 100)\n").empty());
    EXPECT_TRUE(errors("moveToPoint(0, 24, 1000, {}, false)\n").empty());
}

TEST_MAIN()
