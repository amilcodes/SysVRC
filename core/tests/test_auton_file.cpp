// .auton files: parsing, round-tripping, and every imported routine from
// v5/src running start to finish against the plant.
#include <dirent.h>

#include <algorithm>
#include <string>
#include <vector>

#include "check.hpp"
#include "visbot/auton_file.hpp"
#include "visbot/plant.hpp"

using namespace visbot;

TEST(parses_the_common_calls) {
    const auto r = parseAuton(R"AUTON(
# @name demo
odom_xyt_set(0, 0, -89)
pid_drive_set(36, 127)            @1977  # set_drive(32 + 4, ...)
pid_drive_set(-12, 80, false)
delay(100)
action("intake.move(-127)")
pid_wait_until(6)
pid_turn_set(90, 110, longest)
pid_swing_set(left, 45, 90, 30)
pid_odom_set(24, 24, 110, rev)
pid_turn_to_point(24, -48, 127)
pid_turn_relative_set(30, 127)
drive_set(100, 100)
pid_wait_quick_chain()
pid_speed_max_set(70)
odom_xyt_set(10, 20, 30)
)AUTON");
    EXPECT_TRUE(r.ok());
    for (const auto& e : r.errors) std::printf("  %s\n", e.c_str());
    EXPECT_TRUE(r.def.name == "demo");
    EXPECT_NEAR(r.def.start.theta, -89.0, 1e-12);
    EXPECT_TRUE(r.def.mission.size() == 14u);  // first odom_xyt_set became the start pose
    EXPECT_TRUE(r.def.mission[0].op == Instr::Op::DriveSet);
    EXPECT_TRUE(r.def.mission[0].sourceLine == 1977);
    EXPECT_TRUE(!r.def.mission[1].slew);
    EXPECT_TRUE(r.def.mission[3].label == "intake.move(-127)");
    EXPECT_TRUE(r.def.mission[5].behavior == AngleBehavior::Longest);
    EXPECT_TRUE(r.def.mission[6].side == SwingSide::Left);
    EXPECT_TRUE(r.def.mission[7].dir == DriveDirection::Reverse);
    EXPECT_TRUE(r.def.mission[13].op == Instr::Op::OdomReset);  // later one resets mid-run
}

TEST(errors_name_the_line) {
    const auto r = parseAuton("pid_drive_set(24)\nbogus(1)\npid_turn_set(x, 90)\n");
    EXPECT_TRUE(r.errors.size() == 3u);
    EXPECT_TRUE(r.errors[0].find("line 1") == 0);
    EXPECT_TRUE(r.errors[1].find("unknown call 'bogus'") != std::string::npos);
    EXPECT_TRUE(r.errors[2].find("must be a number") != std::string::npos);
}

TEST(action_labels_keep_commas_hashes_and_quotes) {
    const auto r = parseAuton(R"AUTON(action("setIntake(127, \"fast\") # not a comment"))AUTON" "\n");
    EXPECT_TRUE(r.ok());
    EXPECT_TRUE(r.def.mission[0].label == "setIntake(127, \"fast\") # not a comment");
}

TEST(round_trips) {
    MissionDef def;
    def.name = "rt";
    def.start = {1, 2, 3};
    def.mission = missions::skillsLoop();
    def.hasFieldStart = true;
    def.fieldStart = {-48, -60, 90};
    def.mission.push_back(Instr::act("mogoClamp.toggle()"));
    def.mission.push_back(Instr::turnToPoint(5, 6, 100, DriveDirection::Reverse));
    const std::string text = toAutonText(def);
    const auto back = parseAuton(text);
    EXPECT_TRUE(back.ok());
    EXPECT_TRUE(back.def.mission.size() == def.mission.size());
    EXPECT_TRUE(toAutonText(back.def) == text);
    EXPECT_TRUE(back.def.hasFieldStart);
    EXPECT_NEAR(back.def.fieldStart.y, -60.0, 1e-12);
}

namespace {
std::vector<std::string> autonFiles() {
    std::vector<std::string> out;
    if (DIR* d = opendir(VISBOT_AUTONS_DIR)) {
        while (dirent* e = readdir(d)) {
            const std::string n = e->d_name;
            if (n.size() > 6 && n.compare(n.size() - 6, 6, ".auton") == 0) out.push_back(n);
        }
        closedir(d);
    }
    std::sort(out.begin(), out.end());
    return out;
}
}  // namespace

TEST(every_imported_auton_parses_and_runs) {
    // The point of the importer: the team's real routines, unmodified, run in
    // the sim. Each must parse cleanly and finish without hitting the
    // headless timeout backstop.
    const auto files = autonFiles();
    EXPECT_TRUE(files.size() >= 18u);
    for (const auto& f : files) {
        const auto r = loadAutonFile(std::string(VISBOT_AUTONS_DIR) + "/" + f);
        if (!r.ok()) {
            for (const auto& e : r.errors) std::printf("  %s: %s\n", f.c_str(), e.c_str());
            EXPECT_TRUE(r.ok());
            continue;
        }
        RobotParams rp;
        DiffDrivePlant plant(rp);
        MotionController ctrl(rp);
        plant.reset(r.def.start);
        ctrl.resetPose(r.def.start, plant.sensors());
        ctrl.setMission(r.def.mission);
        const double dt = 1.0 / 120.0;
        double t = 0;
        int timeouts = 0;
        ExitReason last = ExitReason::Running;
        while (!ctrl.status().done && t < 120) {
            plant.step(ctrl.tick(plant.sensors(), dt), dt);
            t += dt;
            if (ctrl.status().lastExit != last) {
                last = ctrl.status().lastExit;
                if (last == ExitReason::Timeout) ++timeouts;
            }
        }
        std::printf("  %-38s %4zu instr  %6.2f s  %3d actions  %s\n", f.c_str(), r.def.mission.size(), t,
                    ctrl.status().actionCount, ctrl.status().done ? "done" : "DID NOT FINISH");
        EXPECT_TRUE(ctrl.status().done);
        EXPECT_TRUE(timeouts == 0);
    }
}

TEST_MAIN()
