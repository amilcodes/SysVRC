// visbot/missions.hpp — routines the testbed can run.
//
// Field frame: origin at field centre, +y toward the far wall, inches. A VRC
// field is 12 ft square, so the legal area is ±72 in.
//
// `worldsMogoRush` is a direct transcription of the drive skeleton of
// worldsMogoRush() in v5/src/autons.cpp — same distances, same intermediate
// wait_until triggers, same mid-motion speed changes, same chaining. That is
// the point of the instruction model: a routine tuned on the robot can be
// replayed here without being redesigned.
#pragma once

#include <string>
#include <vector>

#include "visbot/motion.hpp"

namespace visbot {

struct MissionDef {
    std::string name;
    Pose start;
    Mission mission;
};

namespace missions {

/// Exercises every motion type: drive, turn, swing, odom point, chaining.
inline Mission skillsLoop() {
    return {
        Instr::driveSet(24.0, 110),  Instr::wait(),
        Instr::turnSet(90.0, 90),    Instr::wait(),
        Instr::driveSet(24.0, 110),  Instr::waitQuickChain(),
        Instr::turnSet(180.0, 90),   Instr::wait(),
        Instr::odomSet(24.0, -24.0, 100),   Instr::wait(6000),
        Instr::turnSet(-90.0, 90),   Instr::wait(),
        Instr::odomSet(-24.0, -24.0, 100),  Instr::wait(6000),
        Instr::swingSet(SwingSide::Left, 0.0, 90), Instr::wait(),
        Instr::odomSet(-24.0, 24.0, 100),   Instr::wait(6000),
        Instr::delay(300),
        Instr::odomSet(0.0, 0.0, 90, DriveDirection::Reverse), Instr::wait(6000),
        Instr::turnSet(0.0, 90),     Instr::wait(),
    };
}

/// Simple square — the clearest way to eyeball odometry drift.
inline Mission square(double side = 24.0) {
    return {
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(90, 90),   Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(180, 90),  Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(-90, 90),  Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(0, 90),    Instr::wait(),
    };
}

/// Back-to-back chained motions: shows momentum carrying between segments.
inline Mission chainDemo() {
    return {
        Instr::driveChainConstant(6),
        Instr::driveSet(24, 127), Instr::waitQuickChain(),
        Instr::turnSet(90, 110),  Instr::waitQuickChain(),
        Instr::driveSet(24, 127), Instr::waitQuickChain(),
        Instr::turnSet(180, 110), Instr::waitQuickChain(),
        Instr::driveSet(24, 127), Instr::wait(),
    };
}

/// worldsMogoRush() from v5/src/autons.cpp, drive skeleton, line for line.
/// `isBlue` mirrors the `int sgn = isBlue ? 1 : -1` in the original.
inline Mission worldsMogoRush(bool isBlue = true) {
    const double s = isBlue ? 1.0 : -1.0;
    const ActionId nearDoinker = isBlue ? ActionId::DoinkerRight : ActionId::DoinkerLeft;
    return {
        Instr::act(ActionId::ColorSortOff),
        Instr::act(ActionId::Ladybrown, 2),              // ChangeLBState(SEMIEXTENDED)

        // Rush the first mogo, grabbing the two-stack on the way past.
        Instr::driveSet(36, 127),                        // set_drive(32 + 4, ...)
        Instr::delay(100),
        Instr::act(ActionId::IntakeOut),
        Instr::act(ActionId::ColorSortOn),
        Instr::waitUntil(6),                             // pid_wait_until(12 - 6)
        Instr::act(nearDoinker),
        Instr::delay(150),
        Instr::act(ActionId::IntakeStop),
        Instr::waitUntil(30.5),                          // pid_wait_until(27 + 3.5)
        Instr::act(nearDoinker),
        Instr::waitUntil(34.5),                          // pid_wait_until(30.5 + 4)

        // Drag it back out of the corner.
        Instr::driveSet(-12, 120),
        Instr::waitUntil(-9),
        Instr::act(nearDoinker),
        Instr::wait(),
        Instr::delay(250),
        Instr::driveSet(-12, 127),                       // set_drive(-7 - 5)
        Instr::wait(),
        Instr::act(nearDoinker),

        // Turn to the first ring and collect it.
        Instr::turnSet(-57 * s, 90),
        Instr::wait(),
        Instr::act(ActionId::IntakeIn),
        Instr::driveSet(18, 127),                        // set_drive(17 - 1 + 2)
        Instr::waitUntil(16),
        Instr::driveSet(-2, 127),
        Instr::wait(),

        // Back onto the mogo and clamp it.
        Instr::turnSet(90 * s, 90),
        Instr::waitUntil(2),
        Instr::wait(),
        Instr::driveSet(-12, 80),                        // set_drive(-17 + 5, ..., 80)
        Instr::waitUntil(-7),
        Instr::speedMax(70),
        Instr::waitUntil(-8),                            // pid_wait_until(-10 + 2)
        Instr::act(ActionId::MogoClamp),
        Instr::act(ActionId::IntakeIn),
        Instr::act(ActionId::ColorSortOff),
        Instr::waitUntil(-11),

        // Push through the ring line, dropping the goal mid-motion.
        Instr::driveChainConstant(2),
        Instr::driveSet(38, 127),                        // pid_drive_set(43 - 5, 127)
        Instr::waitUntil(6),                             // pid_wait_until(15 - 9)
        Instr::act(ActionId::MogoRelease),
        Instr::act(ActionId::IntakeOut),
        Instr::waitQuickChain(),

        // Second mogo.
        Instr::turnSet(125 * s, 90),                     // (127 - 2) * sgn
        Instr::wait(),
        Instr::driveSet(-38, 90),                        // set_drive(-35 - 3, ..., 90)
        Instr::waitUntil(-10),
        Instr::speedMax(60),
        Instr::waitUntil(-33),                           // pid_wait_until(-30 - 3)
        Instr::act(ActionId::MogoClamp),
        Instr::wait(),
        Instr::act(ActionId::IntakeIn),

        // Into the corner.
        Instr::turnSet(138.5 * s, 90),                   // (135.5 + 3) * sgn
        Instr::waitQuickChain(),
        Instr::driveSet(30, 127),
        Instr::waitQuickChain(),
    };
}

inline MissionDef byName(const std::string& n, bool isBlue = true) {
    if (n == "mogo_rush" || n == "worlds_mogo_rush")
        return {"worlds_mogo_rush", {0, 0, -89.0 * (isBlue ? 1 : -1)}, worldsMogoRush(isBlue)};
    if (n == "square")  return {"square", {0, 0, 0}, square()};
    if (n == "chain")   return {"chain", {0, 0, 0}, chainDemo()};
    return {"skills", {0, 0, 0}, skillsLoop()};
}

inline std::vector<std::string> names() {
    return {"skills", "square", "chain", "mogo_rush"};
}

}  // namespace missions
}  // namespace visbot
