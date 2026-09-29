// visbot/missions.hpp — a few built-in routines for testing the sim itself.
//
// Real autons don't live here. They're imported from v5/src/autons.cpp by
// tools/ez_import.py into autons/*.auton and loaded with auton_file.hpp, so
// the sim runs exactly what's in the competition code instead of a hand copy.
//
// Field frame: origin at field centre, +y toward the far wall, inches. A VRC
// field is 12 ft square, so the legal area is +/-72 in.
#pragma once

#include <string>
#include <vector>

#include "visbot/motion.hpp"

namespace visbot {

struct MissionDef {
    std::string name;
    Pose start;             // what the code believes (odom_xyt_set)
    Mission mission;
    /// Where the robot really starts, in field coordinates (origin at field
    /// centre, +y toward the far wall). Optional; needed to model walls.
    bool hasFieldStart = false;
    Pose fieldStart;
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

/// Simple square. The clearest way to eyeball odometry drift.
inline Mission square(double side = 24.0) {
    return {
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(90, 90),  Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(180, 90), Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(-90, 90), Instr::wait(),
        Instr::driveSet(side, 100), Instr::wait(), Instr::turnSet(0, 90),   Instr::wait(),
    };
}

/// Back-to-back chained motions: momentum carries between segments.
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

inline bool isBuiltin(const std::string& n) {
    return n == "skills" || n == "square" || n == "chain";
}

inline MissionDef byName(const std::string& n) {
    MissionDef d;
    d.name = n == "square" || n == "chain" ? n : "skills";
    d.mission = n == "square" ? square() : n == "chain" ? chainDemo() : skillsLoop();
    return d;
}

inline std::vector<std::string> names() { return {"skills", "square", "chain"}; }

}  // namespace missions
}  // namespace visbot
