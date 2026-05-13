// visbot/missions.hpp — canned routes used by the sim and the tests.
// Field frame: origin at field centre, +y toward the far wall, inches.
// A VRC field is 12 ft square: ±72 in.
#pragma once

#include "visbot/motion.hpp"

namespace visbot::missions {

/// A skills-style loop: sprint, square off, odom hops across the field, back.
inline Mission skillsLoop() {
    return {
        Step::driveDistance(24.0, 110),
        Step::turnTo(90.0, 90),
        Step::driveDistance(24.0, 110),
        Step::turnTo(180.0, 90),
        Step::driveToPoint(24.0, -24.0, 100),
        Step::turnTo(-90.0, 90),
        Step::driveToPoint(-24.0, -24.0, 100),
        Step::driveToPoint(-24.0, 24.0, 100),
        Step::wait(300),
        Step::driveToPoint(0.0, 0.0, 90, /*reverse=*/true),
        Step::turnTo(0.0, 90),
    };
}

/// The opening of the "mogo rush" from autons.cpp: a fast straight, a hard
/// turn, and a reverse onto the goal.
inline Mission mogoRush() {
    return {
        Step::driveDistance(36.0, 127),
        Step::turnTo(-45.0, 110),
        Step::driveDistance(-14.0, 90),
        Step::wait(250),
        Step::driveToPoint(-18.0, 30.0, 110),
    };
}

/// Simple square — good for eyeballing odometry drift.
inline Mission square(double side = 24.0) {
    return {
        Step::driveDistance(side, 100), Step::turnTo(90, 90),
        Step::driveDistance(side, 100), Step::turnTo(180, 90),
        Step::driveDistance(side, 100), Step::turnTo(-90, 90),
        Step::driveDistance(side, 100), Step::turnTo(0, 90),
    };
}

}  // namespace visbot::missions
