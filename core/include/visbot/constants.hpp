// visbot/constants.hpp — the one place robot geometry and tuned gains live.
//
// Every number here is lifted from the competition code (v5/src/autons.cpp
// default_constants() and v5/src/subsystemFiles/globals.cpp) so that what is
// tuned in sim is exactly what gets flashed.
#pragma once

#include "visbot/pid.hpp"

namespace visbot {

struct RobotParams {
    double wheelDiameterIn = 3.25;   // ez::Drive chassis(... 3.25 ...)
    double wheelRpm        = 450.0;  // blue cartridge, 450 RPM after gearing
    double trackWidthIn    = 12.5;   // centre-to-centre of drive wheels
    double robotLengthIn   = 15.0;   // for the sim model / collision box
    double robotWidthIn    = 15.0;
    double massKg          = 7.5;

    /// Peak wheel surface speed at full command (127).
    double maxWheelSpeedInPerSec() const {
        return wheelRpm / 60.0 * wheelDiameterIn * 3.14159265358979323846;
    }
    double inchesPerWheelDegree() const {
        return wheelDiameterIn * 3.14159265358979323846 / 360.0;
    }
};

/// EZ-Template constants from default_constants() in autons.cpp.
struct DriveGains {
    // chassis.pid_drive_constants_set(16.0, 0.0, 100.0)
    PidGains drive{16.0, 0.0, 100.0, 0.0};
    // chassis.pid_heading_constants_set(9.5, 0.0, 20)
    PidGains heading{9.5, 0.0, 20.0, 0.0};
    // chassis.pid_turn_constants_set(3.7, 0, 20, 15.0)
    PidGains turn{3.7, 0.0, 20.0, 15.0};
    // chassis.pid_swing_constants_set(6.0, 0.0, 65.0)
    PidGains swing{6.0, 0.0, 65.0, 0.0};
    // chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5)
    PidGains odomAngular{6.5, 0.0, 52.5, 0.0};

    // chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 150_ms, 500_ms)
    ExitConditions turnExit{100, 3.0, 250, 7.0, 150};
    // chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 200_ms, 500_ms)
    ExitConditions driveExit{90, 1.0, 250, 3.0, 200};
    // chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 7_deg, 500_ms, 750_ms)
    ExitConditions odomDriveExit{90, 1.0, 250, 3.0, 500};
    // chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 7_deg, 500_ms, 750_ms)
    ExitConditions odomTurnExit{90, 1.0, 250, 7.0, 500};

    // chassis.slew_drive_constants_set(3_in, 70) / slew_turn_constants_set(3_deg, 70)
    double slewDriveDistanceIn = 3.0;
    double slewDriveMinSpeed   = 70.0;
    double slewTurnDistanceDeg = 3.0;
    double slewTurnMinSpeed    = 70.0;

    // chassis.odom_turn_bias_set(0.9)
    double odomTurnBias = 0.9;
};

/// Cheesy-drive tuning from drive.cpp.
struct CheesyParams {
    double deadband         = 0.1;   // DRIVE_DEADBAND
    double slew             = 0.02;  // DRIVE_SLEW (per tick, in [-1,1] units)
    double turnNonlinearity = 0.65;  // CD_TURN_NONLINEARITY
    double negInertiaScalar = 4.0;   // CD_NEG_INERTIA_SCALAR
    double sensitivity      = 1.00;  // CD_SENSITIVITY
};

}  // namespace visbot
