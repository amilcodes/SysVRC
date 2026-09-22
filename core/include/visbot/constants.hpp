// visbot/constants.hpp — the one place robot geometry and tuned gains live.
//
// Every number here is lifted from the competition code: geometry from
// v5/src/subsystemFiles/globals.cpp, gains and exit conditions from
// default_constants() in v5/src/autons.cpp. Change them here and both the
// firmware and the sim pick up the change.
#pragma once

#include "visbot/pid.hpp"

namespace visbot {

struct RobotParams {
    double wheelDiameterIn = 3.25;   // ez::Drive chassis(..., 3.25, ...)
    double wheelRpm        = 450.0;  // blue cartridge through the gearing
    double trackWidthIn    = 12.5;   // centre-to-centre of the drive wheels
    double robotLengthIn   = 15.0;
    double robotWidthIn    = 15.0;
    double massKg          = 7.5;

    /// Wheel surface speed at full command (127).
    double maxWheelSpeedInPerSec() const {
        return wheelRpm / 60.0 * wheelDiameterIn * kPi;
    }
    double inchesPerWheelDegree() const {
        return wheelDiameterIn * kPi / 360.0;
    }
};

/// default_constants() from v5/src/autons.cpp, one for one.
struct DriveGains {
    // pid_drive_constants_set(16.0, 0.0, 100.0)
    PidGains drive{16.0, 0.0, 100.0, 0.0};
    // pid_heading_constants_set(9.5, 0.0, 20)
    PidGains heading{9.5, 0.0, 20.0, 0.0};
    // pid_turn_constants_set(3.7, 0, 20, 15.0)
    PidGains turn{3.7, 0.0, 20.0, 15.0};
    // pid_swing_constants_set(6.0, 0.0, 65.0)
    PidGains swing{6.0, 0.0, 65.0, 0.0};
    // pid_odom_angular_constants_set(6.5, 0.0, 52.5)
    PidGains odomAngular{6.5, 0.0, 52.5, 0.0};
    // pid_odom_boomerang_constants_set(5.8, 0.0, 32.5) — boomerang motions are
    // not implemented here; kept so the constant isn't silently lost.
    PidGains boomerang{5.8, 0.0, 32.5, 0.0};

    // pid_turn_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 150_ms, 500_ms)
    ExitConditions turnExit{100, 3.0, 250, 7.0, 150};
    // pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms)
    ExitConditions swingExit{90, 3.0, 250, 7.0, 500};
    // pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 200_ms, 500_ms)
    ExitConditions driveExit{90, 1.0, 250, 3.0, 200};
    // pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms)
    ExitConditions odomDriveExit{90, 1.0, 250, 3.0, 500};
    // pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 7_deg, 500_ms, 750_ms)
    ExitConditions odomTurnExit{90, 1.0, 250, 7.0, 500};

    // slew_drive_constants_set(3_in, 70) / slew_turn_constants_set(3_deg, 70)
    // / slew_swing_constants_set(3_in, 80)
    double slewDriveDistanceIn  = 3.0;
    double slewDriveMinSpeed    = 70.0;
    double slewTurnDistanceDeg  = 3.0;
    double slewTurnMinSpeed     = 70.0;
    double slewSwingDistanceDeg = 3.0;
    double slewSwingMinSpeed    = 80.0;

    // pid_drive_chain_constant_set(3_in) / turn 3_deg / swing 5_deg
    double driveChainConstantIn  = 3.0;
    double turnChainConstantDeg  = 3.0;
    double swingChainConstantDeg = 5.0;

    // odom_turn_bias_set(0.9), odom_look_ahead_set(7_in)
    double odomTurnBias   = 0.9;
    double odomLookAheadIn = 7.0;

    /// ez::Drive::turn_min — caps turn speed once inside start_i. EZ defaults
    /// it to 0 (disabled) and the competition code never sets it.
    double turnMinSpeed = 0.0;
};

/// Cheesy-drive tuning from v5/src/subsystemFiles/drive.cpp.
struct CheesyParams {
    double deadband         = 0.1;   // DRIVE_DEADBAND
    double slew             = 0.02;  // DRIVE_SLEW (per tick, in [-1,1] units)
    double turnNonlinearity = 0.65;  // CD_TURN_NONLINEARITY
    double negInertiaScalar = 4.0;   // CD_NEG_INERTIA_SCALAR
    double sensitivity      = 1.00;  // CD_SENSITIVITY
};

}  // namespace visbot
