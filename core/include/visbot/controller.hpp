// visbot/controller.hpp: what the tools need from a drive library's auton
// code. MotionController (EZ-Template) and LemController (LemLib) both fit.
#pragma once

#include <cstdint>

#include "visbot/geometry.hpp"
#include "visbot/instr.hpp"
#include "visbot/odometry.hpp"
#include "visbot/pid.hpp"

namespace visbot {

enum class DriveMode : uint8_t { Disabled, Drive, Turn, Swing, PointToPoint, Raw };

inline const char* toString(DriveMode m) {
    switch (m) {
        case DriveMode::Disabled:     return "disabled";
        case DriveMode::Drive:        return "drive";
        case DriveMode::Turn:         return "turn";
        case DriveMode::Swing:        return "swing";
        case DriveMode::PointToPoint: return "point_to_point";
        case DriveMode::Raw:          return "raw";
    }
    return "?";
}

struct MotionStatus {
    int pc = -1;                  // program counter into the mission
    const char* instr = "idle";
    DriveMode mode = DriveMode::Disabled;
    ExitReason lastExit = ExitReason::Running;
    double error = 0.0;           // primary error of the active motion
    double instrElapsedMs = 0.0;
    double motionElapsedMs = 0.0;
    double missionMs = 0.0;       // since the routine started; stops when it's done
    bool done = false;
    bool interfered = false;      // ez::interfered: exited on velocity/timeout
    // fixed buffer, not a std::string, so this stays trivially copyable for the SeqLock
    char lastAction[48] = {0};
    int lastActionPc = -1;
    double lastActionAtMs = -1.0;
    int actionCount = 0;
    // LemLib motions run beside the auton instead of inside a wait, so they
    // report their own exits. EZ leaves these alone.
    int activePc = -1;            // the motion driving the robot right now
    bool busy = false;            // a motion is running or queued
    int exitCount = 0;            // bumped each time a motion finishes
    int exitPc = -1;              // ...and which instruction it was
};


class AutonController {
public:
    virtual ~AutonController() = default;
    /// Where the routine starts, as the code believes (odom_xyt_set / setPose).
    virtual void resetPose(const Pose& p, const SensorSnapshot& s) = 0;
    virtual void setMission(Mission m) = 0;
    /// Advance by dtSeconds and return the drive command for the next step.
    virtual WheelCmd tick(const SensorSnapshot& s, double dtSeconds) = 0;
    virtual const MotionStatus& status() const = 0;
    /// The controller's odometry estimate.
    virtual const Pose& pose() const = 0;
};

}  // namespace visbot
