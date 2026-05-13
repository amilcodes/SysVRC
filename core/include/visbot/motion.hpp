// visbot/motion.hpp — autonomous motion primitives that mirror the EZ-Template
// calls used in v5/src/autons.cpp:
//
//   chassis.pid_drive_set(24_in, 110)        -> Step::driveDistance(24, 110)
//   chassis.pid_turn_set(90_deg, 90)         -> Step::turnTo(90, 90)
//   chassis.pid_odom_set({{24, 24}, fwd, 90})-> Step::driveToPoint(24, 24, 90)
//   pros::delay(500)                         -> Step::wait(500)
//
// A Mission is a list of Steps run by the MotionController, which is ticked
// from the 120 Hz loop with the current sensor snapshot and returns a
// WheelCmd. The V5 build and the sim build tick the identical code.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "visbot/constants.hpp"
#include "visbot/geometry.hpp"
#include "visbot/odometry.hpp"
#include "visbot/pid.hpp"
#include "visbot/slew.hpp"

namespace visbot {

struct Step {
    enum class Kind : uint8_t { DriveDistance, TurnTo, DriveToPoint, Wait };
    Kind kind = Kind::Wait;
    double a = 0.0;          // distance (in) | heading (deg) | x (in)
    double b = 0.0;          // y (in)
    double speed = 127.0;    // 0..127 cap
    bool reverse = false;    // DriveToPoint: drive backwards to the point
    double timeoutMs = 0.0;  // 0 = no timeout
    bool slew = true;

    static Step driveDistance(double in, double speed = 110, double timeoutMs = 3000) {
        Step s; s.kind = Kind::DriveDistance; s.a = in; s.speed = speed; s.timeoutMs = timeoutMs; return s;
    }
    static Step turnTo(double headingDeg, double speed = 90, double timeoutMs = 2000) {
        Step s; s.kind = Kind::TurnTo; s.a = headingDeg; s.speed = speed; s.timeoutMs = timeoutMs; s.slew = false; return s;
    }
    static Step driveToPoint(double x, double y, double speed = 110, bool reverse = false, double timeoutMs = 4000) {
        Step s; s.kind = Kind::DriveToPoint; s.a = x; s.b = y; s.speed = speed; s.reverse = reverse; s.timeoutMs = timeoutMs; return s;
    }
    static Step wait(double ms) { Step s; s.kind = Kind::Wait; s.a = ms; return s; }

    const char* name() const {
        switch (kind) {
            case Kind::DriveDistance: return "drive";
            case Kind::TurnTo:        return "turn";
            case Kind::DriveToPoint:  return "odom_drive";
            case Kind::Wait:          return "wait";
        }
        return "?";
    }
};

using Mission = std::vector<Step>;

/// Per-tick controller telemetry, surfaced on /visbot/control_state in sim.
struct MotionStatus {
    int stepIndex = -1;
    const char* stepName = "idle";
    ExitReason lastExit = ExitReason::Running;
    double error = 0.0;      // primary controller error for the active step
    double stepElapsedMs = 0.0;
    bool done = false;
};

class MotionController {
public:
    explicit MotionController(RobotParams rp = {}, DriveGains g = {})
        : params_(rp), gains_(g), odom_(rp),
          leftPid_(g.drive, g.driveExit), rightPid_(g.drive, g.driveExit),
          headingPid_(g.heading), turnPid_(g.turn, g.turnExit),
          odomDrivePid_(g.drive, g.odomDriveExit), odomAngPid_(g.odomAngular),
          driveSlew_(g.slewDriveDistanceIn, g.slewDriveMinSpeed),
          turnSlew_(g.slewTurnDistanceDeg, g.slewTurnMinSpeed) {}

    /// Set the field pose (e.g. at the start of an auton) — like
    /// chassis.odom_xyt_set(...) + chassis.drive_imu_reset().
    void resetPose(const Pose& p, const SensorSnapshot& s) { odom_.reset(p, s.enc, s.imuHeadingDeg); }

    void setMission(Mission m) {
        mission_ = std::move(m);
        stepIdx_ = -1;
        status_ = {};
        advance();
    }

    const Pose& pose() const { return odom_.pose(); }
    const MotionStatus& status() const { return status_; }
    const Mission& mission() const { return mission_; }

    /// Main entry: one control tick.
    WheelCmd tick(const SensorSnapshot& s, double dtSeconds) {
        odom_.update(s.enc, s.imuHeadingDeg);
        // No mission armed yet (or finished): keep odometry alive, output zero.
        if (status_.done || stepIdx_ < 0 || stepIdx_ >= static_cast<int>(mission_.size())) return {};
        const Step& st = mission_[stepIdx_];
        status_.stepElapsedMs += dtSeconds * 1000.0;

        WheelCmd cmd;
        ExitReason exit = ExitReason::Running;
        switch (st.kind) {
            case Step::Kind::DriveDistance: cmd = tickDrive(st, s, dtSeconds, exit); break;
            case Step::Kind::TurnTo:        cmd = tickTurn(st, s, dtSeconds, exit); break;
            case Step::Kind::DriveToPoint:  cmd = tickOdomDrive(st, s, dtSeconds, exit); break;
            case Step::Kind::Wait:
                if (status_.stepElapsedMs >= st.a) exit = ExitReason::SmallError;
                break;
        }
        if (st.timeoutMs > 0 && status_.stepElapsedMs > st.timeoutMs && exit == ExitReason::Running)
            exit = ExitReason::Timeout;

        if (exit != ExitReason::Running) {
            status_.lastExit = exit;
            advance();
            return {};  // one tick of zero output between steps, like pros::delay boundaries
        }
        return cmd.clipped();
    }

private:
    void advance() {
        ++stepIdx_;
        status_.stepElapsedMs = 0.0;
        status_.error = 0.0;
        if (stepIdx_ >= static_cast<int>(mission_.size())) {
            status_.done = true;
            status_.stepIndex = stepIdx_;
            status_.stepName = "done";
            return;
        }
        status_.stepIndex = stepIdx_;
        status_.stepName = mission_[stepIdx_].name();
        armed_ = false;
    }

    WheelCmd tickDrive(const Step& st, const SensorSnapshot& s, double dt, ExitReason& exit) {
        const double l = odom_.leftInches(s.enc), r = odom_.rightInches(s.enc);
        if (!armed_) {
            leftPid_.setTarget(l + st.a);
            rightPid_.setTarget(r + st.a);
            headingPid_.setTarget(odom_.pose().theta);
            driveSlew_.initialize(st.slew, st.speed, l + st.a, l);
            armed_ = true;
        }
        const double lo = leftPid_.compute(l, dt);
        const double ro = rightPid_.compute(r, dt);
        const double ho = headingPid_.computeError(wrapDeg(headingPid_.target() - odom_.pose().theta), dt);
        const double cap = driveSlew_.iterate(l);

        WheelCmd c{clamp(lo, -cap, cap) + ho, clamp(ro, -cap, cap) - ho};
        status_.error = 0.5 * (leftPid_.error() + rightPid_.error());

        const ExitReason le = leftPid_.exitCondition(dt), re = rightPid_.exitCondition(dt);
        if (le != ExitReason::Running && re != ExitReason::Running) exit = le;
        return c;
    }

    WheelCmd tickTurn(const Step& st, const SensorSnapshot&, double dt, ExitReason& exit) {
        if (!armed_) {
            turnPid_.setTarget(st.a);
            turnSlew_.initialize(st.slew, st.speed, st.a, odom_.pose().theta);
            armed_ = true;
        }
        const double err = wrapDeg(turnPid_.target() - odom_.pose().theta);  // shortest path
        double out = turnPid_.computeError(err, dt);
        const double cap = turnSlew_.enabled() ? turnSlew_.iterate(odom_.pose().theta) : st.speed;
        out = clamp(out, -cap, cap);
        status_.error = err;
        exit = turnPid_.exitCondition(dt);
        return {out, -out};  // compass heading: positive error = turn right
    }

    WheelCmd tickOdomDrive(const Step& st, const SensorSnapshot&, double dt, ExitReason& exit) {
        const Pose& p = odom_.pose();
        const double dist = p.distanceTo({st.a, st.b, 0});
        if (!armed_) {
            odomDrivePid_.setTarget(0.0);
            odomAngPid_.setTarget(0.0);
            driveSlew_.initialize(st.slew, st.speed, dist, 0.0);
            startDist_ = dist;
            settleHeading_ = p.theta;
            armed_ = true;
        }
        // Heading to the point (flipped when driving in reverse).
        double targetHeading = p.headingTo(st.a, st.b);
        if (st.reverse) targetHeading = wrapDeg(targetHeading + 180.0);

        // Close to the point the bearing swings wildly; freeze steering and
        // finish the approach straight (EZ's "settle" behaviour).
        const bool settling = dist < 2.0 * gains_.odomDriveExit.bigError;
        if (!settling) settleHeading_ = targetHeading;
        const double angErr = wrapDeg(settleHeading_ - p.theta);

        // Signed distance along our heading so overshoot drives backwards.
        const double sign = st.reverse ? -1.0 : 1.0;
        const double along = dist * std::cos(deg2rad(angErr));
        const double lin = -odomDrivePid_.compute(sign * along, dt);   // error = 0 - (-along)
        const double ang = settling ? 0.0 : odomAngPid_.computeError(angErr, dt);

        const double cap = driveSlew_.iterate(startDist_ - dist);
        // Turn bias: angular gets first claim on the speed budget.
        const double angCap = cap * gains_.odomTurnBias;
        const double a = clamp(ang, -angCap, angCap);
        const double linCap = cap - std::fabs(a);
        const double v = clamp(lin, -linCap, linCap);

        status_.error = dist;
        exit = odomDrivePid_.exitCondition(dt);
        return {v + a, v - a};
    }

    RobotParams params_;
    DriveGains gains_;
    Odometry odom_;
    Pid leftPid_, rightPid_, headingPid_, turnPid_, odomDrivePid_, odomAngPid_;
    Slew driveSlew_, turnSlew_;
    Mission mission_;
    int stepIdx_ = -1;
    bool armed_ = false;
    double startDist_ = 0.0, settleHeading_ = 0.0;
    MotionStatus status_{};
};

}  // namespace visbot
