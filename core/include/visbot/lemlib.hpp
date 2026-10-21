// visbot/lemlib.hpp: LemLib v0.5.6's chassis on the sim's clock (chassis.cpp,
// odom.cpp, trackingWheel.cpp, pid.cpp, exitcondition.cpp, util.cpp,
// driveCurve.cpp, opcontrol.cpp, motions/*.cpp).
//
// On the brain a LemLib auton is three kinds of task: odometry every 10 ms, one
// task per motion (every loop iteration ends in pros::delay(10)), and the auton
// itself, which queues motions and blocks in waitUntil, waitUntilDone and
// pros::delay. LemController runs them on a 1 ms clock, in that order within a
// millisecond, so a routine sees the timing it gets on the robot: an async
// motion returns 10 ms after it's called, the next one waits for it, waitUntil
// polls every 10 ms.
//
// The math keeps LemLib's names, order and float type so the two can be read
// side by side. Anything that differs is marked "Divergence".
//
// LemLib is MIT licensed:
//
//   Copyright (c) 2024 Liam Teale and LemLib contributors
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to deal
//   in the Software without restriction, including without limitation the rights
//   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//   copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "visbot/controller.hpp"
#include "visbot/geometry.hpp"
#include "visbot/instr.hpp"
#include "visbot/odometry.hpp"

namespace visbot {
namespace lem {

constexpr float kPi = 3.14159265358979323846f;

inline float f(double v) { return static_cast<float>(v); }

// ------------------------------------------------------------------ util.cpp
/// sgn(0) is 1, by LemLib's convention. The PID's sign-flip reset depends on it.
template <typename T> constexpr T sgn(T value) { return value < 0 ? T(-1) : T(1); }
inline float radToDeg(float rad) { return rad * 180 / kPi; }
inline float degToRad(float deg) { return deg * kPi / 180; }

inline float slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

inline float sanitizeAngle(float angle, bool radians = true) {
    if (radians) return std::fmod(std::fmod(angle, 2 * kPi) + 2 * kPi, 2 * kPi);
    return std::fmod(std::fmod(angle, 360.0f) + 360.0f, 360.0f);
}

inline float angleError(float target, float position, bool radians = true,
                        LemDirection direction = LemDirection::Auto) {
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? 2 * kPi : 360.0f;
    const float rawError = target - position;
    switch (direction) {
        case LemDirection::CW: return rawError < 0 ? rawError + max : rawError;
        case LemDirection::CCW: return rawError > 0 ? rawError - max : rawError;
        case LemDirection::Auto: break;
    }
    return std::remainder(rawError, max);
}

// ------------------------------------------------------------------ pose.cpp
/// lemlib::Pose. x/y in inches; theta depends on who's asking (see getPose).
struct LPose {
    float x = 0, y = 0, theta = 0;

    LPose() = default;
    LPose(float px, float py, float ptheta = 0) : x(px), y(py), theta(ptheta) {}

    LPose operator+(const LPose& o) const { return {x + o.x, y + o.y, theta}; }
    LPose operator-(const LPose& o) const { return {x - o.x, y - o.y, theta}; }
    float operator*(const LPose& o) const { return x * o.x + y * o.y; }
    LPose operator*(float k) const { return {x * k, y * k, theta}; }
    LPose lerp(const LPose& o, float t) const { return {x + (o.x - x) * t, y + (o.y - y) * t, theta}; }
    float distance(const LPose& o) const { return std::hypot(x - o.x, y - o.y); }
    float angle(const LPose& o) const { return std::atan2(o.y - y, o.x - x); }
};

/// Signed curvature of the circle tangent to `pose` (standard form, radians)
/// through `other`. Used by boomerang (getCurvature) and pure pursuit
/// (findLookaheadCurvature, which is the same formula with the heading passed in).
inline float curvature(const LPose& pose, float heading, const LPose& other) {
    const float side = sgn(std::sin(heading) * (other.x - pose.x) - std::cos(heading) * (other.y - pose.y));
    const float a = -std::tan(heading);
    const float c = std::tan(heading) * pose.x - pose.y;
    const float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    const float d = std::hypot(other.x - pose.x, other.y - pose.y);
    return side * ((2 * x) / (d * d));
}

// ------------------------------------------------------- pid.cpp, exitcondition.cpp
class Pid {
public:
    Pid(float kP, float kI, float kD, float windupRange, bool signFlipReset)
        : kP_(kP), kI_(kI), kD_(kD), windupRange_(windupRange), signFlipReset_(signFlipReset) {}

    float update(float error) {
        integral_ += error;
        if (sgn(error) != sgn(prevError_) && signFlipReset_) integral_ = 0;
        if (std::fabs(error) > windupRange_ && windupRange_ != 0) integral_ = 0;
        const float derivative = error - prevError_;
        prevError_ = error;
        return error * kP_ + integral_ * kI_ + derivative * kD_;
    }
    void reset() {
        integral_ = 0;
        prevError_ = 0;
    }

private:
    float kP_, kI_, kD_, windupRange_;
    bool signFlipReset_;
    float integral_ = 0, prevError_ = 0;
};

/// In range for `time` ms straight, then done for good (until reset).
class ExitCondition {
public:
    ExitCondition(float range, int time) : range_(range), time_(time) {}

    bool getExit() const { return done_; }
    bool update(float input, int64_t nowMs) {
        if (std::fabs(input) > range_) startTime_ = -1;
        else if (startTime_ == -1) startTime_ = nowMs;
        else if (nowMs >= startTime_ + time_) done_ = true;
        return done_;
    }
    void reset() {
        startTime_ = -1;
        done_ = false;
    }

private:
    float range_;
    int time_;
    int64_t startTime_ = -1;
    bool done_ = false;
};

// ------------------------------------------------------------------ driveCurve.cpp
inline float expoCurve(const LemConfig::Curve& c, float input) {
    const float deadband = f(c.deadband), minOutput = f(c.minOutput), gain = f(c.gain);
    if (std::fabs(input) <= deadband) return 0;
    const float g = std::fabs(input) - deadband;
    const float g127 = 127 - deadband;
    const float i = std::pow(gain, g - 127) * g * sgn(input);
    const float i127 = std::pow(gain, g127 - 127) * g127;
    return (127.0f - minOutput) / 127 * i * 127 / i127 + minOutput * sgn(input);
}

// ------------------------------------------------------------------ odometry
/// Raw sensor values since calibrate().
struct Readings {
    float leftDeg = 0, rightDeg = 0;   // drive encoders, wheel degrees as the code's gearing reads them
    float forwardIn = 0, rightIn = 0;  // chassis travel, for unpowered tracking wheels
    float turnRad = 0;                 // chassis rotation, CW positive
    float imuDeg = 0;                  // get_rotation()
};

/// lemlib::TrackingWheel, reading from the sim instead of a sensor.
struct TrackingWheel {
    LemConfig::Wheel::Kind kind = LemConfig::Wheel::Kind::None;
    bool vertical = true;
    float offset = 0, diameter = 0, rpm = 0;
    float driveRpm = 0;   // the drivetrain rpm the encoder readings are scaled to

    bool present() const { return kind != LemConfig::Wheel::Kind::None; }
    /// getType(): 1 for a drive motor group, 0 for an unpowered wheel.
    int type() const {
        return kind == LemConfig::Wheel::Kind::LeftDrive || kind == LemConfig::Wheel::Kind::RightDrive ? 1 : 0;
    }
    float distance(const Readings& r) const {
        switch (kind) {
            case LemConfig::Wheel::Kind::Pod:
                // vertical: the chassis's forward travel plus the arc from the offset
                return vertical ? r.forwardIn - offset * r.turnRad : -r.rightIn - offset * r.turnRad;
            case LemConfig::Wheel::Kind::LeftDrive:
            case LemConfig::Wheel::Kind::RightDrive: {
                // positions (rotations) * (diameter * pi) * (rpm / cartridge rpm), from wheel degrees
                const float deg = kind == LemConfig::Wheel::Kind::LeftDrive ? r.leftDeg : r.rightDeg;
                return deg / 360 * (diameter * kPi) * (rpm / driveRpm);
            }
            case LemConfig::Wheel::Kind::None: break;
        }
        return 0;
    }
};

/// odom.cpp. Pose theta is radians, compass (0 = +y, clockwise positive).
class Odometry {
public:
    void configure(const LemConfig& cfg) {
        auto wheel = [&](const LemConfig::Wheel& w, bool vertical) {
            TrackingWheel t;
            t.kind = w.kind;
            t.vertical = vertical;
            t.offset = f(w.offset);
            t.diameter = f(w.diameter);
            t.rpm = f(w.rpm);
            t.driveRpm = f(cfg.rpm);
            return t;
        };
        v1_ = wheel(cfg.vertical1, true);
        v2_ = wheel(cfg.vertical2, true);
        h1_ = wheel(cfg.horizontal1, false);
        h2_ = wheel(cfg.horizontal2, false);
        // Chassis::calibrate: empty verticals become the drive sides
        if (!v1_.present()) {
            v1_ = wheel(LemConfig::Wheel{LemConfig::Wheel::Kind::LeftDrive, -cfg.trackWidth / 2, cfg.wheelDiameter, cfg.rpm},
                        true);
        }
        if (!v2_.present()) {
            v2_ = wheel(LemConfig::Wheel{LemConfig::Wheel::Kind::RightDrive, cfg.trackWidth / 2, cfg.wheelDiameter, cfg.rpm},
                        true);
        }
        imu_ = cfg.imu;
    }

    /// calibrate(): sensors read zero from here on.
    void calibrate() {
        prevVertical_ = prevVertical1_ = prevVertical2_ = 0;
        prevHorizontal_ = prevHorizontal1_ = prevHorizontal2_ = 0;
        prevImu_ = 0;
    }

    void setPose(const LPose& p) { pose_ = p; }
    const LPose& pose() const { return pose_; }

    void update(const Readings& r) {
        const float vertical1Raw = v1_.distance(r);
        const float vertical2Raw = v2_.distance(r);
        const float horizontal1Raw = h1_.present() ? h1_.distance(r) : 0;
        const float horizontal2Raw = h2_.present() ? h2_.distance(r) : 0;
        const float imuRaw = imu_ ? degToRad(r.imuDeg) : 0;

        const float deltaVertical1 = vertical1Raw - prevVertical1_;
        const float deltaVertical2 = vertical2Raw - prevVertical2_;
        const float deltaHorizontal1 = horizontal1Raw - prevHorizontal1_;
        const float deltaHorizontal2 = horizontal2Raw - prevHorizontal2_;
        const float deltaImu = imuRaw - prevImu_;
        prevVertical1_ = vertical1Raw;
        prevVertical2_ = vertical2Raw;
        prevHorizontal1_ = horizontal1Raw;
        prevHorizontal2_ = horizontal2Raw;
        prevImu_ = imuRaw;

        // heading: horizontal pair, then unpowered vertical pair, then IMU, then the drive
        float heading = pose_.theta;
        if (h1_.present() && h2_.present())
            heading -= (deltaHorizontal1 - deltaHorizontal2) / (h1_.offset - h2_.offset);
        else if (!v1_.type() && !v2_.type())
            heading -= (deltaVertical1 - deltaVertical2) / (v1_.offset - v2_.offset);
        else if (imu_)
            heading += deltaImu;
        else
            heading -= (deltaVertical1 - deltaVertical2) / (v1_.offset - v2_.offset);
        const float deltaHeading = heading - pose_.theta;
        const float avgHeading = pose_.theta + deltaHeading / 2;

        // prefer unpowered wheels; with none it's the left drive side
        const TrackingWheel* vertical = !v1_.type() ? &v1_ : !v2_.type() ? &v2_ : &v1_;
        const TrackingWheel* horizontal = h1_.present() ? &h1_ : h2_.present() ? &h2_ : nullptr;
        const float rawVertical = vertical->distance(r);
        const float rawHorizontal = horizontal ? horizontal->distance(r) : 0;
        const float verticalOffset = vertical->offset;
        const float horizontalOffset = horizontal ? horizontal->offset : 0;

        const float deltaY = rawVertical - prevVertical_;
        const float deltaX = horizontal ? rawHorizontal - prevHorizontal_ : 0;
        prevVertical_ = rawVertical;
        prevHorizontal_ = rawHorizontal;

        float localX = 0, localY = 0;
        if (deltaHeading == 0) {
            localX = deltaX;
            localY = deltaY;
        } else {
            localX = 2 * std::sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
            localY = 2 * std::sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
        }
        pose_.x += localY * std::sin(avgHeading);
        pose_.y += localY * std::cos(avgHeading);
        pose_.x += localX * -std::cos(avgHeading);
        pose_.y += localX * std::sin(avgHeading);
        pose_.theta = heading;
        // Divergence: odomSpeed / odomLocalSpeed aren't kept. No motion reads them.
    }

private:
    TrackingWheel v1_, v2_, h1_, h2_;
    bool imu_ = true;
    LPose pose_;
    float prevVertical_ = 0, prevVertical1_ = 0, prevVertical2_ = 0;
    float prevHorizontal_ = 0, prevHorizontal1_ = 0, prevHorizontal2_ = 0;
    float prevImu_ = 0;
};

}  // namespace lem

// ======================================================================
/// lemlib::Chassis driving one routine. See the top of the file.
class LemController final : public AutonController {
public:
    explicit LemController(LemConfig cfg = {}) { configure(std::move(cfg)); }

    /// Swap in another robot's settings. Takes ownership; no allocation.
    void configure(LemConfig cfg) {
        cfg_ = std::move(cfg);
        lateralPid_ = lem::Pid(f(cfg_.lateral.kP), f(cfg_.lateral.kI), f(cfg_.lateral.kD),
                               f(cfg_.lateral.windupRange), true);
        angularPid_ = lem::Pid(f(cfg_.angular.kP), f(cfg_.angular.kI), f(cfg_.angular.kD),
                               f(cfg_.angular.windupRange), true);
        // ExitCondition takes an int: smallErrorTimeout is truncated, same as the constructor call
        lateralLargeExit_ = lem::ExitCondition(f(cfg_.lateral.largeError), static_cast<int>(cfg_.lateral.largeErrorTimeout));
        lateralSmallExit_ = lem::ExitCondition(f(cfg_.lateral.smallError), static_cast<int>(cfg_.lateral.smallErrorTimeout));
        angularLargeExit_ = lem::ExitCondition(f(cfg_.angular.largeError), static_cast<int>(cfg_.angular.largeErrorTimeout));
        angularSmallExit_ = lem::ExitCondition(f(cfg_.angular.smallError), static_cast<int>(cfg_.angular.smallErrorTimeout));
        odom_.configure(cfg_);
    }

    const LemConfig& config() const { return cfg_; }

    /// chassis.calibrate() then chassis.setPose(x, y, theta).
    void resetPose(const Pose& p, const SensorSnapshot& s) override {
        zero_ = s;
        odom_.calibrate();
        odom_.setPose({f(p.x), f(p.y), lem::degToRad(f(p.theta))});
        syncPose();
    }

    void setMission(Mission m) override {
        mission_ = std::move(m);
        armed_ = true;
        status_ = {};
        clockUs_ = 0;
        lastMs_ = -1;
        nowMs_ = 0;
        doneMs_ = -1;
        nextOdomMs_ = 0;
        rawDriving_ = false;
        pc_ = 0;
        wait_ = Wait::None;
        wakeMs_ = 0;
        motion_ = {};
        motionRunning_ = motionQueued_ = mutexHeld_ = false;
        distTraveled_ = 0;
        out_ = {};
        status_.pc = 0;
        status_.instr = mission_.empty() ? "done" : mission_.front().name();
    }

    WheelCmd tick(const SensorSnapshot& s, double dtSeconds) override {
        if (!armed_ || status_.done) return {};
        const lem::Readings r = readings(s);
        clockUs_ += static_cast<int64_t>(std::llround(dtSeconds * 1e6));
        const int64_t now = clockUs_ / 1000;
        for (int64_t ms = lastMs_ + 1; ms <= now; ++ms) {
            runMillisecond(ms, r);
            if (status_.done) break;
        }
        lastMs_ = now;
        syncPose();
        status_.missionMs = static_cast<double>(doneMs_ >= 0 ? doneMs_ : now);
        status_.activePc = motion_.active ? motion_.pc : -1;
        status_.busy = motion_.active || motionRunning_ || motionQueued_;
        status_.mode = motion_.active ? modeOf(motion_.in.op) : rawDriving_ ? DriveMode::Raw : DriveMode::Disabled;
        return status_.done ? WheelCmd{} : out_;
    }

    const MotionStatus& status() const override { return status_; }
    const Pose& pose() const override { return pose_; }
    /// chassis.getPose(): degrees, compass.
    Pose lemPose() const { return pose_; }
    float distTraveled() const { return distTraveled_; }

private:
    using LPose = lem::LPose;
    static float f(double v) { return lem::f(v); }

    enum class Wait : uint8_t {
        None,      // running
        Sleep,     // pros::delay
        Queued,    // requestMotionStart: blocked on the motion mutex
        Sync,      // running a motion itself (async = false)
        PollDist,  // waitUntil
        PollDone,  // waitUntilDone
        Finished,  // autonomous() returned
    };

    /// One motion's loop state: the locals of the LemLib function.
    struct Motion {
        bool active = false;
        bool ownedByAuton = false;   // async = false: the auton is the one running it
        int pc = -1;
        Instr in;
        int64_t nextMs = 0;
        int64_t timerStartMs = 0;
        // params, after the function's own fixups (abs, maxSpeed changes when close)
        float maxSpeed = 127, minSpeed = 0, earlyExitRange = 0, horizontalDrift = 0;
        LPose target, lastPose;
        bool close = false, lateralSettled = false, prevSameSide = false;
        float prevLateralOut = 0, prevAngularOut = 0, prevMotorPower = 0;
        std::optional<bool> prevSide;
        float startTheta = 0;
        bool settling = false;
        std::optional<float> prevRawDeltaTheta, prevDeltaTheta;
        // follow
        const std::vector<LemConfig::PathPoint>* path = nullptr;
        LPose lastLookahead;
        float prevVel = 0;
        int iteration = 0;
    };

    // ------------------------------------------------------------ clock
    void runMillisecond(int64_t ms, const lem::Readings& r) {
        nowMs_ = ms;
        if (ms >= nextOdomMs_) {
            odom_.update(r);
            nextOdomMs_ += 10;
        }
        // The motion task, then the auton task; each can wake the other within
        // the same millisecond (a motion ending releases a queued call, an
        // async call starts its task as soon as the auton yields).
        for (int guard = 0; guard < 64; ++guard) {
            bool ran = false;
            if (motion_.active && ms >= motion_.nextMs) {
                stepMotion();
                ran = true;
            }
            if (autonReady()) {
                runAuton();
                ran = true;
            }
            if (!ran) break;
        }
        if (wait_ == Wait::Finished && !motion_.active && !status_.done) {
            status_.done = true;
            status_.instr = "done";
            doneMs_ = ms;
        }
    }

    bool autonReady() const {
        switch (wait_) {
            case Wait::None: return true;
            case Wait::Sleep:
            case Wait::PollDist:
            case Wait::PollDone: return nowMs_ >= wakeMs_;
            case Wait::Queued:
            case Wait::Sync:
            case Wait::Finished: return false;
        }
        return false;
    }

    // ------------------------------------------------------------ the auton task
    void runAuton() {
        // finish whatever we were blocked in
        switch (wait_) {
            case Wait::Sleep:
                wait_ = Wait::None;
                advance();
                break;
            case Wait::PollDist: {
                const float d = f(current().a);
                if (distTraveled_ <= d && distTraveled_ != -1) { wakeMs_ += 10; return; }
                wait_ = Wait::None;
                advance();
                break;
            }
            case Wait::PollDone:
                if (distTraveled_ != -1) { wakeMs_ += 10; return; }
                wait_ = Wait::None;
                advance();
                break;
            default: break;
        }
        for (int guard = 0; guard < 4096 && wait_ == Wait::None; ++guard) {
            if (pc_ >= static_cast<int>(mission_.size())) {
                wait_ = Wait::Finished;
                status_.pc = pc_;
                status_.instr = motion_.active ? "finishing last motion" : "done";
                return;
            }
            execute(current());
        }
    }

    const Instr& current() const { return mission_[static_cast<size_t>(pc_)]; }

    void advance() {
        ++pc_;
        status_.pc = pc_;
        status_.instrElapsedMs = 0;
        if (pc_ < static_cast<int>(mission_.size())) status_.instr = current().name();
    }

    void sleep(int64_t ms) {
        wait_ = Wait::Sleep;
        wakeMs_ = nowMs_ + ms;
    }

    void execute(const Instr& in) {
        using Op = Instr::Op;
        switch (in.op) {
            case Op::Delay:
                sleep(static_cast<int64_t>(in.a));
                return;
            case Op::Action:
                std::strncpy(status_.lastAction, in.label.c_str(), sizeof(status_.lastAction) - 1);
                status_.lastActionPc = pc_;
                status_.lastActionAtMs = static_cast<double>(nowMs_);
                ++status_.actionCount;
                break;
            case Op::LemSetPose:
                odom_.setPose({f(in.a), f(in.b), lem::degToRad(f(in.lem.theta))});
                break;
            case Op::LemTank: {
                // tank() with the drive curve unless disableDriveCurve; ints in, ints to the motors
                const bool raw = in.lem.disableDriveCurve;
                const float l = raw ? f(in.a) : lem::expoCurve(cfg_.throttle, f(in.a));
                const float rr = raw ? f(in.b) : lem::expoCurve(cfg_.throttle, f(in.b));
                move(l, rr);
                rawDriving_ = l != 0 || rr != 0;
                break;
            }
            case Op::LemArcade:
                arcade(static_cast<int>(in.a), static_cast<int>(in.b), in.lem.disableDriveCurve,
                       f(in.lem.desaturateBias));
                break;
            case Op::LemCancelMotion:
                motionRunning_ = false;
                sleep(10);  // "give time for motion to stop"
                return;
            case Op::LemCancelAllMotions:
                motionRunning_ = false;
                motionQueued_ = false;
                sleep(10);
                return;
            case Op::LemWaitUntil:
                wait_ = Wait::PollDist;  // do pros::delay(10); while (...)
                wakeMs_ = nowMs_ + 10;
                return;
            case Op::LemWaitUntilDone:
                wait_ = Wait::PollDone;
                wakeMs_ = nowMs_ + 10;
                return;
            default:
                if (in.lemMotion()) { callMotion(in); return; }
                break;  // EZ calls don't belong here; the parser keeps them out
        }
        advance();
    }

    // requestMotionStart() / endMotion() and the async wrapper every motion starts with
    void callMotion(const Instr& in) {
        if (motionRunning_) motionQueued_ = true;
        else motionRunning_ = true;
        if (mutexHeld_) { wait_ = Wait::Queued; return; }  // resumes in endMotion()
        mutexHeld_ = true;
        acquiredMotion(in);
    }

    void acquiredMotion(const Instr& in) {
        wait_ = Wait::None;
        if (!motionRunning_) {
            // "were all motions cancelled?" LemLib returns here still holding
            // the mutex, so every later motion would block. Kept as is; with
            // one auton task it can't happen.
            advance();
            return;
        }
        if (in.lem.async) {
            // pros::Task task([&]() { moveToPoint(..., false); }); endMotion(); pros::delay(10);
            endMotion();
            sleep(10);
            // the new task's requestMotionStart(): nothing is running now
            motionRunning_ = true;
            mutexHeld_ = true;
            startMotion(in, pc_, false);
            return;
        }
        wait_ = Wait::Sync;
        startMotion(in, pc_, true);
    }

    void endMotion() {
        motionRunning_ = motionQueued_;
        motionQueued_ = false;
        mutexHeld_ = false;
        if (wait_ == Wait::Queued) {  // the queued call gets the mutex
            mutexHeld_ = true;
            acquiredMotion(current());
        }
    }

    // ------------------------------------------------------------ the motion task
    void startMotion(const Instr& in, int pc, bool byAuton) {
        Motion m;
        m.active = true;
        m.ownedByAuton = byAuton;
        m.pc = pc;
        m.in = in;
        m.nextMs = nowMs_;
        m.timerStartMs = nowMs_;
        m.maxSpeed = f(in.lem.maxSpeed);
        m.minSpeed = f(in.lem.minSpeed);
        m.earlyExitRange = f(in.lem.earlyExitRange);
        m.horizontalDrift = f(in.lem.horizontalDrift);
        motion_ = std::move(m);
        status_.motionElapsedMs = 0;
        using Op = Instr::Op;
        switch (in.op) {
            case Op::LemMoveToPoint: initMoveToPoint(); break;
            case Op::LemMoveToPose: initMoveToPose(); break;
            case Op::LemFollow: initFollow(); break;
            default: initTurn(); break;
        }
    }

    bool timerDone() const { return nowMs_ - motion_.timerStartMs >= static_cast<int64_t>(motion_.in.lem.timeoutMs); }

    void stepMotion() {
        ExitReason e = ExitReason::Running;
        using Op = Instr::Op;
        switch (motion_.in.op) {
            case Op::LemMoveToPoint: e = iterateMoveToPoint(); break;
            case Op::LemMoveToPose: e = iterateMoveToPose(); break;
            case Op::LemFollow: e = iterateFollow(); break;
            default: e = iterateTurn(); break;
        }
        status_.motionElapsedMs = static_cast<double>(nowMs_ - motion_.timerStartMs);
        if (e == ExitReason::Running) {
            motion_.nextMs = nowMs_ + 10;
            return;
        }
        // stop the drivetrain, distTraveled = -1, endMotion()
        move(0, 0);
        distTraveled_ = -1;
        status_.lastExit = e;
        status_.interfered = e == ExitReason::Timeout;
        status_.exitPc = motion_.pc;
        ++status_.exitCount;
        const bool byAuton = motion_.ownedByAuton;
        motion_.active = false;
        rawDriving_ = false;
        if (byAuton) wait_ = Wait::None;  // a synchronous call returns to the auton straight away
        endMotion();                      // nothing below may touch motion_: this can start the next one
        if (byAuton) advance();
    }

    // chassis.getPose(radians, standardPos)
    LPose getPose(bool radians = false, bool standardPos = false) const {
        LPose p = odom_.pose();
        if (standardPos) p.theta = kHalfPi - p.theta;
        if (!radians) p.theta = lem::radToDeg(p.theta);
        return p;
    }

    void move(float left, float right) {
        // MotorGroup::move takes an int32_t, so the outputs are truncated.
        // Divergence: a NaN is undefined behaviour there; here it's 0.
        auto motor = [](float v) { return std::isfinite(v) ? std::trunc(clamp(static_cast<double>(v), -127.0, 127.0)) : 0.0; };
        out_.left = motor(left);
        out_.right = motor(right);
    }

    // motions/moveToPoint.cpp
    void initMoveToPoint() {
        Motion& m = motion_;
        m.earlyExitRange = std::fabs(m.earlyExitRange);
        lateralPid_.reset();
        lateralLargeExit_.reset();
        lateralSmallExit_.reset();
        angularPid_.reset();
        m.lastPose = getPose();
        distTraveled_ = 0;
        m.target = LPose(f(m.in.a), f(m.in.b));
        m.target.theta = m.lastPose.angle(m.target);
    }

    ExitReason iterateMoveToPoint() {
        Motion& m = motion_;
        const bool forwards = m.in.lem.forwards;
        if (timerDone()) return ExitReason::Timeout;
        if (!((!lateralSmallExit_.getExit() && !lateralLargeExit_.getExit()) || !m.close))
            return lateralSmallExit_.getExit() ? ExitReason::SmallError : ExitReason::BigError;
        if (!motionRunning_) return ExitReason::Cancelled;

        const LPose pose = getPose(true, true);
        distTraveled_ += pose.distance(m.lastPose);
        m.lastPose = pose;
        const float distTarget = pose.distance(m.target);
        if (distTarget < 7.5f && !m.close) {
            m.close = true;
            m.maxSpeed = std::fmax(std::fabs(m.prevLateralOut), 60.0f);
        }
        // motion chaining: crossed the line through the target, perpendicular to the path
        const bool side = (pose.y - m.target.y) * -std::sin(m.target.theta) <=
                          (pose.x - m.target.x) * std::cos(m.target.theta) + m.earlyExitRange;
        if (!m.prevSide) m.prevSide = side;
        const bool sameSide = side == *m.prevSide;
        if (!sameSide && m.minSpeed != 0) return ExitReason::Passed;
        m.prevSide = side;

        const float adjustedRobotTheta = forwards ? pose.theta : pose.theta + lem::kPi;
        const float angularError = lem::angleError(adjustedRobotTheta, pose.angle(m.target));
        const float lateralError = pose.distance(m.target) * std::cos(lem::angleError(pose.theta, pose.angle(m.target)));
        lateralSmallExit_.update(lateralError, nowMs_);
        lateralLargeExit_.update(lateralError, nowMs_);
        status_.error = static_cast<double>(lateralError);

        float lateralOut = lateralPid_.update(lateralError);
        float angularOut = angularPid_.update(lem::radToDeg(angularError));
        if (m.close) angularOut = 0;
        angularOut = std::clamp(angularOut, -m.maxSpeed, m.maxSpeed);
        angularOut = lem::slew(angularOut, m.prevAngularOut, f(cfg_.angular.slew));
        lateralOut = std::clamp(lateralOut, -m.maxSpeed, m.maxSpeed);
        // slew only while accelerating; it would fight the settle
        if (!m.close) lateralOut = lem::slew(lateralOut, m.prevLateralOut, f(cfg_.lateral.slew));
        if (forwards && !m.close) lateralOut = std::fmax(lateralOut, 0.0f);
        else if (!forwards && !m.close) lateralOut = std::fmin(lateralOut, 0.0f);
        if (forwards && lateralOut < std::fabs(m.minSpeed) && lateralOut > 0) lateralOut = std::fabs(m.minSpeed);
        if (!forwards && -lateralOut < std::fabs(m.minSpeed) && lateralOut < 0) lateralOut = -std::fabs(m.minSpeed);
        m.prevAngularOut = angularOut;
        m.prevLateralOut = lateralOut;
        driveRatioed(lateralOut, angularOut, m.maxSpeed);
        return ExitReason::Running;
    }

    // motions/moveToPose.cpp (boomerang)
    void initMoveToPose() {
        Motion& m = motion_;
        lateralPid_.reset();
        lateralLargeExit_.reset();
        lateralSmallExit_.reset();
        angularPid_.reset();
        angularLargeExit_.reset();
        angularSmallExit_.reset();
        m.target = LPose(f(m.in.a), f(m.in.b), kHalfPi - lem::degToRad(f(m.in.lem.theta)));
        if (!m.in.lem.forwards) m.target.theta = std::fmod(m.target.theta + lem::kPi, 2 * lem::kPi);
        if (m.horizontalDrift == 0) m.horizontalDrift = f(cfg_.horizontalDrift);
        m.lastPose = getPose();
        distTraveled_ = 0;
    }

    ExitReason iterateMoveToPose() {
        Motion& m = motion_;
        const bool forwards = m.in.lem.forwards;
        if (timerDone()) return ExitReason::Timeout;
        if (!((!m.lateralSettled || (!angularLargeExit_.getExit() && !angularSmallExit_.getExit())) || !m.close))
            return angularSmallExit_.getExit() ? ExitReason::SmallError : ExitReason::BigError;
        if (!motionRunning_) return ExitReason::Cancelled;

        const LPose pose = getPose(true, true);
        distTraveled_ += pose.distance(m.lastPose);
        m.lastPose = pose;
        const float distTarget = pose.distance(m.target);
        if (distTarget < 7.5f && !m.close) {
            m.close = true;
            m.maxSpeed = std::fmax(std::fabs(m.prevLateralOut), 60.0f);
        }
        if (lateralLargeExit_.getExit() && lateralSmallExit_.getExit()) m.lateralSettled = true;

        LPose carrot = m.target - LPose(std::cos(m.target.theta), std::sin(m.target.theta)) * f(m.in.lem.lead) * distTarget;
        if (m.close) carrot = m.target;

        const bool robotSide = (pose.y - m.target.y) * -std::sin(m.target.theta) <=
                               (pose.x - m.target.x) * std::cos(m.target.theta) + m.earlyExitRange;
        const bool carrotSide = (carrot.y - m.target.y) * -std::sin(m.target.theta) <=
                                (carrot.x - m.target.x) * std::cos(m.target.theta) + m.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
        if (!sameSide && m.prevSameSide && m.close && m.minSpeed != 0) return ExitReason::Passed;
        m.prevSameSide = sameSide;

        const float adjustedRobotTheta = forwards ? pose.theta : pose.theta + lem::kPi;
        const float angularError = m.close ? lem::angleError(adjustedRobotTheta, m.target.theta)
                                           : lem::angleError(adjustedRobotTheta, pose.angle(carrot));
        float lateralError = pose.distance(carrot);
        // full cos only when settling; otherwise just its sign (maxSlipSpeed handles the rest)
        if (m.close) lateralError *= std::cos(lem::angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= lem::sgn(std::cos(lem::angleError(pose.theta, pose.angle(carrot))));
        lateralSmallExit_.update(lateralError, nowMs_);
        lateralLargeExit_.update(lateralError, nowMs_);
        angularSmallExit_.update(lem::radToDeg(angularError), nowMs_);
        angularLargeExit_.update(lem::radToDeg(angularError), nowMs_);
        status_.error = static_cast<double>(lateralError);

        float lateralOut = lateralPid_.update(lateralError);
        float angularOut = angularPid_.update(lem::radToDeg(angularError));
        angularOut = std::clamp(angularOut, -m.maxSpeed, m.maxSpeed);
        lateralOut = std::clamp(lateralOut, -m.maxSpeed, m.maxSpeed);
        if (!m.close) lateralOut = lem::slew(lateralOut, m.prevLateralOut, f(cfg_.lateral.slew));

        // as fast as it can corner without sliding. LemLib's units don't
        // check out (in, in/s and 9.8 m/s^2); horizontalDrift is a fudge factor.
        const float radius = 1 / std::fabs(lem::curvature(pose, pose.theta, carrot));
        const float maxSlipSpeed = std::sqrt(m.horizontalDrift * radius * 9.8f);
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        // angular wins when both want the motor
        const float overturn = std::fabs(angularOut) + std::fabs(lateralOut) - m.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

        if (forwards && !m.close) lateralOut = std::fmax(lateralOut, 0.0f);
        else if (!forwards && !m.close) lateralOut = std::fmin(lateralOut, 0.0f);
        if (forwards && lateralOut < std::fabs(m.minSpeed) && lateralOut > 0) lateralOut = std::fabs(m.minSpeed);
        if (!forwards && -lateralOut < std::fabs(m.minSpeed) && lateralOut < 0) lateralOut = -std::fabs(m.minSpeed);
        m.prevAngularOut = angularOut;
        m.prevLateralOut = lateralOut;
        driveRatioed(lateralOut, angularOut, m.maxSpeed);
        return ExitReason::Running;
    }

    void driveRatioed(float lateralOut, float angularOut, float maxSpeed) {
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }
        move(leftPower, rightPower);
    }

    // motions/turnToHeading.cpp, turnToPoint.cpp, swingToHeading.cpp, swingToPoint.cpp.
    // Four copies of one loop in LemLib; they differ in the target and the output.
    void initTurn() {
        Motion& m = motion_;
        m.minSpeed = std::fabs(m.minSpeed);
        m.startTheta = getPose().theta;
        distTraveled_ = 0;
        angularLargeExit_.reset();
        angularSmallExit_.reset();
        angularPid_.reset();
    }

    ExitReason iterateTurn() {
        Motion& m = motion_;
        using Op = Instr::Op;
        const Op op = m.in.op;
        if (timerDone()) return ExitReason::Timeout;
        if (angularLargeExit_.getExit()) return ExitReason::BigError;
        if (angularSmallExit_.getExit()) return ExitReason::SmallError;
        if (!motionRunning_) return ExitReason::Cancelled;

        LPose pose = getPose();
        const bool toPoint = op == Op::LemTurnToPoint || op == Op::LemSwingToPoint;
        if (toPoint) pose.theta = m.in.lem.forwards ? std::fmod(pose.theta, 360.0f) : std::fmod(pose.theta - 180, 360.0f);
        else if (op == Op::LemSwingToHeading) pose.theta = std::fmod(pose.theta, 360.0f);
        // turns report degrees turned; never more than 180 (it's an angleError)
        distTraveled_ = std::fabs(lem::angleError(pose.theta, m.startTheta, false));

        float targetTheta = f(m.in.lem.theta);
        if (toPoint) {
            const float deltaX = f(m.in.a) - pose.x;
            const float deltaY = f(m.in.b) - pose.y;
            targetTheta = std::fmod(lem::radToDeg(kHalfPi - std::atan2(deltaY, deltaX)), 360.0f);
        }
        const float rawDeltaTheta = lem::angleError(targetTheta, pose.theta, false);
        if (!m.prevRawDeltaTheta) m.prevRawDeltaTheta = rawDeltaTheta;
        if (lem::sgn(rawDeltaTheta) != lem::sgn(*m.prevRawDeltaTheta)) m.settling = true;
        m.prevRawDeltaTheta = rawDeltaTheta;

        const float deltaTheta = m.settling ? lem::angleError(targetTheta, pose.theta, false)
                                            : lem::angleError(targetTheta, pose.theta, false, m.in.lem.direction);
        // never updated after the first pass: "crossed the target" for chaining
        if (!m.prevDeltaTheta) m.prevDeltaTheta = deltaTheta;
        if (m.minSpeed != 0 && std::fabs(deltaTheta) < m.earlyExitRange) return ExitReason::Passed;
        if (m.minSpeed != 0 && lem::sgn(deltaTheta) != lem::sgn(*m.prevDeltaTheta)) return ExitReason::Passed;

        float motorPower = angularPid_.update(deltaTheta);
        angularLargeExit_.update(deltaTheta, nowMs_);
        angularSmallExit_.update(deltaTheta, nowMs_);
        status_.error = static_cast<double>(deltaTheta);

        if (motorPower > m.maxSpeed) motorPower = m.maxSpeed;
        else if (motorPower < -m.maxSpeed) motorPower = -m.maxSpeed;
        if (std::fabs(deltaTheta) > 20) motorPower = lem::slew(motorPower, m.prevMotorPower, f(cfg_.angular.slew));
        if (motorPower < 0 && motorPower > -m.minSpeed) motorPower = -m.minSpeed;
        else if (motorPower > 0 && motorPower < m.minSpeed) motorPower = m.minSpeed;
        m.prevMotorPower = motorPower;

        if (op == Op::LemTurnToHeading || op == Op::LemTurnToPoint) {
            move(motorPower, -motorPower);
        } else if (m.in.lem.lockedSide == SwingSide::Left) {
            // Divergence: the locked side brakes on HOLD on the brain; the plant has no brake modes
            move(0, -motorPower);
        } else {
            move(motorPower, 0);
        }
        return ExitReason::Running;
    }

    // motions/pursuit.cpp
    void initFollow() {
        Motion& m = motion_;
        auto it = cfg_.paths.find(m.in.lem.path);
        m.path = it == cfg_.paths.end() ? nullptr : &it->second;
        m.lastPose = getPose(true);
        if (m.path && !m.path->empty()) {
            const auto& p0 = m.path->front();
            m.lastLookahead = LPose(f(p0.x), f(p0.y), 0);
        }
        m.prevVel = 0;
        m.iteration = 0;
        distTraveled_ = 0;
    }

    ExitReason iterateFollow() {
        Motion& m = motion_;
        // "No points in path! ... Skipping motion"
        if (!m.path || m.path->empty()) return ExitReason::NoConstants;
        const int timeout = static_cast<int>(m.in.lem.timeoutMs);
        if (!(m.iteration < timeout / 10)) return ExitReason::Timeout;
        if (!motionRunning_) return ExitReason::Cancelled;
        const std::vector<LemConfig::PathPoint>& path = *m.path;

        LPose pose = getPose(true);
        if (!m.in.lem.forwards) pose.theta -= lem::kPi;
        distTraveled_ += pose.distance(m.lastPose);
        m.lastPose = pose;

        // closest point; at the end of the path (speed 0) we're done
        size_t closest = 0;
        float closestDist = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < path.size(); ++i) {
            const float d = pose.distance({f(path[i].x), f(path[i].y)});
            if (d < closestDist) {
                closestDist = d;
                closest = i;
            }
        }
        if (f(path[closest].speed) == 0) return ExitReason::PathEnd;

        const LPose lookahead = lookaheadPoint(m.lastLookahead, pose, path, static_cast<int>(closest),
                                               f(m.in.lem.lookahead));
        m.lastLookahead = lookahead;
        const float curvatureHeading = kHalfPi - pose.theta;
        const float curv = lem::curvature(pose, curvatureHeading, lookahead);

        float targetVel = f(path[closest].speed);
        targetVel = lem::slew(targetVel, m.prevVel, f(cfg_.lateral.slew));
        m.prevVel = targetVel;
        const float tw = f(cfg_.trackWidth);
        float targetLeftVel = targetVel * (2 + curv * tw) / 2;
        float targetRightVel = targetVel * (2 - curv * tw) / 2;
        const float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }
        if (m.in.lem.forwards) move(targetLeftVel, targetRightVel);
        else move(-targetRightVel, -targetLeftVel);
        ++m.iteration;
        return ExitReason::Running;
    }

    /// Where the lookahead circle crosses the path, searching forward from the
    /// closest point and the last lookahead. theta carries the segment index.
    static LPose lookaheadPoint(const LPose& lastLookahead, const LPose& pose,
                                const std::vector<LemConfig::PathPoint>& path, int closest, float lookaheadDist) {
        const int start = std::max(closest, static_cast<int>(lastLookahead.theta));
        for (int i = start; i < static_cast<int>(path.size()) - 1; ++i) {
            const LPose p1(f(path[static_cast<size_t>(i)].x), f(path[static_cast<size_t>(i)].y));
            const LPose p2(f(path[static_cast<size_t>(i) + 1].x), f(path[static_cast<size_t>(i) + 1].y));
            const float t = circleIntersect(p1, p2, pose, lookaheadDist);
            if (t != -1) {
                LPose la = p1.lerp(p2, t);
                la.theta = static_cast<float>(i);
                return la;
            }
        }
        return lastLookahead;  // off the path: keep the last one
    }

    static float circleIntersect(const LPose& p1, const LPose& p2, const LPose& pose, float lookaheadDist) {
        const LPose d = p2 - p1;
        const LPose fv = p1 - pose;
        const float a = d * d;
        const float b = 2 * (fv * d);
        const float c = (fv * fv) - lookaheadDist * lookaheadDist;
        float discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            discriminant = std::sqrt(discriminant);
            const float t1 = (-b - discriminant) / (2 * a);
            const float t2 = (-b + discriminant) / (2 * a);
            if (t2 >= 0 && t2 <= 1) return t2;  // prefer further along
            if (t1 >= 0 && t1 <= 1) return t1;
        }
        return -1;
    }

    // opcontrol.cpp: arcade() as autons use it, for timed pushes
    void arcade(int throttle, int turn, bool disableDriveCurve, float desaturateBias) {
        if (!disableDriveCurve) {
            throttle = static_cast<int>(std::round(lem::expoCurve(cfg_.throttle, static_cast<float>(throttle))));
            turn = static_cast<int>(std::round(lem::expoCurve(cfg_.steer, static_cast<float>(turn))));
        }
        if (std::abs(throttle) + std::abs(turn) > 127) {
            const int oldThrottle = throttle;
            const int oldTurn = turn;
            throttle = static_cast<int>(throttle * (1 - desaturateBias * std::abs(oldTurn / 127.0f)));
            turn = static_cast<int>(turn * (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0f)));
            if (std::abs(turn) + std::abs(throttle) == 126) {
                if (desaturateBias < 0.5f) throttle += lem::sgn(throttle);
                else turn += lem::sgn(turn);
            }
        }
        move(static_cast<float>(throttle + turn), static_cast<float>(throttle - turn));
        rawDriving_ = throttle != 0 || turn != 0;
    }

    // ------------------------------------------------------------ helpers
    lem::Readings readings(const SensorSnapshot& s) const {
        lem::Readings r;
        r.leftDeg = f(s.enc.leftDeg - zero_.enc.leftDeg);
        r.rightDeg = f(s.enc.rightDeg - zero_.enc.rightDeg);
        r.imuDeg = f(s.imuHeadingDeg - zero_.imuHeadingDeg);
        if (s.travel.valid) {
            r.forwardIn = f(s.travel.forwardIn - zero_.travel.forwardIn);
            r.rightIn = f(s.travel.rightIn - zero_.travel.rightIn);
            r.turnRad = f(s.travel.turnRad - zero_.travel.turnRad);
        } else {
            // no ground truth (ROS): tracking wheels see what the drive sees
            const float inPerDeg = f(cfg_.wheelDiameter) * lem::kPi / 360;
            r.forwardIn = (r.leftDeg + r.rightDeg) / 2 * inPerDeg;
            r.turnRad = lem::degToRad(r.imuDeg);
        }
        return r;
    }

    void syncPose() {
        const LPose p = getPose();
        pose_ = {static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.theta)};
    }

    static DriveMode modeOf(Instr::Op op) {
        using Op = Instr::Op;
        switch (op) {
            case Op::LemTurnToHeading:
            case Op::LemTurnToPoint: return DriveMode::Turn;
            case Op::LemSwingToHeading:
            case Op::LemSwingToPoint: return DriveMode::Swing;
            default: return DriveMode::PointToPoint;
        }
    }

    static constexpr float kHalfPi = lem::kPi / 2;

    LemConfig cfg_;
    lem::Odometry odom_;
    lem::Pid lateralPid_{0, 0, 0, 0, true};
    lem::Pid angularPid_{0, 0, 0, 0, true};
    lem::ExitCondition lateralLargeExit_{0, 0}, lateralSmallExit_{0, 0};
    lem::ExitCondition angularLargeExit_{0, 0}, angularSmallExit_{0, 0};

    Mission mission_;
    bool armed_ = false;
    MotionStatus status_;
    SensorSnapshot zero_;
    Pose pose_;
    WheelCmd out_;
    bool rawDriving_ = false;

    int64_t clockUs_ = 0, lastMs_ = -1, nowMs_ = 0, nextOdomMs_ = 0, doneMs_ = -1;
    int pc_ = 0;
    Wait wait_ = Wait::None;
    int64_t wakeMs_ = 0;

    Motion motion_;
    bool motionRunning_ = false, motionQueued_ = false, mutexHeld_ = false;
    float distTraveled_ = 0;   // LemLib starts it at 0, not -1: see waitUntilDone()
};

}  // namespace visbot
