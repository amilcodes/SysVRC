// visbot/motion.hpp: EZ-Template's auton motion code (v3.2.2: pid_tasks.cpp,
// exit_conditions.cpp, set_pid/*.cpp).
//
// On the brain EZ is two threads: a 10 ms PID task that drives whatever motion
// is set, and the auton thread that sets motions and blocks in pid_wait*. So
// an auton is really a flat list:
//
//     chassis.pid_drive_set(36, 127);     -> Instr::driveSet(36, 127)
//     chassis.pid_wait_until(12);         -> Instr::waitUntil(12)
//     leftDoinker.toggle();               -> Instr::act("leftDoinker.toggle()")
//     chassis.pid_wait();                 -> Instr::wait()
//
// MotionController is the PID task plus a program counter over that list.
// Anything that isn't bit-identical to EZ is marked "Divergence".
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "visbot/constants.hpp"
#include "visbot/geometry.hpp"
#include "visbot/odometry.hpp"
#include "visbot/pid.hpp"
#include "visbot/slew.hpp"

namespace visbot {

/// ez::e_angle_behavior
enum class AngleBehavior : uint8_t { Shortest, Longest, CW, CCW, Raw };
enum class DriveDirection : uint8_t { Forward, Reverse };
enum class SwingSide : uint8_t { Left, Right };

/// ez::util::turn_shortest
inline double turnShortest(double target, double current) {
    double error = target - current;
    if (std::fabs(error) < 180.0) return target;
    double nt = target;
    while (error > 180) { nt -= 360; error = nt - current; }
    while (error < -180) { nt += 360; error = nt - current; }
    if (nt - current == 0.0) return current;
    return nt;
}

/// ez::util::turn_longest
inline double turnLongest(double target, double current) {
    const double shortest = turnShortest(target, current);
    return shortest - (360 * sgn(shortest - current));
}

inline double newTurnTarget(double target, double current, AngleBehavior b) {
    switch (b) {
        case AngleBehavior::Shortest: return turnShortest(target, current);
        case AngleBehavior::Longest:  return turnLongest(target, current);
        case AngleBehavior::CW:  { double t = target; while (t < current) t += 360; return t; }
        case AngleBehavior::CCW: { double t = target; while (t > current) t -= 360; return t; }
        case AngleBehavior::Raw: return target;
    }
    return target;
}

/// ez::Drive::pid_speed_max_set. EZ takes abs() of every speed, so a
/// negative speed does nothing; direction always comes from the target.
inline double ezSpeed(double speed) {
    return std::fabs(clamp(speed, -127.0, 127.0));
}

/// ez::util::absolute_angle_to_point: compass bearing from (cx, cy) to (tx, ty).
inline double absoluteAngleToPoint(double tx, double ty, double cx, double cy) {
    return rad2deg(std::atan2(tx - cx, ty - cy));
}

/// ez::util::vector_off_point
inline Pose vectorOffPoint(double added, const Pose& p) {
    return {std::sin(deg2rad(p.theta)) * added + p.x,
            std::cos(deg2rad(p.theta)) * added + p.y,
            p.theta};
}

struct Instr {
    enum class Op : uint8_t {
        DriveSet, TurnSet, TurnToPoint, TurnRelative, SwingSet, OdomSet,  // set a motion
        DriveRaw,                                           // chassis.drive_set(l, r)
        Wait, WaitUntil, WaitQuick, WaitQuickChain, Delay,  // blocking
        SpeedMax, DriveChainConstant, TurnChainConstant,    // immediate
        SlewDriveConstants, OdomLookAhead, OdomReset, Action,
    };

    Op op = Op::Wait;
    double a = 0.0, b = 0.0;
    double speed = 127.0;
    bool slew = true;
    DriveDirection dir = DriveDirection::Forward;
    AngleBehavior behavior = AngleBehavior::Shortest;
    SwingSide side = SwingSide::Left;
    double timeoutMs = 0.0;  // backstop for headless runs, not an EZ feature
    std::string label;       // Action only: the mechanism call, e.g. "mogoClamp.toggle()"
    int sourceLine = 0;      // line in the original autons.cpp, when imported

    // --- motions ---
    static Instr driveSet(double inches, double speed = 110, bool slew = true) {
        Instr i; i.op = Op::DriveSet; i.a = inches; i.speed = speed; i.slew = slew; return i;
    }
    static Instr turnSet(double headingDeg, double speed = 90, AngleBehavior b = AngleBehavior::Shortest) {
        Instr i; i.op = Op::TurnSet; i.a = headingDeg; i.speed = speed; i.behavior = b; i.slew = false; return i;
    }
    /// pid_turn_set({x, y}, fwd|rev, speed): keep turning to face a point.
    static Instr turnToPoint(double x, double y, double speed = 90, DriveDirection d = DriveDirection::Forward,
                             AngleBehavior b = AngleBehavior::Shortest) {
        Instr i; i.op = Op::TurnToPoint; i.a = x; i.b = y; i.speed = speed; i.dir = d; i.behavior = b;
        i.slew = false; return i;
    }
    /// pid_turn_relative_set(delta, speed): turn relative to the last heading target.
    static Instr turnRelative(double deltaDeg, double speed = 90, AngleBehavior b = AngleBehavior::Shortest) {
        Instr i; i.op = Op::TurnRelative; i.a = deltaDeg; i.speed = speed; i.behavior = b; i.slew = false; return i;
    }
    static Instr swingSet(SwingSide s, double headingDeg, double speed = 90, double oppositeSpeed = 0.0) {
        Instr i; i.op = Op::SwingSet; i.side = s; i.a = headingDeg; i.speed = speed; i.b = oppositeSpeed; i.slew = false; return i;
    }
    static Instr odomSet(double x, double y, double speed = 110, DriveDirection d = DriveDirection::Forward) {
        Instr i; i.op = Op::OdomSet; i.a = x; i.b = y; i.speed = speed; i.dir = d; return i;
    }
    /// chassis.drive_set(left, right): open loop until the next motion is set.
    static Instr driveRaw(double left, double right) {
        Instr i; i.op = Op::DriveRaw; i.a = left; i.b = right; return i;
    }
    // --- waits ---
    static Instr wait(double timeoutMs = 5000) {
        Instr i; i.op = Op::Wait; i.timeoutMs = timeoutMs; return i;
    }
    static Instr waitUntil(double v, double timeoutMs = 5000) {
        Instr i; i.op = Op::WaitUntil; i.a = v; i.timeoutMs = timeoutMs; return i;
    }
    static Instr waitQuick(double timeoutMs = 5000) {
        Instr i; i.op = Op::WaitQuick; i.timeoutMs = timeoutMs; return i;
    }
    static Instr waitQuickChain(double timeoutMs = 5000) {
        Instr i; i.op = Op::WaitQuickChain; i.timeoutMs = timeoutMs; return i;
    }
    static Instr delay(double ms) { Instr i; i.op = Op::Delay; i.a = ms; return i; }
    // --- immediate ---
    static Instr speedMax(double s) { Instr i; i.op = Op::SpeedMax; i.speed = s; return i; }
    static Instr driveChainConstant(double v) { Instr i; i.op = Op::DriveChainConstant; i.a = v; return i; }
    static Instr turnChainConstant(double v) { Instr i; i.op = Op::TurnChainConstant; i.a = v; return i; }
    static Instr slewDriveConstants(double dist, double minSpeed) {
        Instr i; i.op = Op::SlewDriveConstants; i.a = dist; i.b = minSpeed; return i;
    }
    static Instr odomLookAhead(double v) { Instr i; i.op = Op::OdomLookAhead; i.a = v; return i; }
    /// chassis.odom_xyt_set(x, y, theta) partway through a routine.
    static Instr odomReset(double x, double y, double thetaDeg) {
        Instr i; i.op = Op::OdomReset; i.a = x; i.b = y; i.speed = thetaDeg; return i;
    }
    static Instr act(std::string label) { Instr i; i.op = Op::Action; i.label = std::move(label); return i; }

    /// The EZ function name, as it appears in autons.cpp and .auton files.
    const char* name() const {
        switch (op) {
            case Op::DriveSet:           return "pid_drive_set";
            case Op::TurnSet:            return "pid_turn_set";
            case Op::TurnToPoint:        return "pid_turn_to_point";
            case Op::TurnRelative:       return "pid_turn_relative_set";
            case Op::SwingSet:           return "pid_swing_set";
            case Op::OdomSet:            return "pid_odom_set";
            case Op::DriveRaw:           return "drive_set";
            case Op::Wait:               return "pid_wait";
            case Op::WaitUntil:          return "pid_wait_until";
            case Op::WaitQuick:          return "pid_wait_quick";
            case Op::WaitQuickChain:     return "pid_wait_quick_chain";
            case Op::Delay:              return "delay";
            case Op::SpeedMax:           return "pid_speed_max_set";
            case Op::DriveChainConstant: return "pid_drive_chain_constant_set";
            case Op::TurnChainConstant:  return "pid_turn_chain_constant_set";
            case Op::SlewDriveConstants: return "slew_drive_constants_set";
            case Op::OdomLookAhead:      return "odom_look_ahead_set";
            case Op::OdomReset:          return "odom_xyt_set";
            case Op::Action:             return "action";
        }
        return "?";
    }

    bool blocking() const {
        return op == Op::Wait || op == Op::WaitUntil || op == Op::WaitQuick ||
               op == Op::WaitQuickChain || op == Op::Delay;
    }
};

using Mission = std::vector<Instr>;

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
};

class MotionController {
public:
    explicit MotionController(RobotParams rp = {}, DriveGains g = {})
        : params_(rp), gains_(g), odom_(rp) {
        leftPid_.setGains(g.drive);        leftPid_.setExit(g.driveExit);
        rightPid_.setGains(g.drive);       rightPid_.setExit(g.driveExit);
        headingPid_.setGains(g.heading);
        turnPid_.setGains(g.turn);         turnPid_.setExit(g.turnExit);
        swingPid_.setGains(g.swing);       swingPid_.setExit(g.swingExit);
        xyPid_.setGains(g.drive);          xyPid_.setExit(g.odomDriveExit);
        aOdomPid_.setGains(g.odomAngular); aOdomPid_.setExit(g.odomTurnExit);
        slewLeft_.setConstants(g.slewDriveDistanceIn, g.slewDriveMinSpeed);
        slewRight_.setConstants(g.slewDriveDistanceIn, g.slewDriveMinSpeed);
        slewTurn_.setConstants(g.slewTurnDistanceDeg, g.slewTurnMinSpeed);
        slewSwing_.setConstants(g.slewSwingDistanceDeg, g.slewSwingMinSpeed);
        chainConstant_ = g.driveChainConstantIn;
    }

    /// chassis.odom_xyt_set(...) + drive_imu_reset()
    void resetPose(const Pose& p, const SensorSnapshot& s) {
        odom_.reset(p, s.enc, s.imuHeadingDeg);
        headingPid_.setTarget(p.theta);
    }

    void setMission(Mission m) {
        mission_ = std::move(m);
        raw_ = {};
        draining_ = false;
        pc_ = -1;
        status_ = {};
        mode_ = DriveMode::Disabled;
        totalMs_ = 0.0;
        advance();
    }

    const Pose& pose() const { return odom_.pose(); }
    const MotionStatus& status() const { return status_; }
    const Mission& mission() const { return mission_; }
    const Odometry& odometry() const { return odom_; }
    DriveMode mode() const { return mode_; }
    double elapsedMs() const { return totalMs_; }

    /// Scale both sides down together when the faster one is over the cap,
    /// like EZ, so saturating doesn't change the curvature.
    static void vectorScale(double& l, double& r, double cap) {
        const double faster = std::fmax(std::fabs(l), std::fabs(r));
        if (faster > cap && faster > 0.0) {
            l *= cap / faster;
            r *= cap / faster;
        }
    }

    /// One control tick.
    WheelCmd tick(const SensorSnapshot& s, double dtSeconds) {
        odom_.update(s.enc, s.imuHeadingDeg);
        sensors_ = s;
        dt_ = dtSeconds;
        totalMs_ += dtSeconds * 1000.0;
        if (status_.done) return {};
        status_.missionMs = totalMs_;

        runProgram();
        status_.mode = mode_;
        if (status_.done) return {};

        status_.motionElapsedMs += dtSeconds * 1000.0;
        WheelCmd cmd;
        switch (mode_) {
            case DriveMode::Drive:        cmd = driveTask(); break;
            case DriveMode::Turn:         cmd = turnTask(); break;
            case DriveMode::Swing:        cmd = swingTask(); break;
            case DriveMode::PointToPoint: cmd = ptpTask(); break;
            case DriveMode::Raw:          cmd = raw_; break;
            case DriveMode::Disabled:     break;
        }
        return cmd.clipped();
    }

private:
    // ===================== program =====================
    const Instr& current() const { return mission_[static_cast<size_t>(pc_)]; }
    bool pastEnd() const { return pc_ >= static_cast<int>(mission_.size()); }

    void advance() {
        ++pc_;
        status_.instrElapsedMs = 0.0;
        waitArmed_ = false;
        if (pastEnd()) {
            status_.pc = pc_;
            // Lots of routines end on a motion with no wait (`set_drive(-30);`
            // as the last line). On the brain the PID task keeps driving it
            // after the auton function returns, so let it finish here too.
            if (mode_ == DriveMode::Drive || mode_ == DriveMode::Turn || mode_ == DriveMode::Swing ||
                mode_ == DriveMode::PointToPoint) {
                draining_ = true;
                drainMs_ = 0.0;
                resetExitLatches();
                status_.instr = "finishing last motion";
                return;
            }
            status_.done = true;
            status_.instr = "done";
            mode_ = DriveMode::Disabled;
            return;
        }
        status_.pc = pc_;
        status_.instr = current().name();
    }

    /// Run immediate instructions until something blocks, like the auton
    /// thread does until it hits a pid_wait.
    void runProgram() {
        if (mission_.empty() || pc_ < 0) return;  // no mission armed yet
        if (draining_) {
            // Divergence: on the brain it'd hold the target until the period
            // ends. We stop when it settles, or after kDrainLimitMs.
            drainMs_ += dt_ * 1000.0;
            const ExitReason e = motionExit(0.0);
            if (e != ExitReason::Running || drainMs_ > kDrainLimitMs) {
                noteExit(e != ExitReason::Running ? e : ExitReason::Timeout);
                draining_ = false;
                status_.done = true;
                status_.instr = "done";
                mode_ = DriveMode::Disabled;
            }
            return;
        }
        for (int guard = 0; guard < 256; ++guard) {  // guard: 256 immediate calls in one tick means a bug
            if (status_.done || draining_ || pastEnd()) return;
            const Instr& in = current();
            if (in.blocking()) {
                status_.instrElapsedMs += dt_ * 1000.0;
                if (!waitArmed_) { armWait(in); waitArmed_ = true; return; }
                if (!waitSatisfied(in)) return;
                advance();
                continue;
            }
            execute(in);
            advance();
        }
    }

    void execute(const Instr& in) {
        switch (in.op) {
            case Instr::Op::DriveSet:           setDrive(in); break;
            case Instr::Op::TurnSet:            setTurn(in); break;
            case Instr::Op::TurnRelative: {
                // relative to the last heading *target*, not the current heading (same as EZ)
                Instr abs = in;
                abs.a = headingPid_.target() + in.a;
                abs.behavior = AngleBehavior::Raw;
                setTurn(abs);
                break;
            }
            case Instr::Op::TurnToPoint:        setTurnToPoint(in); break;
            case Instr::Op::SwingSet:           setSwing(in); break;
            case Instr::Op::OdomSet:            setOdom(in); break;
            case Instr::Op::DriveRaw:
                raw_ = {in.a, in.b};
                startMotion(DriveMode::Raw);
                break;
            case Instr::Op::SpeedMax:           maxSpeed_ = ezSpeed(in.speed); break;
            case Instr::Op::DriveChainConstant: chainConstant_ = std::fabs(in.a); break;
            case Instr::Op::TurnChainConstant:  gains_.turnChainConstantDeg = std::fabs(in.a); break;
            case Instr::Op::SlewDriveConstants:
                gains_.slewDriveDistanceIn = in.a;
                gains_.slewDriveMinSpeed = in.b;
                slewLeft_.setConstants(in.a, in.b);
                slewRight_.setConstants(in.a, in.b);
                break;
            case Instr::Op::OdomLookAhead:      gains_.odomLookAheadIn = in.a; break;
            case Instr::Op::OdomReset:
                resetPose({in.a, in.b, in.speed}, sensors_);
                break;
            case Instr::Op::Action:
                std::strncpy(status_.lastAction, in.label.c_str(), sizeof(status_.lastAction) - 1);
                status_.lastActionPc = pc_;
                status_.lastActionAtMs = totalMs_;
                ++status_.actionCount;
                break;
            default: break;
        }
    }

    void armWait(const Instr& in) {
        resetExitLatches();
        if (in.op == Instr::Op::WaitUntil) {
            // a distance for drive/odom motions, an absolute heading for turns and swings
            if (mode_ == DriveMode::Turn || mode_ == DriveMode::Swing)
                armPassHeading(newTurnTarget(in.a, odom_.pose().theta, AngleBehavior::Shortest));
            else
                armPassDrive(in.a);
        } else if (in.op == Instr::Op::WaitQuick || in.op == Instr::Op::WaitQuickChain) {
            // quick_chain pushes the target out by the chain constant first.
            // Both then wait only until the robot passes the target it was
            // given, so with the chain the PID is still pulling hard when the
            // wait returns. That's where the carried speed comes from.
            if (in.op == Instr::Op::WaitQuickChain) applyChain();
            armPassOriginalTarget();
        } else {
            waitKind_ = WaitKind::ExitCond;
        }
    }

    void armPassOriginalTarget() {
        switch (mode_) {
            case DriveMode::Drive:
                armPassDrive(chainTargetStart_);
                break;
            case DriveMode::Turn:
            case DriveMode::Swing:
                armPassHeading(chainTargetStart_);
                break;
            case DriveMode::PointToPoint:
                waitKind_ = WaitKind::PassPoint;
                waitPointSign_ = sgn(isPastTarget(odomTargetOriginal_));
                break;
            case DriveMode::Disabled:
            case DriveMode::Raw:
                waitKind_ = WaitKind::ExitCond;
                break;
        }
    }

    /// ez::Drive::wait_until_drive: remember which side of the target we're on.
    void armPassDrive(double relativeTarget) {
        waitKind_ = WaitKind::PassDrive;
        waitLTarget_ = lStart_ + relativeTarget;
        waitRTarget_ = rStart_ + relativeTarget;
        waitLSign_ = sgn(waitLTarget_ - leftInches());
        waitRSign_ = sgn(waitRTarget_ - rightInches());
    }

    /// ez::Drive::wait_until_turn_swing, same thing for heading.
    void armPassHeading(double headingTarget) {
        waitKind_ = WaitKind::PassHeading;
        waitHeadingTarget_ = headingTarget;
        waitHeadingSign_ = sgn(headingTarget - odom_.pose().theta);
    }

    bool waitSatisfied(const Instr& in) {
        if (in.op == Instr::Op::Delay) return status_.instrElapsedMs >= in.a;

        if (waitKind_ != WaitKind::ExitCond) {
            bool passed = false;
            switch (waitKind_) {
                case WaitKind::PassDrive:
                    passed = sgn(waitLTarget_ - leftInches()) != waitLSign_ ||
                             sgn(waitRTarget_ - rightInches()) != waitRSign_;
                    break;
                case WaitKind::PassHeading:
                    passed = sgn(waitHeadingTarget_ - odom_.pose().theta) != waitHeadingSign_;
                    break;
                case WaitKind::PassPoint:
                    passed = sgn(isPastTarget(odomTargetOriginal_)) != waitPointSign_;
                    break;
                case WaitKind::ExitCond: break;
            }
            if (passed) { status_.lastExit = ExitReason::Passed; return true; }
            // the motion gave up before reaching it
            const ExitReason e = motionExit(in.timeoutMs);
            if (e != ExitReason::Running) { noteExit(e); return true; }
            return false;
        }

        const ExitReason e = motionExit(in.timeoutMs);
        if (e == ExitReason::Running) return false;
        noteExit(e);
        // motion stays live and keeps holding its target until the next one is set
        return true;
    }

    void noteExit(ExitReason e) {
        status_.lastExit = e;
        if (e == ExitReason::Velocity || e == ExitReason::Timeout) status_.interfered = true;
    }

    /// Exit conditions for the active motion, once per tick.
    ///
    /// Both sides of a two-PID motion have to exit, and each side's result is
    /// latched once it fires (EZ does the same). Don't remove the latch:
    /// exitCondition() resets its timers on success, so polling an exited side
    /// again restarts its clock and the two can starve each other forever.
    ExitReason motionExit(double timeoutMs) {
        auto poll = [&](ExitReason& latch, Pid& pid) {
            if (latch == ExitReason::Running) latch = pid.exitCondition(dt_, timeoutMs);
        };
        auto combine = [](ExitReason a, ExitReason b) {
            if (a == ExitReason::Timeout || b == ExitReason::Timeout) return ExitReason::Timeout;
            if (a != ExitReason::Running && b != ExitReason::Running) return a;
            return ExitReason::Running;
        };
        switch (mode_) {
            case DriveMode::Drive:
                poll(exitA_, leftPid_);
                poll(exitB_, rightPid_);
                return combine(exitA_, exitB_);
            case DriveMode::Turn:
                poll(exitA_, turnPid_);
                return exitA_;
            case DriveMode::Swing:
                poll(exitA_, swingPid_);
                return exitA_;
            case DriveMode::PointToPoint:
                poll(exitA_, xyPid_);
                poll(exitB_, aOdomPid_);
                return combine(exitA_, exitB_);
            case DriveMode::Disabled:
            case DriveMode::Raw:
                return ExitReason::SmallError;  // nothing to wait for
        }
        return ExitReason::Running;
    }

    void resetExitLatches() { exitA_ = exitB_ = ExitReason::Running; }

    void startMotion(DriveMode m) {
        mode_ = m;
        resetExitLatches();
        status_.motionElapsedMs = 0.0;
    }

    // ===================== motion setup =====================
    double leftInches() const { return odom_.leftInches(sensors_.enc); }
    double rightInches() const { return odom_.rightInches(sensors_.enc); }

    void setDrive(const Instr& in) {
        leftPid_.reset(); rightPid_.reset();
        maxSpeed_ = ezSpeed(in.speed);
        lStart_ = leftInches(); rStart_ = rightInches();
        chainTargetStart_ = in.a;
        chainSensorStart_ = lStart_;
        leftPid_.setTarget(lStart_ + in.a);
        rightPid_.setTarget(rStart_ + in.a);
        headingOn_ = true;
        slewLeft_.initialize(in.slew, maxSpeed_, lStart_ + in.a, lStart_);
        slewRight_.initialize(in.slew, maxSpeed_, rStart_ + in.a, rStart_);
        startMotion(DriveMode::Drive);
    }

    void setTurn(const Instr& in) {
        turnPid_.reset();
        turnToPoint_ = false;
        const double target = newTurnTarget(in.a, odom_.pose().theta, in.behavior);
        chainSensorStart_ = odom_.pose().theta;
        chainTargetStart_ = target;
        turnPid_.setTarget(target);
        headingPid_.setTarget(target);  // next drive holds this heading
        maxSpeed_ = ezSpeed(in.speed);
        slewTurn_.initialize(in.slew, maxSpeed_, target, chainSensorStart_);
        startMotion(DriveMode::Turn);
    }

    /// pid_turn_set({x, y}, dir, speed). The heading target gets recomputed
    /// every tick, so it keeps facing the point if it gets bumped.
    void setTurnToPoint(const Instr& in) {
        turnPid_.reset();
        turnPoint_ = {in.a, in.b, 0};
        turnPointDir_ = in.dir;
        turnPointBehavior_ = in.behavior;
        turnPointImuStart_ = odom_.pose().theta;
        const double target = turnToPointTarget();
        chainSensorStart_ = odom_.pose().theta;
        chainTargetStart_ = target;
        turnPid_.setTarget(target);
        headingPid_.setTarget(target);
        maxSpeed_ = ezSpeed(in.speed);
        slewTurn_.initialize(in.slew, maxSpeed_, target, chainSensorStart_);
        turnToPoint_ = true;
        startMotion(DriveMode::Turn);
    }

    /// Divergence: EZ goes through find_point_to_face for reverse; +180 gets the same heading.
    double turnToPointTarget() const {
        const Pose cur = odom_.pose();
        double bearing = absoluteAngleToPoint(turnPoint_.x, turnPoint_.y, cur.x, cur.y);
        if (turnPointDir_ == DriveDirection::Reverse) bearing += 180.0;
        return newTurnTarget(wrapDeg(bearing), turnPointImuStart_, turnPointBehavior_);
    }

    void setSwing(const Instr& in) {
        swingPid_.reset(); leftPid_.reset(); rightPid_.reset();
        const double target = newTurnTarget(in.a, odom_.pose().theta, in.behavior);
        chainSensorStart_ = odom_.pose().theta;
        chainTargetStart_ = target;
        swingPid_.setTarget(target);
        headingPid_.setTarget(target);
        swingSide_ = in.side;
        swingOppositeSpeed_ = in.b;
        // the other side holds still on its drive PID
        lStart_ = leftInches(); rStart_ = rightInches();
        leftPid_.setTarget(lStart_);
        rightPid_.setTarget(rStart_);
        maxSpeed_ = ezSpeed(in.speed);
        slewSwing_.initialize(in.slew, maxSpeed_, target, chainSensorStart_);
        startMotion(DriveMode::Swing);
    }

    void setOdom(const Instr& in) {
        xyPid_.reset(); aOdomPid_.reset(); leftPid_.reset(); rightPid_.reset();
        maxSpeed_ = ezSpeed(in.speed);
        driveDir_ = in.dir;
        odomTarget_ = {in.a, in.b, 0};
        odomTargetOriginal_ = odomTarget_;
        odomStart_ = odom_.pose();
        lStart_ = leftInches(); rStart_ = rightInches();
        fakeCurrent_ = 0.0;
        prevDistToTarget_ = odom_.pose().distanceTo(odomTarget_);
        findPointToFace();
        pastTarget_ = sgn(isPastTarget());
        xyPid_.setTarget(0.0);
        aOdomPid_.setTarget(0.0);
        const int dir = driveDir_ == DriveDirection::Reverse ? -1 : 1;
        const double distToTarget = prevDistToTarget_ * dir;
        slewLeft_.initialize(in.slew, maxSpeed_, distToTarget + lStart_, lStart_);
        slewRight_.initialize(in.slew, maxSpeed_, distToTarget + rStart_, rStart_);
        startMotion(DriveMode::PointToPoint);
    }

    /// ez::Drive::pid_wait_quick_chain: push the target past where we're
    /// going so the robot doesn't slow down for it.
    void applyChain() {
        switch (mode_) {
            case DriveMode::Drive: {
                const double s = chainConstant_ * sgn(chainTargetStart_);
                leftPid_.addToTarget(s);
                rightPid_.addToTarget(s);
                break;
            }
            case DriveMode::Turn:
                turnPid_.addToTarget(gains_.turnChainConstantDeg * sgn(chainTargetStart_ - chainSensorStart_));
                break;
            case DriveMode::Swing:
                swingPid_.addToTarget(gains_.swingChainConstantDeg * sgn(chainTargetStart_ - chainSensorStart_));
                break;
            case DriveMode::PointToPoint: {
                const double angle = absoluteAngleToPoint(odomTarget_.x, odomTarget_.y, odomStart_.x, odomStart_.y);
                Pose t = odomTarget_;
                t.theta = angle;
                odomTarget_ = vectorOffPoint(chainConstant_, t);
                findPointToFace();
                break;
            }
            case DriveMode::Disabled:
            case DriveMode::Raw: break;
        }
    }

    // ===================== EZ PID tasks =====================

    /// ez::Drive::drive_pid_task
    WheelCmd driveTask() {
        const double l = leftInches(), r = rightInches();
        leftPid_.compute(l, dt_);
        rightPid_.compute(r, dt_);
        // no wrapping: target and heading are both continuous
        const double theta = odom_.pose().theta;
        headingPid_.compute(theta, dt_);
        slewLeft_.setMaxSpeed(maxSpeed_);
        slewRight_.setMaxSpeed(maxSpeed_);
        slewLeft_.iterate(l);
        slewRight_.iterate(r);

        double lOut = leftPid_.output(), rOut = rightPid_.output();
        const double cap = std::fmax(slewLeft_.output(), slewRight_.output());
        vectorScale(lOut, rOut, cap);

        const double imuOut = headingOn_ ? headingPid_.output() : 0.0;
        lOut += imuOut;
        rOut -= imuOut;
        vectorScale(lOut, rOut, cap);

        status_.error = 0.5 * (leftPid_.error() + rightPid_.error());
        return {lOut, rOut};
    }

    /// ez::Drive::turn_pid_task
    WheelCmd turnTask() {
        const double theta = odom_.pose().theta;
        if (turnToPoint_) {
            const double target = turnToPointTarget();
            turnPid_.setTarget(target);
            headingPid_.setTarget(target);
        }
        turnPid_.compute(theta, dt_);
        slewTurn_.setMaxSpeed(maxSpeed_);
        slewTurn_.iterate(theta);

        double out = clamp(turnPid_.output(), -slewTurn_.output(), slewTurn_.output());
        // turn_min: cap the speed inside start_i so a big integral can't overshoot the end
        if (gains_.turn.kI != 0.0 && gains_.turnMinSpeed != 0.0 &&
            std::fabs(turnPid_.target()) > gains_.turn.startI &&
            std::fabs(turnPid_.error()) < gains_.turn.startI) {
            out = clamp(out, -gains_.turnMinSpeed, gains_.turnMinSpeed);
        }
        status_.error = turnPid_.error();
        return {out, -out};
    }

    /// ez::Drive::swing_pid_task
    WheelCmd swingTask() {
        const double theta = odom_.pose().theta;
        swingPid_.compute(theta, dt_);
        leftPid_.compute(leftInches(), dt_);
        rightPid_.compute(rightInches(), dt_);
        slewSwing_.setMaxSpeed(maxSpeed_);
        slewSwing_.iterate(theta);

        const double out = clamp(swingPid_.output(), -slewSwing_.output(), slewSwing_.output());
        const double scale = maxSpeed_ != 0.0 ? out / maxSpeed_ : 0.0;
        status_.error = swingPid_.error();

        if (swingSide_ == SwingSide::Left) {
            const double opp = swingOppositeSpeed_ == 0.0 ? rightPid_.output() : swingOppositeSpeed_ * scale;
            return {out, opp};
        }
        const double opp = swingOppositeSpeed_ == 0.0 ? leftPid_.output() : -swingOppositeSpeed_ * scale;
        return {opp, -out};
    }

    /// ez::Drive::find_point_to_face. Aim at a point look_ahead past the
    /// target, so the bearing doesn't swing wildly as the robot gets close.
    void findPointToFace() {
        const Pose cur = odom_.pose();
        Pose target = odomTarget_;
        if (driveDir_ == DriveDirection::Reverse) {
            target.x = cur.x - (target.x - cur.x);
            target.y = cur.y - (target.y - cur.y);
        }
        const double txcx = target.x - cur.x;
        double angle = 0.0;
        if (txcx != 0.0) angle = 90.0 - rad2deg(std::atan((target.y - cur.y) / txcx));
        const Pose ptf1 = vectorOffPoint(gains_.odomLookAheadIn, {target.x, target.y, angle});
        const Pose ptf2 = vectorOffPoint(-gains_.odomLookAheadIn, {target.x, target.y, angle});
        pointToFace_ = cur.distanceTo(ptf1) > cur.distanceTo(ptf2) ? ptf1 : ptf2;
    }

    /// ez::Drive::is_past_target. Sign of the along-track distance to the
    /// target: flips when we drive past it.
    double isPastTarget() const { return isPastTarget(odomTarget_); }

    double isPastTarget(const Pose& target) const {
        const Pose cur = odom_.pose();
        const double fx = cur.x - target.x;
        const double fy = cur.y - target.y;
        const double px = pointToFace_.x - target.x;
        const double py = pointToFace_.y - target.y;
        const double add = driveDir_ == DriveDirection::Reverse ? 180.0 : 0.0;
        const double a = deg2rad(absoluteAngleToPoint(px, py, fx, fy) + add);
        return (fy * std::cos(a)) + (fx * std::sin(a));
    }

    /// ez::Drive::ptp_task
    WheelCmd ptpTask() {
        const Pose cur = odom_.pose();
        slewLeft_.setMaxSpeed(maxSpeed_);
        slewRight_.setMaxSpeed(maxSpeed_);
        slewLeft_.iterate(leftInches());
        slewRight_.iterate(rightInches());
        const double cap = std::fmax(slewLeft_.output(), slewRight_.output());

        const double tempTarget = isPastTarget();
        const int dir = driveDir_ == DriveDirection::Reverse ? -1 : 1;
        const int flipped = sgn(tempTarget) != pastTarget_ ? -1 : 1;

        // Divergence: EZ keeps a fake "current" (xy_delta_fake) for the xy
        // PID. We rebuild it from the change in range to the target, which is
        // the same thing up to odometry noise.
        const double dist = cur.distanceTo(odomTarget_);
        fakeCurrent_ += (prevDistToTarget_ - dist) * (dir * flipped);
        prevDistToTarget_ = dist;
        xyPid_.computeError(std::fabs(tempTarget) * dir * flipped, fakeCurrent_, dt_);

        // bearing comes back wrapped; unwrap against the start heading
        const double aTargetRaw = absoluteAngleToPoint(pointToFace_.x, pointToFace_.y, cur.x, cur.y);
        const double aTarget = newTurnTarget(aTargetRaw, odomStart_.theta, AngleBehavior::Shortest);
        const double aErr = aTarget - cur.theta;
        aOdomPid_.computeError(aErr, cur.theta, dt_);

        // turn bias: slow down while pointed the wrong way, so it turns onto the line first
        double xyOut = clamp(xyPid_.output(), -cap, cap);
        if (gains_.odomTurnBias > 0.0)
            xyOut *= 1.0 - ((1.0 - std::cos(deg2rad(aErr))) / gains_.odomTurnBias);
        double aOut = aOdomPid_.output();

        vectorScale(xyOut, aOut, cap);
        double lOut = xyOut + aOut, rOut = xyOut - aOut;
        vectorScale(lOut, rOut, cap);

        status_.error = dist;
        // keep the drive PIDs updated so pid_wait_until works during odom motions
        leftPid_.compute(leftInches(), dt_);
        rightPid_.compute(rightInches(), dt_);
        return {lOut, rOut};
    }

    // ===================== state =====================
    RobotParams params_;
    DriveGains gains_;
    Odometry odom_;
    Pid leftPid_, rightPid_, headingPid_, turnPid_, swingPid_, xyPid_, aOdomPid_;
    Slew slewLeft_, slewRight_, slewTurn_, slewSwing_;

    Mission mission_;
    int pc_ = -1;
    MotionStatus status_{};
    DriveMode mode_ = DriveMode::Disabled;
    SensorSnapshot sensors_{};
    double dt_ = 0.010, totalMs_ = 0.0;
    WheelCmd raw_{};

    double maxSpeed_ = 127.0;
    bool headingOn_ = true;
    double lStart_ = 0.0, rStart_ = 0.0;
    double chainTargetStart_ = 0.0, chainSensorStart_ = 0.0;
    double chainConstant_ = 3.0;

    bool turnToPoint_ = false;
    Pose turnPoint_{};
    DriveDirection turnPointDir_ = DriveDirection::Forward;
    AngleBehavior turnPointBehavior_ = AngleBehavior::Shortest;
    double turnPointImuStart_ = 0.0;

    SwingSide swingSide_ = SwingSide::Left;
    double swingOppositeSpeed_ = 0.0;

    DriveDirection driveDir_ = DriveDirection::Forward;
    Pose odomTarget_{}, odomTargetOriginal_{}, odomStart_{}, pointToFace_{};
    int pastTarget_ = 0;
    double fakeCurrent_ = 0.0, prevDistToTarget_ = 0.0;

    ExitReason exitA_ = ExitReason::Running, exitB_ = ExitReason::Running;

    // how the current wait finishes: exit conditions, or passing a target
    enum class WaitKind : uint8_t { ExitCond, PassDrive, PassHeading, PassPoint };
    WaitKind waitKind_ = WaitKind::ExitCond;
    bool waitArmed_ = false;
    bool draining_ = false;
    double drainMs_ = 0.0;
    static constexpr double kDrainLimitMs = 5000.0;
    double waitLTarget_ = 0.0, waitRTarget_ = 0.0;
    int waitLSign_ = 1, waitRSign_ = 1;
    double waitHeadingTarget_ = 0.0;
    int waitHeadingSign_ = 1;
    int waitPointSign_ = 0;
};

}  // namespace visbot
