// visbot/motion.hpp — EZ-Template's autonomous motion engine, ported.
//
// EZ's model is two cooperating threads: a 10 ms PID task that drives the
// motors from whatever motion is currently "set", and the auton thread that
// sets motions and then blocks in pid_wait* calls. autons.cpp reads as a flat
// sequence of those two kinds of call:
//
//     chassis.pid_drive_set(36, 127);     -> Instr::driveSet(36, 127)
//     chassis.pid_wait_until(12);         -> Instr::waitUntil(12)
//     leftDoinker.toggle();               -> Instr::act(ActionId::DoinkerLeft)
//     chassis.pid_speed_max_set(70);      -> Instr::speedMax(70)
//     chassis.pid_wait();                 -> Instr::wait()
//
// So a Mission here is that same flat list, and MotionController is the PID
// task plus a program counter: non-blocking instructions execute immediately,
// blocking ones hold until their condition is met. A real routine can then be
// transcribed nearly line for line — see missions::worldsMogoRush.
//
// Ported against EZ-Template v3.2.2 (pid_tasks.cpp, exit_conditions.cpp,
// set_pid/*.cpp). Places where this is not bit-identical are marked
// `Divergence:`.
#pragma once

#include <cstdint>
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

/// ez::util::absolute_angle_to_point — compass bearing from current to target.
inline double absoluteAngleToPoint(double tx, double ty, double cx, double cy) {
    return rad2deg(std::atan2(tx - cx, ty - cy));
}

/// ez::util::vector_off_point
inline Pose vectorOffPoint(double added, const Pose& p) {
    return {std::sin(deg2rad(p.theta)) * added + p.x,
            std::cos(deg2rad(p.theta)) * added + p.y,
            p.theta};
}

/// Mechanism hooks. The sim models no intake or clamp, but *when* a routine
/// fires them is half of what makes it a routine, so they are recorded and
/// surfaced on the dashboard rather than silently dropped.
enum class ActionId : uint8_t {
    None, IntakeIn, IntakeOut, IntakeStop, MogoClamp, MogoRelease,
    DoinkerLeft, DoinkerRight, Ladybrown, ColorSortOn, ColorSortOff,
};

inline const char* toString(ActionId a) {
    switch (a) {
        case ActionId::None:         return "none";
        case ActionId::IntakeIn:     return "intake_in";
        case ActionId::IntakeOut:    return "intake_out";
        case ActionId::IntakeStop:   return "intake_stop";
        case ActionId::MogoClamp:    return "mogo_clamp";
        case ActionId::MogoRelease:  return "mogo_release";
        case ActionId::DoinkerLeft:  return "doinker_left";
        case ActionId::DoinkerRight: return "doinker_right";
        case ActionId::Ladybrown:    return "ladybrown";
        case ActionId::ColorSortOn:  return "color_sort_on";
        case ActionId::ColorSortOff: return "color_sort_off";
    }
    return "?";
}

struct Instr {
    enum class Op : uint8_t {
        DriveSet, TurnSet, SwingSet, OdomSet,     // set a motion (non-blocking)
        Wait, WaitUntil, WaitQuickChain, Delay,   // blocking
        SpeedMax, DriveChainConstant, Action,     // immediate
    };

    Op op = Op::Wait;
    double a = 0.0, b = 0.0;
    double speed = 127.0;
    bool slew = true;
    DriveDirection dir = DriveDirection::Forward;
    AngleBehavior behavior = AngleBehavior::Shortest;
    SwingSide side = SwingSide::Left;
    ActionId action = ActionId::None;
    double timeoutMs = 0.0;  // backstop for headless runs, not an EZ feature

    // --- motions ---
    static Instr driveSet(double inches, double speed = 110, bool slew = true) {
        Instr i; i.op = Op::DriveSet; i.a = inches; i.speed = speed; i.slew = slew; return i;
    }
    static Instr turnSet(double headingDeg, double speed = 90, AngleBehavior b = AngleBehavior::Shortest) {
        Instr i; i.op = Op::TurnSet; i.a = headingDeg; i.speed = speed; i.behavior = b; i.slew = false; return i;
    }
    static Instr swingSet(SwingSide s, double headingDeg, double speed = 90, double oppositeSpeed = 0.0) {
        Instr i; i.op = Op::SwingSet; i.side = s; i.a = headingDeg; i.speed = speed; i.b = oppositeSpeed; i.slew = false; return i;
    }
    static Instr odomSet(double x, double y, double speed = 110, DriveDirection d = DriveDirection::Forward) {
        Instr i; i.op = Op::OdomSet; i.a = x; i.b = y; i.speed = speed; i.dir = d; return i;
    }
    // --- waits ---
    static Instr wait(double timeoutMs = 5000) {
        Instr i; i.op = Op::Wait; i.timeoutMs = timeoutMs; return i;
    }
    static Instr waitUntil(double v, double timeoutMs = 5000) {
        Instr i; i.op = Op::WaitUntil; i.a = v; i.timeoutMs = timeoutMs; return i;
    }
    static Instr waitQuickChain(double timeoutMs = 5000) {
        Instr i; i.op = Op::WaitQuickChain; i.timeoutMs = timeoutMs; return i;
    }
    static Instr delay(double ms) { Instr i; i.op = Op::Delay; i.a = ms; return i; }
    // --- immediate ---
    static Instr speedMax(double s) { Instr i; i.op = Op::SpeedMax; i.speed = s; return i; }
    static Instr driveChainConstant(double v) { Instr i; i.op = Op::DriveChainConstant; i.a = v; return i; }
    static Instr act(ActionId id, double arg = 0) { Instr i; i.op = Op::Action; i.action = id; i.a = arg; return i; }

    const char* name() const {
        switch (op) {
            case Op::DriveSet:           return "drive_set";
            case Op::TurnSet:            return "turn_set";
            case Op::SwingSet:           return "swing_set";
            case Op::OdomSet:            return "odom_set";
            case Op::Wait:               return "wait";
            case Op::WaitUntil:          return "wait_until";
            case Op::WaitQuickChain:     return "wait_quick_chain";
            case Op::Delay:              return "delay";
            case Op::SpeedMax:           return "speed_max";
            case Op::DriveChainConstant: return "chain_const";
            case Op::Action:             return "action";
        }
        return "?";
    }

    bool blocking() const {
        return op == Op::Wait || op == Op::WaitUntil || op == Op::WaitQuickChain || op == Op::Delay;
    }
};

using Mission = std::vector<Instr>;

enum class DriveMode : uint8_t { Disabled, Drive, Turn, Swing, PointToPoint };

inline const char* toString(DriveMode m) {
    switch (m) {
        case DriveMode::Disabled:     return "disabled";
        case DriveMode::Drive:        return "drive";
        case DriveMode::Turn:         return "turn";
        case DriveMode::Swing:        return "swing";
        case DriveMode::PointToPoint: return "point_to_point";
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
    bool done = false;
    bool interfered = false;      // ez::interfered — exited on velocity/timeout
    ActionId lastAction = ActionId::None;
    double lastActionArg = 0.0;
    double lastActionAtMs = -1.0;
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

    /// One control tick.
    WheelCmd tick(const SensorSnapshot& s, double dtSeconds) {
        odom_.update(s.enc, s.imuHeadingDeg);
        sensors_ = s;
        dt_ = dtSeconds;
        totalMs_ += dtSeconds * 1000.0;
        if (status_.done) return {};

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
            case DriveMode::Disabled:     break;
        }
        return cmd.clipped();
    }

private:
    // ===================== program =====================
    void advance() {
        ++pc_;
        status_.instrElapsedMs = 0.0;
        waitArmed_ = false;
        if (pc_ >= static_cast<int>(mission_.size())) {
            status_.done = true;
            status_.pc = pc_;
            status_.instr = "done";
            mode_ = DriveMode::Disabled;
            return;
        }
        status_.pc = pc_;
        status_.instr = mission_[pc_].name();
    }

    /// The auton thread races ahead through immediate calls until it hits a
    /// pid_wait; that is exactly this loop.
    void runProgram() {
        if (mission_.empty() || pc_ < 0) return;  // no mission armed yet
        for (int guard = 0; guard < 256; ++guard) {
            if (status_.done) return;
            const Instr& in = mission_[pc_];
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
            case Instr::Op::SwingSet:           setSwing(in); break;
            case Instr::Op::OdomSet:            setOdom(in); break;
            case Instr::Op::SpeedMax:           maxSpeed_ = in.speed; break;
            case Instr::Op::DriveChainConstant: chainConstant_ = std::fabs(in.a); break;
            case Instr::Op::Action:
                status_.lastAction = in.action;
                status_.lastActionArg = in.a;
                status_.lastActionAtMs = totalMs_;
                ++actionCount_;
                break;
            default: break;
        }
    }

    void armWait(const Instr& in) {
        resetExitLatches();
        if (in.op == Instr::Op::WaitUntil) {
            armPassDrive(in.a);
        } else if (in.op == Instr::Op::WaitQuickChain) {
            // ez::pid_wait_quick_chain extends the PID target by the chain
            // constant and then calls pid_wait_quick, which waits only until
            // the robot passes the ORIGINAL target. The loop is still pulling
            // hard toward the extended target when the wait returns — that is
            // what carries speed into the next motion. Waiting on the extended
            // target's exit conditions instead just moves the stopping point.
            applyChain();
            switch (mode_) {
                case DriveMode::Drive:        armPassDrive(chainTargetStart_); break;
                case DriveMode::Turn:
                case DriveMode::Swing:        armPassHeading(chainTargetStart_); break;
                case DriveMode::PointToPoint: waitKind_ = WaitKind::PassPoint;
                                              waitPointSign_ = sgn(isPastTarget(odomTargetOriginal_));
                                              break;
                case DriveMode::Disabled:     waitKind_ = WaitKind::ExitCond; break;
            }
        } else {
            waitKind_ = WaitKind::ExitCond;
        }
    }

    /// ez::Drive::wait_until_drive — remember the sign of the error to the
    /// intermediate target so we can tell when we pass it.
    void armPassDrive(double relativeTarget) {
        waitKind_ = WaitKind::PassDrive;
        waitLTarget_ = lStart_ + relativeTarget;
        waitRTarget_ = rStart_ + relativeTarget;
        waitLSign_ = sgn(waitLTarget_ - leftInches());
        waitRSign_ = sgn(waitRTarget_ - rightInches());
    }

    /// ez::Drive::wait_until_turn_swing — same idea against the heading.
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
            if (passed) return true;
            // Failsafe: the motion ended before we ever got there.
            const ExitReason e = motionExit(in.timeoutMs);
            if (e != ExitReason::Running) { noteExit(e); return true; }
            return false;
        }

        const ExitReason e = motionExit(in.timeoutMs);
        if (e == ExitReason::Running) return false;
        noteExit(e);
        // The motion stays live: EZ's PID task keeps holding its target until
        // the next motion is set, which is what carries momentum through a
        // chained hand-off. tick() stops driving once the mission is done.
        return true;
    }

    void noteExit(ExitReason e) {
        status_.lastExit = e;
        if (e == ExitReason::Velocity || e == ExitReason::Timeout) status_.interfered = true;
    }

    /// Evaluate the active motion's exit conditions once per tick.
    ///
    /// Both halves of a two-PID motion must exit, and each half's result is
    /// latched once it fires. EZ does the same
    /// (`left_exit = left_exit != RUNNING ? left_exit : ...`) and it is not
    /// optional: PID::exit_condition resets its own timers when it succeeds,
    /// so re-polling a side that has already exited restarts its clock and
    /// the two sides can starve each other indefinitely.
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
                return ExitReason::SmallError;  // nothing to wait for
        }
        return ExitReason::Running;
    }

    void resetExitLatches() { exitA_ = exitB_ = ExitReason::Running; }

    // ===================== motion setup =====================
    double leftInches() const { return odom_.leftInches(sensors_.enc); }
    double rightInches() const { return odom_.rightInches(sensors_.enc); }

    void setDrive(const Instr& in) {
        leftPid_.reset(); rightPid_.reset();
        maxSpeed_ = in.speed;
        lStart_ = leftInches(); rStart_ = rightInches();
        chainTargetStart_ = in.a;
        chainSensorStart_ = lStart_;
        leftPid_.setTarget(lStart_ + in.a);
        rightPid_.setTarget(rStart_ + in.a);
        headingOn_ = true;
        slewLeft_.initialize(in.slew, maxSpeed_, lStart_ + in.a, lStart_);
        slewRight_.initialize(in.slew, maxSpeed_, rStart_ + in.a, rStart_);
        mode_ = DriveMode::Drive;
        resetExitLatches();
        status_.motionElapsedMs = 0.0;
    }

    void setTurn(const Instr& in) {
        turnPid_.reset();
        const double target = newTurnTarget(in.a, odom_.pose().theta, in.behavior);
        chainSensorStart_ = odom_.pose().theta;
        chainTargetStart_ = target;
        turnPid_.setTarget(target);
        headingPid_.setTarget(target);  // the next drive holds this heading
        maxSpeed_ = in.speed;
        slewTurn_.initialize(in.slew, maxSpeed_, target, chainSensorStart_);
        mode_ = DriveMode::Turn;
        resetExitLatches();
        status_.motionElapsedMs = 0.0;
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
        // The stationary side holds position through its drive PID.
        lStart_ = leftInches(); rStart_ = rightInches();
        leftPid_.setTarget(lStart_);
        rightPid_.setTarget(rStart_);
        maxSpeed_ = in.speed;
        slewSwing_.initialize(in.slew, maxSpeed_, target, chainSensorStart_);
        mode_ = DriveMode::Swing;
        resetExitLatches();
        status_.motionElapsedMs = 0.0;
    }

    void setOdom(const Instr& in) {
        xyPid_.reset(); aOdomPid_.reset(); leftPid_.reset(); rightPid_.reset();
        maxSpeed_ = in.speed;
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
        mode_ = DriveMode::PointToPoint;
        resetExitLatches();
        status_.motionElapsedMs = 0.0;
    }

    /// ez::Drive::pid_wait_quick_chain — push the target past where we're
    /// going so the robot carries its momentum into the next motion instead
    /// of settling on the spot.
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
            case DriveMode::Disabled: break;
        }
    }

    // ===================== EZ PID tasks =====================

public:
    /// Scale both sides down together when the faster one exceeds the cap.
    /// EZ does this rather than clamping each side independently, so the L/R
    /// ratio — and therefore the path curvature — survives saturation.
    static void vectorScale(double& l, double& r, double cap) {
        const double faster = std::fmax(std::fabs(l), std::fabs(r));
        if (faster > cap && faster > 0.0) {
            l *= cap / faster;
            r *= cap / faster;
        }
    }

private:
    /// ez::Drive::drive_pid_task
    WheelCmd driveTask() {
        const double l = leftInches(), r = rightInches();
        leftPid_.compute(l, dt_);
        rightPid_.compute(r, dt_);
        // EZ: headingPID.compute(drive_imu_get()) — no wrapping, because
        // both the target and the measurement are continuous.
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
        turnPid_.compute(theta, dt_);
        slewTurn_.setMaxSpeed(maxSpeed_);
        slewTurn_.iterate(theta);

        double out = clamp(turnPid_.output(), -slewTurn_.output(), slewTurn_.output());
        // EZ caps the speed once inside start_i so the tail of a turn isn't
        // driven by a large accumulated integral.
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

    /// ez::Drive::find_point_to_face — two points `look_ahead` either side of
    /// the target along the approach line. Steering at the far one keeps the
    /// bearing stable as the robot closes in, instead of spinning when the
    /// target is underfoot.
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

    /// ez::Drive::is_past_target — project the displacement from the target
    /// into the frame of the approach bearing; the sign of the along-track
    /// component says whether we've driven past it.
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

        // Divergence: EZ integrates a synthetic "current" from its own
        // per-tick change in range (xy_delta_fake) and runs the xy PID
        // against that. We rebuild the same quantity from the measured change
        // in range to the target — equal to within odometry noise, but not
        // bit-identical to EZ's internal bookkeeping.
        const double dist = cur.distanceTo(odomTarget_);
        fakeCurrent_ += (prevDistToTarget_ - dist) * (dir * flipped);
        prevDistToTarget_ = dist;
        xyPid_.computeError(std::fabs(tempTarget) * dir * flipped, fakeCurrent_, dt_);

        // The bearing comes back wrapped; reconcile it against the heading
        // the motion started at so the error stays continuous (ez: ptp_task).
        const double aTargetRaw = absoluteAngleToPoint(pointToFace_.x, pointToFace_.y, cur.x, cur.y);
        const double aTarget = newTurnTarget(aTargetRaw, odomStart_.theta, AngleBehavior::Shortest);
        const double aErr = aTarget - cur.theta;
        aOdomPid_.computeError(aErr, cur.theta, dt_);

        // Turn bias: give up forward speed in proportion to how far off the
        // bearing is, so the robot turns onto the line before sprinting down it.
        double xyOut = clamp(xyPid_.output(), -cap, cap);
        if (gains_.odomTurnBias > 0.0)
            xyOut *= 1.0 - ((1.0 - std::cos(deg2rad(aErr))) / gains_.odomTurnBias);
        double aOut = aOdomPid_.output();

        vectorScale(xyOut, aOut, cap);
        double lOut = xyOut + aOut, rOut = xyOut - aOut;
        vectorScale(lOut, rOut, cap);

        status_.error = dist;
        // EZ keeps the drive PIDs fed so pid_wait_until still works in odom mode.
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
    int actionCount_ = 0;

    double maxSpeed_ = 127.0;
    bool headingOn_ = true;
    double lStart_ = 0.0, rStart_ = 0.0;
    double chainTargetStart_ = 0.0, chainSensorStart_ = 0.0;
    double chainConstant_ = 3.0;

    SwingSide swingSide_ = SwingSide::Left;
    double swingOppositeSpeed_ = 0.0;

    DriveDirection driveDir_ = DriveDirection::Forward;
    Pose odomTarget_{}, odomTargetOriginal_{}, odomStart_{}, pointToFace_{};
    int pastTarget_ = 0;
    double fakeCurrent_ = 0.0, prevDistToTarget_ = 0.0;

    ExitReason exitA_ = ExitReason::Running, exitB_ = ExitReason::Running;

    /// How the active blocking instruction decides it is finished: either the
    /// motion's exit conditions, or passing a value while the motion runs on.
    enum class WaitKind : uint8_t { ExitCond, PassDrive, PassHeading, PassPoint };
    WaitKind waitKind_ = WaitKind::ExitCond;
    bool waitArmed_ = false;
    double waitLTarget_ = 0.0, waitRTarget_ = 0.0;
    int waitLSign_ = 1, waitRSign_ = 1;
    double waitHeadingTarget_ = 0.0;
    int waitHeadingSign_ = 1;
    int waitPointSign_ = 0;
};

}  // namespace visbot
