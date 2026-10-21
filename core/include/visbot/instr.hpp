// visbot/instr.hpp: a routine as data. One Instr per drive library call, in
// the order the auton runs them. EZ-Template and LemLib calls share the type so
// the tools around them (auton_check, the report, the sim) don't care which
// library a routine was written for.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace visbot {

/// ez::e_angle_behavior
enum class AngleBehavior : uint8_t { Shortest, Longest, CW, CCW, Raw };
enum class DriveDirection : uint8_t { Forward, Reverse };
enum class SwingSide : uint8_t { Left, Right };

enum class Library : uint8_t { EzTemplate, LemLib };

/// lemlib::AngularDirection
enum class LemDirection : uint8_t { CW, CCW, Auto };

/// The params struct of a LemLib motion, plus the arguments around it.
/// Defaults are LemLib's (lemlib/chassis/chassis.hpp).
struct LemArgs {
    double theta = 0.0;            // target heading, deg (moveToPose, turnToHeading, swingToHeading)
    double timeoutMs = 0.0;
    bool forwards = true;
    double maxSpeed = 127.0;
    double minSpeed = 0.0;
    double earlyExitRange = 0.0;
    double lead = 0.6;             // moveToPose
    double horizontalDrift = 0.0;  // moveToPose; 0 means the drivetrain's
    LemDirection direction = LemDirection::Auto;
    SwingSide lockedSide = SwingSide::Left;
    bool async = true;
    double lookahead = 0.0;        // follow
    std::string path;              // follow: the asset, e.g. "skills1_txt"
    bool disableDriveCurve = false;  // tank / arcade
    double desaturateBias = 0.5;     // arcade
};

struct Instr {
    enum class Op : uint8_t {
        DriveSet, TurnSet, TurnToPoint, TurnRelative, SwingSet, OdomSet,  // set a motion
        DriveRaw,                                           // chassis.drive_set(l, r)
        Wait, WaitUntil, WaitQuick, WaitQuickChain, Delay,  // blocking
        SpeedMax, DriveChainConstant, TurnChainConstant,    // immediate
        SlewDriveConstants, OdomLookAhead, OdomReset, Action,
        // LemLib: chassis.moveToPoint(...) etc. Arguments in `lem`.
        LemMoveToPoint, LemMoveToPose, LemTurnToHeading, LemTurnToPoint,
        LemSwingToHeading, LemSwingToPoint, LemFollow,
        LemWaitUntil, LemWaitUntilDone, LemCancelMotion, LemCancelAllMotions,
        LemSetPose, LemTank, LemArcade,
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
    LemArgs lem;             // LemLib motions; x and y go in a and b

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
            case Op::LemMoveToPoint:     return "moveToPoint";
            case Op::LemMoveToPose:      return "moveToPose";
            case Op::LemTurnToHeading:   return "turnToHeading";
            case Op::LemTurnToPoint:     return "turnToPoint";
            case Op::LemSwingToHeading:  return "swingToHeading";
            case Op::LemSwingToPoint:    return "swingToPoint";
            case Op::LemFollow:          return "follow";
            case Op::LemWaitUntil:       return "waitUntil";
            case Op::LemWaitUntilDone:   return "waitUntilDone";
            case Op::LemCancelMotion:    return "cancelMotion";
            case Op::LemCancelAllMotions: return "cancelAllMotions";
            case Op::LemSetPose:         return "setPose";
            case Op::LemTank:            return "tank";
            case Op::LemArcade:          return "arcade";
        }
        return "?";
    }

    /// EZ: the auton thread stops here until something happens.
    bool blocking() const {
        return op == Op::Wait || op == Op::WaitUntil || op == Op::WaitQuick ||
               op == Op::WaitQuickChain || op == Op::Delay;
    }

    /// A LemLib call that starts a motion (and so takes the motion mutex).
    bool lemMotion() const {
        return op == Op::LemMoveToPoint || op == Op::LemMoveToPose || op == Op::LemTurnToHeading ||
               op == Op::LemTurnToPoint || op == Op::LemSwingToHeading || op == Op::LemSwingToPoint ||
               op == Op::LemFollow;
    }
};

using Mission = std::vector<Instr>;


/// What a LemLib project passes to lemlib::Chassis. Defaults are the values in
/// LemLib's example project, used when the importer can't find the real ones.
struct LemConfig {
    /// lemlib::ControllerSettings, same argument order.
    struct Controller {
        double kP, kI, kD, windupRange, smallError, smallErrorTimeout, largeError, largeErrorTimeout, slew;
    };
    /// lemlib::TrackingWheel. Pod = an unpowered wheel on a rotation sensor or
    /// encoder; LeftDrive/RightDrive = a drive motor group used as one.
    struct Wheel {
        enum class Kind : uint8_t { None, Pod, LeftDrive, RightDrive };
        Kind kind = Kind::None;
        double offset = 0.0;       // in; vertical: + is right of centre, horizontal: + is forward
        double diameter = 2.75;
        double rpm = 0.0;          // drive wheels only
    };
    /// lemlib::ExpoDriveCurve. (0, 0, 1) is a straight line.
    struct Curve {
        double deadband = 0.0, minOutput = 0.0, gain = 1.0;
    };
    struct PathPoint {
        double x, y, speed;
    };

    // lemlib::Drivetrain
    double trackWidth = 10.0;
    double wheelDiameter = 4.0;
    double rpm = 360.0;
    double horizontalDrift = 2.0;
    Controller lateral{10, 0, 3, 3, 1, 100, 3, 500, 20};
    Controller angular{2, 0, 10, 3, 1, 100, 3, 500, 0};
    // lemlib::OdomSensors. Empty verticals get the drive sides, like Chassis::calibrate.
    Wheel vertical1, vertical2, horizontal1, horizontal2;
    bool imu = true;
    Curve throttle, steer;
    /// ASSET(...) files for follow(), by asset name.
    std::map<std::string, std::vector<PathPoint>> paths;
};

}  // namespace visbot
