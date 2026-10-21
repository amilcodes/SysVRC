// visbot/plant.hpp: kinematic drivetrain with motor lag, encoder ticks and
// IMU noise. Used by the unit tests, auton_check, and plant_node (the fast,
// deterministic stand-in for Gazebo that CI runs).
#pragma once

#include <cmath>
#include <cstdint>

#include "visbot/constants.hpp"
#include "visbot/geometry.hpp"
#include "visbot/odometry.hpp"

namespace visbot {

struct PlantParams {
    double motorTauSec      = 0.12;  // first-order lag to commanded wheel speed
    double encoderStepDeg   = 0.2;   // ~1800 ticks/rev for a blue V5 motor
    double imuNoiseStdDeg   = 0.05;
    double imuDriftDegPerS  = 0.0;
    double slipFraction     = 0.0;   // 0..1: fraction of wheel travel lost to slip
    /// Top speed as a fraction of nominal. A low battery sags under load.
    double batteryScale     = 1.0;
    /// Per-side gain. Real drive sides never match exactly; 1.03 / 0.97 is
    /// already enough to curve an open-loop drive.
    double leftGain         = 1.0;
    double rightGain        = 1.0;
    /// Encoder degrees the controller sees per real wheel degree. Not 1 when
    /// the code's gear ratio is wrong (RobotSpec::encoderScale).
    double encoderScale     = 1.0;
    /// Field perimeter. Only meaningful with a real field start, since most
    /// routines zero odom wherever the robot happens to be.
    bool walls              = false;
    double fieldHalfIn      = 72.0;   // 12 ft field
    double robotHalfIn      = 7.5;    // half the chassis width
    uint32_t seed           = 0x5157;
};

class DiffDrivePlant {
public:
    explicit DiffDrivePlant(RobotParams rp = {}, PlantParams pp = {}) : rp_(rp), pp_(pp), rng_(pp.seed) {}

    void reset(const Pose& p) {
        truth_ = p;
        vL_ = vR_ = 0.0;
        encL_ = encR_ = 0.0;
        drift_ = 0.0;
        travel_ = {0.0, 0.0, 0.0, true};
    }

    /// Advance by dt with the given command (V5 units, -127..127).
    void step(const WheelCmd& cmd, double dt) {
        const double vmax = rp_.maxWheelSpeedInPerSec() * pp_.batteryScale;
        const double tL = clamp(cmd.left, -127.0, 127.0) / 127.0 * vmax * pp_.leftGain;
        const double tR = clamp(cmd.right, -127.0, 127.0) / 127.0 * vmax * pp_.rightGain;
        const double alpha = 1.0 - std::exp(-dt / pp_.motorTauSec);
        vL_ += (tL - vL_) * alpha;
        vR_ += (tR - vR_) * alpha;

        // slip: the wheels turn more than the chassis moves, and the
        // encoders can't tell
        const double dl = vL_ * dt, dr = vR_ * dt;
        const double d = 0.5 * (dl + dr) * (1.0 - pp_.slipFraction);
        const double dTheta = (dl - dr) / rp_.trackWidthIn;  // rad, CW positive

        const double mid = deg2rad(truth_.theta) + dTheta / 2.0;
        double chord = d;
        if (std::fabs(dTheta) > 1e-9) chord = 2.0 * (d / dTheta) * std::sin(dTheta / 2.0);
        const double x0 = truth_.x, y0 = truth_.y;
        truth_.x += chord * std::sin(mid);
        truth_.y += chord * std::cos(mid);
        truth_.theta += rad2deg(dTheta);   // continuous, like the V5 inertial

        // The wall stops the chassis, not the wheels. Encoders keep counting
        // through a wall push, same as on the real robot.
        if (pp_.walls) {
            const double lim = pp_.fieldHalfIn - pp_.robotHalfIn;
            wallContact_ = std::fabs(truth_.x) > lim || std::fabs(truth_.y) > lim;
            truth_.x = clamp(truth_.x, -lim, lim);
            truth_.y = clamp(truth_.y, -lim, lim);
        }
        // what a tracking wheel would see: the move that actually happened
        const double dx = truth_.x - x0, dy = truth_.y - y0;
        travel_.forwardIn += dx * std::sin(mid) + dy * std::cos(mid);
        travel_.rightIn += dx * std::cos(mid) - dy * std::sin(mid);
        travel_.turnRad += dTheta;

        encL_ += dl / rp_.inchesPerWheelDegree() * pp_.encoderScale;
        encR_ += dr / rp_.inchesPerWheelDegree() * pp_.encoderScale;
        drift_ += pp_.imuDriftDegPerS * dt;
    }

    const Pose& truth() const { return truth_; }
    bool wallContact() const { return wallContact_; }
    double leftSpeed() const { return vL_; }
    double rightSpeed() const { return vR_; }

    /// Sensor readings: quantised encoders, noisy IMU.
    SensorSnapshot sensors() {
        SensorSnapshot s;
        s.enc.leftDeg = quantise(encL_);
        s.enc.rightDeg = quantise(encR_);
        s.imuHeadingDeg = truth_.theta + drift_ + gaussian() * pp_.imuNoiseStdDeg;
        s.travel = travel_;
        return s;
    }

private:
    double quantise(double deg) const {
        if (pp_.encoderStepDeg <= 0) return deg;
        return std::round(deg / pp_.encoderStepDeg) * pp_.encoderStepDeg;
    }
    // xorshift32 + Box-Muller. Cheap, and the same seed gives the same run.
    double uniform() {
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        return (rng_ & 0xFFFFFF) / 16777216.0 + 1e-9;
    }
    double gaussian() {
        const double u1 = uniform(), u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
    }

    RobotParams rp_;
    PlantParams pp_;
    Pose truth_{};
    ChassisTravel travel_{0.0, 0.0, 0.0, true};
    double vL_ = 0.0, vR_ = 0.0, encL_ = 0.0, encR_ = 0.0, drift_ = 0.0;
    bool wallContact_ = false;
    uint32_t rng_;
};

}  // namespace visbot
