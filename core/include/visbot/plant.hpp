// visbot/plant.hpp — a fast kinematic model of the drivebase, with V5-like
// motor lag, encoder quantisation and IMU noise.
//
// Two jobs:
//   1. closed-loop unit tests of the controllers with no ROS or Gazebo, and
//   2. the `visbot_plant` node, a lightweight stand-in for Gazebo when you
//      only care about control timing (CI, laptops without a GPU).
// Gazebo remains the high-fidelity backend; this one is deterministic.
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
    }

    /// Advance by dt with the given command (V5 units, -127..127).
    void step(const WheelCmd& cmd, double dt) {
        const double vmax = rp_.maxWheelSpeedInPerSec();
        const double tL = clamp(cmd.left, -127.0, 127.0) / 127.0 * vmax;
        const double tR = clamp(cmd.right, -127.0, 127.0) / 127.0 * vmax;
        const double alpha = 1.0 - std::exp(-dt / pp_.motorTauSec);
        vL_ += (tL - vL_) * alpha;
        vR_ += (tR - vR_) * alpha;

        const double dl = vL_ * dt, dr = vR_ * dt;
        const double d = 0.5 * (dl + dr) * (1.0 - pp_.slipFraction);
        const double dTheta = (dl - dr) / rp_.trackWidthIn;  // rad, CW positive (left faster => turn right)

        const double mid = deg2rad(truth_.theta) + dTheta / 2.0;
        double chord = d;
        if (std::fabs(dTheta) > 1e-9) chord = 2.0 * (d / dTheta) * std::sin(dTheta / 2.0);
        truth_.x += chord * std::sin(mid);
        truth_.y += chord * std::cos(mid);
        truth_.theta = wrapDeg(truth_.theta + rad2deg(dTheta));

        encL_ += dl / rp_.inchesPerWheelDegree();
        encR_ += dr / rp_.inchesPerWheelDegree();
        drift_ += pp_.imuDriftDegPerS * dt;
    }

    const Pose& truth() const { return truth_; }
    double leftSpeed() const { return vL_; }
    double rightSpeed() const { return vR_; }

    /// What the sensors report (quantised / noisy).
    SensorSnapshot sensors() {
        SensorSnapshot s;
        s.enc.leftDeg = quantise(encL_);
        s.enc.rightDeg = quantise(encR_);
        s.imuHeadingDeg = wrapDeg(truth_.theta + drift_ + gaussian() * pp_.imuNoiseStdDeg);
        return s;
    }

private:
    double quantise(double deg) const {
        if (pp_.encoderStepDeg <= 0) return deg;
        return std::round(deg / pp_.encoderStepDeg) * pp_.encoderStepDeg;
    }
    // xorshift32 + Box-Muller: deterministic, no <random> in the hot path.
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
    double vL_ = 0.0, vR_ = 0.0, encL_ = 0.0, encR_ = 0.0, drift_ = 0.0;
    uint32_t rng_;
};

}  // namespace visbot
