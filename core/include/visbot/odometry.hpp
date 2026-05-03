// visbot/odometry.hpp — dead reckoning from the drive encoders + IMU heading,
// the same scheme ez::Drive uses when no dedicated tracking wheels are fitted
// (which is how this robot is configured — see globals.cpp).
//
// Heading comes from the IMU (it is far better than differential encoders);
// translation comes from the mean of left/right wheel travel, integrated
// along the arc the heading change implies rather than a straight line.
#pragma once

#include <cmath>

#include "visbot/constants.hpp"
#include "visbot/geometry.hpp"

namespace visbot {

struct EncoderReading {
    double leftDeg = 0.0;   // cumulative drive-motor encoder, degrees
    double rightDeg = 0.0;
};

/// Everything the controller reads per tick.
struct SensorSnapshot {
    EncoderReading enc;
    double imuHeadingDeg = 0.0;
};

class Odometry {
public:
    explicit Odometry(RobotParams p = {}) : params_(p) {}

    void reset(const Pose& pose, const EncoderReading& enc, double imuHeadingDeg) {
        pose_ = pose;
        prevEnc_ = enc;
        imuOffset_ = wrapDeg(pose.theta - imuHeadingDeg);
        initialised_ = true;
    }

    /// Feed the latest encoder + IMU sample. Returns the updated pose.
    const Pose& update(const EncoderReading& enc, double imuHeadingDeg) {
        if (!initialised_) { reset(pose_, enc, imuHeadingDeg); return pose_; }

        const double k = params_.inchesPerWheelDegree();
        const double dl = (enc.leftDeg - prevEnc_.leftDeg) * k;
        const double dr = (enc.rightDeg - prevEnc_.rightDeg) * k;
        prevEnc_ = enc;

        const double newHeading = wrapDeg(imuHeadingDeg + imuOffset_);
        const double dTheta = deg2rad(wrapDeg(newHeading - pose_.theta));
        const double d = 0.5 * (dl + dr);

        // Arc integration: chord length of an arc with distance d and turn dTheta.
        double chord = d;
        if (std::fabs(dTheta) > 1e-6) chord = 2.0 * (d / dTheta) * std::sin(dTheta / 2.0);

        const double midHeading = deg2rad(pose_.theta) + dTheta / 2.0;
        pose_.x += chord * std::sin(midHeading);   // compass: sin for x (right)
        pose_.y += chord * std::cos(midHeading);   //          cos for y (forward)
        pose_.theta = newHeading;
        distanceTravelled_ += std::fabs(d);
        return pose_;
    }

    const Pose& pose() const { return pose_; }
    void setPose(const Pose& p) { pose_ = p; }
    double distanceTravelled() const { return distanceTravelled_; }

    /// Mean encoder travel in inches — what EZ's drive PID operates on.
    double leftInches(const EncoderReading& e) const { return e.leftDeg * params_.inchesPerWheelDegree(); }
    double rightInches(const EncoderReading& e) const { return e.rightDeg * params_.inchesPerWheelDegree(); }

private:
    RobotParams params_;
    Pose pose_{};
    EncoderReading prevEnc_{};
    double imuOffset_ = 0.0;
    double distanceTravelled_ = 0.0;
    bool initialised_ = false;
};

}  // namespace visbot
