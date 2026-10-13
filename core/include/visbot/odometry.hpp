// visbot/odometry.hpp: drive encoders + IMU dead reckoning, same as ez::Drive
// without tracking wheels (which is how our robot is set up, see globals.cpp).
// Heading from the IMU, distance from the average of both sides, integrated
// along an arc.
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
        imuContinuous_ = imuHeadingDeg;
        imuOffset_ = pose.theta - imuHeadingDeg;
        initialised_ = true;
    }

    /// Feed the latest encoder + IMU sample. Returns the updated pose.
    const Pose& update(const EncoderReading& enc, double imuHeadingDeg) {
        if (!initialised_) { reset(pose_, enc, imuHeadingDeg); return pose_; }

        const double k = params_.inchesPerWheelDegree();
        const double dl = (enc.leftDeg - prevEnc_.leftDeg) * k;
        const double dr = (enc.rightDeg - prevEnc_.rightDeg) * k;
        prevEnc_ = enc;

        // Gazebo's yaw is wrapped, the V5's isn't. Handle both.
        imuContinuous_ = unwrapInto(imuContinuous_, wrapDeg(imuHeadingDeg));
        const double newHeading = imuContinuous_ + imuOffset_;
        const double dTheta = deg2rad(newHeading - pose_.theta);
        const double d = 0.5 * (dl + dr);

        // chord of the arc, not d, or fast turning drives drift outward
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

    /// Encoder travel in inches, which is what EZ's drive PID runs on.
    double leftInches(const EncoderReading& e) const { return e.leftDeg * params_.inchesPerWheelDegree(); }
    double rightInches(const EncoderReading& e) const { return e.rightDeg * params_.inchesPerWheelDegree(); }

private:
    RobotParams params_;
    Pose pose_{};
    EncoderReading prevEnc_{};
    double imuOffset_ = 0.0, imuContinuous_ = 0.0;
    double distanceTravelled_ = 0.0;
    bool initialised_ = false;
};

}  // namespace visbot
