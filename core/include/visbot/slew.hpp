// visbot/slew.hpp — EZ-style slew: ramp the speed cap from a minimum up to
// the requested max over the first `distance` units of travel, so the drive
// doesn't slip its wheels on launch.
#pragma once

#include <cmath>

#include "visbot/geometry.hpp"

namespace visbot {

class Slew {
public:
    Slew() = default;
    Slew(double distance, double minSpeed) : distance_(distance), minSpeed_(minSpeed) {}

    void setConstants(double distance, double minSpeed) {
        distance_ = distance;
        minSpeed_ = minSpeed;
    }

    /// Arm the slew for a new motion.
    void initialize(bool enabled, double maxSpeed, double target, double current) {
        enabled_ = enabled;
        maxSpeed_ = std::fabs(maxSpeed);
        start_ = current;
        sign_ = sgn(target - current);
        if (sign_ == 0) sign_ = 1;
        output_ = enabled_ ? minSpeed_ : maxSpeed_;
    }

    /// Current speed cap given how far we've travelled.
    double iterate(double current) {
        if (!enabled_ || distance_ <= 0.0) { output_ = maxSpeed_; return output_; }
        const double travelled = std::fabs(current - start_);
        const double progress = clamp(travelled / distance_, 0.0, 1.0);
        output_ = minSpeed_ + progress * (maxSpeed_ - minSpeed_);
        if (output_ > maxSpeed_) output_ = maxSpeed_;
        return output_;
    }

    double output() const { return output_; }
    bool enabled() const { return enabled_; }

private:
    double distance_ = 0.0, minSpeed_ = 0.0, maxSpeed_ = 127.0;
    double start_ = 0.0, output_ = 127.0;
    int sign_ = 1;
    bool enabled_ = false;
};

}  // namespace visbot
