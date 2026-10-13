// visbot/slew.hpp: port of ez::slew, EZ-Template v3.2.2 (src/EZ-Template/slew.cpp).
//
// EZ's slew is a line in position, not time: the speed cap ramps from
// min_speed to max_speed over the first distance_to_travel of the motion,
// then switches off for good.
#pragma once

#include <cmath>

#include "visbot/geometry.hpp"

namespace visbot {

class Slew {
public:
    struct Constants {
        double distanceToTravel = 0.0;
        double minSpeed = 0.0;
    };

    Slew() = default;
    Slew(double distance, double minSpeed) : c_{distance, minSpeed} {}

    void setConstants(double distance, double minSpeed) { c_ = {distance, minSpeed}; }
    Constants constants() const { return c_; }

    /// ez::slew::initialize. A motion slower than min_speed isn't ramped at all.
    void initialize(bool enabled, double maximumSpeed, double target, double current) {
        enabled_ = maximumSpeed < c_.minSpeed ? false : enabled;
        maxSpeed_ = maximumSpeed;
        sign_ = sgn(target - current);
        xIntercept_ = current + (c_.distanceToTravel * sign_);
        yIntercept_ = maxSpeed_ * sign_;
        const double dx = xIntercept_ - current;
        slope_ = dx != 0.0 ? ((sign_ * c_.minSpeed) - yIntercept_) / dx : 0.0;
        lastOutput_ = enabled_ ? c_.minSpeed : maxSpeed_;
    }

    /// ez::slew::iterate. Returns the speed cap for this tick.
    double iterate(double current) {
        if (enabled_) {
            error_ = xIntercept_ - current;
            if (sgn(error_) != sign_) {
                enabled_ = false;          // past the ramp, done for this motion
                lastOutput_ = maxSpeed_;
            } else {
                lastOutput_ = ((slope_ * error_) + yIntercept_) * sign_;
            }
        } else {
            lastOutput_ = maxSpeed_;
        }
        return lastOutput_;
    }

    double output() const { return lastOutput_; }
    bool enabled() const { return enabled_; }
    void setMaxSpeed(double s) { maxSpeed_ = s; }
    double maxSpeed() const { return maxSpeed_; }

private:
    Constants c_{};
    double maxSpeed_ = 127.0, lastOutput_ = 127.0;
    double xIntercept_ = 0.0, yIntercept_ = 0.0, slope_ = 0.0, error_ = 0.0;
    int sign_ = 1;
    bool enabled_ = false;
};

}  // namespace visbot
