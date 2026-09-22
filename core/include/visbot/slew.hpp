// visbot/slew.hpp — a faithful port of ez::slew (EZ-Template v3.2.2).
//
// EZ's slew is a straight line in *error space*: at the start of a motion it
// builds y = mx + b through (x_intercept, sign*min_speed) and (current,
// max_speed), where x_intercept is `distance_to_travel` beyond the current
// sensor value. Each tick it evaluates that line at the remaining distance
// to the intercept, so the cap ramps from min_speed up to max_speed over the
// first `distance_to_travel` units of travel and then latches off.
//
// Verified against src/EZ-Template/slew.cpp.
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

    /// ez::slew::initialize. Note EZ disables slew outright when the
    /// requested max speed is below min_speed — a slow motion is not ramped.
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

    /// ez::slew::iterate — returns the current speed cap.
    double iterate(double current) {
        if (enabled_) {
            error_ = xIntercept_ - current;
            if (sgn(error_) != sign_) {
                enabled_ = false;          // travelled past the ramp: done
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
