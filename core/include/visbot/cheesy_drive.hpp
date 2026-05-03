// visbot/cheesy_drive.hpp — the operator-control mixer from
// v5/src/subsystemFiles/drive.cpp, ported verbatim into a stateful class.
//
// Inputs are normalised joystick values in [-1, 1]; outputs are wheel commands
// in [-1, 1] (multiply by 127 for the brain, or by max wheel speed in sim).
#pragma once

#include <cmath>
#include <utility>

#include "visbot/constants.hpp"
#include "visbot/geometry.hpp"

namespace visbot {

class CheesyDrive {
public:
    explicit CheesyDrive(CheesyParams p = {}) : p_(p) {}

    void reset() {
        prevTurn_ = prevThrottle_ = 0.0;
        quickStopAccumulator_ = negInertiaAccumulator_ = 0.0;
    }

    /// One tick. Returns {left, right} in [-1, 1].
    std::pair<double, double> update(double throttle, double turn) {
        bool turnInPlace = false;
        double linearCmd = throttle;

        if (std::fabs(throttle) < p_.deadband && std::fabs(turn) > p_.deadband) {
            linearCmd = 0.0;
            turnInPlace = true;
        } else if (throttle - prevThrottle_ > p_.slew) {
            linearCmd = prevThrottle_ + p_.slew;
        } else if (throttle - prevThrottle_ < -(p_.slew * 2)) {
            // double slew rate in reverse for faster stopping
            linearCmd = prevThrottle_ - (p_.slew * 2);
        }

        const double remappedTurn = turnRemapping(turn);

        double left, right;
        if (turnInPlace) {
            // squared for finer control at small speeds
            left = remappedTurn * std::fabs(remappedTurn);
            right = -remappedTurn * std::fabs(remappedTurn);
        } else {
            const double negInertiaPower = (turn - prevTurn_) * p_.negInertiaScalar;
            negInertiaAccumulator_ += negInertiaPower;

            const double angularCmd =
                std::fabs(linearCmd) * (remappedTurn + negInertiaAccumulator_) * p_.sensitivity
                - quickStopAccumulator_;

            right = left = linearCmd;
            left += angularCmd;
            right -= angularCmd;

            updateAccumulators();
        }

        prevTurn_ = turn;
        prevThrottle_ = linearCmd;
        return {clamp(left, -1.0, 1.0), clamp(right, -1.0, 1.0)};
    }

private:
    // sinusoidal remap applied twice for fine control near centre
    double turnRemapping(double iturn) const {
        const double denominator = std::sin(kPi / 2 * p_.turnNonlinearity);
        const double first = std::sin(kPi / 2 * p_.turnNonlinearity * iturn) / denominator;
        return std::sin(kPi / 2 * p_.turnNonlinearity * first) / denominator;
    }

    void updateAccumulators() {
        if (negInertiaAccumulator_ > 1)       negInertiaAccumulator_ -= 1;
        else if (negInertiaAccumulator_ < -1) negInertiaAccumulator_ += 1;
        else                                  negInertiaAccumulator_ = 0;

        if (quickStopAccumulator_ > 1)       quickStopAccumulator_ -= 1;
        else if (quickStopAccumulator_ < -1) quickStopAccumulator_ += 1;
        else                                 quickStopAccumulator_ = 0.0;
    }

    CheesyParams p_;
    double prevTurn_ = 0.0, prevThrottle_ = 0.0;
    double quickStopAccumulator_ = 0.0, negInertiaAccumulator_ = 0.0;
};

}  // namespace visbot
