// visbot/pid.hpp — PID with EZ-Template semantics (start_i, sign-flip reset,
// small/big/velocity exit conditions).
//
// EZ runs its loops at 10 ms. The gains in constants.hpp were tuned at that
// rate, and kD in particular is "per tick". To run the *same* gains at 120 Hz
// in sim (or any other rate) the derivative is rescaled to a 10 ms tick, and
// the integral is accumulated per 10 ms-equivalent. That is what `dtSeconds`
// is for: pass the real loop period and the controller behaves as if it were
// the 100 Hz brain loop.
#pragma once

#include <cmath>
#include <cstdint>

#include "visbot/geometry.hpp"

namespace visbot {

struct PidGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double startI = 0.0;  // integral only accumulates when |error| < startI (0 = always)
};

/// Exit conditions à la ez::PID::exit_condition().
struct ExitConditions {
    int    smallTimeMs   = 0;    // time |error| must stay below smallError
    double smallError    = 0.0;
    int    bigTimeMs     = 0;    // time |error| must stay below bigError
    double bigError      = 0.0;
    int    velocityTimeMs = 0;   // time with ~zero velocity while still outside bigError
};

enum class ExitReason : uint8_t { Running = 0, SmallError, BigError, Velocity, Timeout };

inline const char* toString(ExitReason r) {
    switch (r) {
        case ExitReason::Running:    return "running";
        case ExitReason::SmallError: return "small_error";
        case ExitReason::BigError:   return "big_error";
        case ExitReason::Velocity:   return "velocity";
        case ExitReason::Timeout:    return "timeout";
    }
    return "?";
}

class Pid {
public:
    static constexpr double kEzTickSec = 0.010;

    Pid() = default;
    explicit Pid(PidGains g, ExitConditions e = {}) : gains_(g), exit_(e) {}

    void setGains(PidGains g) { gains_ = g; }
    void setExit(ExitConditions e) { exit_ = e; }
    const PidGains& gains() const { return gains_; }

    void setTarget(double t) {
        target_ = t;
        reset();
    }
    double target() const { return target_; }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        derivative_ = 0.0;
        error_ = 0.0;
        first_ = true;
        smallMs_ = bigMs_ = velMs_ = elapsedMs_ = 0.0;
    }

    /// One controller step. `current` is the measured value in the same units
    /// as the target; `dtSeconds` is the real loop period.
    double compute(double current, double dtSeconds = kEzTickSec) {
        return computeError(target_ - current, dtSeconds);
    }

    /// Same as compute() but with the error supplied directly (used for
    /// wrapped-angle targets where error is computed by the caller).
    double computeError(double error, double dtSeconds = kEzTickSec) {
        const double tickScale = dtSeconds / kEzTickSec;  // 1.0 at 100 Hz, 0.833 at 120 Hz
        error_ = error;
        if (first_) { prevError_ = error_; first_ = false; }

        derivative_ = (error_ - prevError_) / tickScale;  // per-EZ-tick derivative

        if (gains_.kI != 0.0) {
            if (gains_.startI == 0.0 || std::fabs(error_) < gains_.startI)
                integral_ += error_ * tickScale;
            if (sgn(error_) != sgn(prevError_)) integral_ = 0.0;
        }

        const double out = error_ * gains_.kP + integral_ * gains_.kI + derivative_ * gains_.kD;
        prevError_ = error_;
        elapsedMs_ += dtSeconds * 1000.0;
        return out;
    }

    double error() const { return error_; }
    double derivative() const { return derivative_; }
    double elapsedMs() const { return elapsedMs_; }

    /// Evaluate exit conditions after each compute(). Mirrors EZ: timers
    /// accumulate while the condition holds and reset when it breaks.
    ExitReason exitCondition(double dtSeconds = kEzTickSec, double timeoutMs = 0.0) {
        const double ms = dtSeconds * 1000.0;
        const double ae = std::fabs(error_);

        if (exit_.smallError > 0.0) {
            smallMs_ = (ae < exit_.smallError) ? smallMs_ + ms : 0.0;
            if (smallMs_ > exit_.smallTimeMs) return ExitReason::SmallError;
        }
        if (exit_.bigError > 0.0) {
            bigMs_ = (ae < exit_.bigError) ? bigMs_ + ms : 0.0;
            if (bigMs_ > exit_.bigTimeMs) return ExitReason::BigError;
        }
        if (exit_.velocityTimeMs > 0) {
            // stalled: no motion while still far from target
            const bool stalled = std::fabs(derivative_) <= 0.05 && ae > exit_.bigError;
            velMs_ = stalled ? velMs_ + ms : 0.0;
            if (velMs_ > exit_.velocityTimeMs) return ExitReason::Velocity;
        }
        if (timeoutMs > 0.0 && elapsedMs_ > timeoutMs) return ExitReason::Timeout;
        return ExitReason::Running;
    }

private:
    PidGains gains_{};
    ExitConditions exit_{};
    double target_ = 0.0;
    double error_ = 0.0, prevError_ = 0.0, derivative_ = 0.0, integral_ = 0.0;
    bool first_ = true;
    double smallMs_ = 0.0, bigMs_ = 0.0, velMs_ = 0.0, elapsedMs_ = 0.0;
};

}  // namespace visbot
