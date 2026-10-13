// visbot/pid.hpp: port of ez::PID, EZ-Template v3.2.2 (src/EZ-Template/PID.cpp).
//
// Kept bug-for-bug with EZ so gains tuned here mean the same thing on the
// brain. Anything that isn't textbook PID is marked "EZ quirk".
//
// EZ runs every 10 ms and its D and I terms are per tick, not per second. The
// sim loop is 120 Hz, so both get scaled back to a 10 ms tick from the real
// dt. test_pid.cpp checks the output doesn't depend on the loop rate.
#pragma once

#include <cmath>
#include <cstdint>

#include "visbot/geometry.hpp"

namespace visbot {

struct PidGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double startI = 0.0;  // integral only accumulates when |error| < startI
};

/// ez::PID::exit_condition_. Times in ms, errors in the PID's own units.
struct ExitConditions {
    int    smallTimeMs    = 0;
    double smallError     = 0.0;
    int    bigTimeMs      = 0;
    double bigError       = 0.0;
    int    velocityTimeMs = 0;

    bool any() const {
        return smallError != 0 || smallTimeMs != 0 || bigError != 0 || bigTimeMs != 0 || velocityTimeMs != 0;
    }
};

/// Passed isn't an EZ exit. It's what a pid_wait_until / pid_wait_quick
/// reports when the robot crossed its target instead of settling on it.
enum class ExitReason : uint8_t { Running = 0, SmallError, BigError, Velocity, Timeout, NoConstants, Passed };

inline const char* toString(ExitReason r) {
    switch (r) {
        case ExitReason::Running:      return "running";
        case ExitReason::SmallError:   return "small_error";
        case ExitReason::BigError:     return "big_error";
        case ExitReason::Velocity:     return "velocity";
        case ExitReason::Timeout:      return "timeout";
        case ExitReason::NoConstants:  return "no_constants";
        case ExitReason::Passed:       return "passed";
    }
    return "?";
}

class Pid {
public:
    /// ez::util::DELAY_TIME, the tick EZ's gains assume.
    static constexpr double kEzTickSec = 0.010;
    /// ez::PID::velocity_zero_main: |derivative| below this counts as stopped.
    static constexpr double kVelocityZero = 0.05;

    Pid() = default;
    explicit Pid(PidGains g, ExitConditions e = {}) : gains_(g), exit_(e) {}

    void setGains(PidGains g) { gains_ = g; }
    void setExit(ExitConditions e) { exit_ = e; }
    const PidGains& gains() const { return gains_; }
    const ExitConditions& exitConditions() const { return exit_; }

    /// ez::PID::target_set. Doesn't touch the integral or the last
    /// measurement, which is what lets pid_wait_quick_chain move the target
    /// mid-motion without a bump.
    void setTarget(double t) { target_ = t; }
    double target() const { return target_; }
    void addToTarget(double d) { target_ += d; }

    /// ez::PID::reset_variables, called when a new motion starts.
    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        prevCurrent_ = 0.0;
        derivative_ = 0.0;
        error_ = 0.0;
        output_ = 0.0;
        first_ = true;
        resetTimers();
    }

    /// ez::PID::timers_reset
    void resetTimers() { smallMs_ = bigMs_ = velMs_ = 0.0; elapsedMs_ = 0.0; }

    /// ez::PID::compute(current)
    double compute(double current, double dtSeconds = kEzTickSec) {
        return computeError(target_ - current, current, dtSeconds);
    }

    /// ez::PID::compute_error. For callers that wrap the error themselves
    /// (turns, odom angular). Still needs `current` because D is taken on the
    /// measurement.
    double computeError(double error, double current, double dtSeconds = kEzTickSec) {
        const double tickScale = dtSeconds / kEzTickSec;  // 1.0 at 100 Hz, 0.8333 at 120 Hz
        error_ = error;
        cur_ = current;
        if (first_) { prevCurrent_ = cur_; prevError_ = error_; first_ = false; }

        // D on measurement so moving the target doesn't kick the output.
        derivative_ = (cur_ - prevCurrent_) / tickScale;

        if (gains_.kI != 0.0) {
            if (std::fabs(error_) < gains_.startI) integral_ += error_ * tickScale;

            // EZ quirk: the sign-flip reset compares against the previous
            // *measurement*, not the previous error. Probably a bug in EZ, but
            // it's what the brain runs. setTextbookIReset() if you want the fix.
            const double against = textbookIReset_ ? prevError_ : prevCurrent_;
            if (sgn(error_) != sgn(against)) integral_ = 0.0;
        }

        output_ = (error_ * gains_.kP) + (integral_ * gains_.kI) - (derivative_ * gains_.kD);

        prevCurrent_ = cur_;
        prevError_ = error_;
        elapsedMs_ += dtSeconds * 1000.0;
        return output_;
    }

    /// Use sgn(prevError) for the integral reset instead of EZ's version.
    void setTextbookIReset(bool on) { textbookIReset_ = on; }

    double output() const { return output_; }
    double error() const { return error_; }
    double derivative() const { return derivative_; }
    double integral() const { return integral_; }
    double elapsedMs() const { return elapsedMs_; }

    /// ez::PID::exit_condition. Call once per tick after compute().
    ///
    /// EZ quirk: small and big are if / else if, so once small_error is set
    /// (every motion in autons.cpp sets it) BigError can never fire. Porting
    /// it as "big is a looser fallback" makes motions exit early.
    ///
    /// timeoutMs isn't in EZ. It's a backstop so a stuck headless run ends,
    /// and it reports as Timeout so it can't pass for robot behaviour.
    ExitReason exitCondition(double dtSeconds = kEzTickSec, double timeoutMs = 0.0) {
        if (!exit_.any()) return ExitReason::NoConstants;
        const double ms = dtSeconds * 1000.0;
        const double ae = std::fabs(error_);

        if (exit_.smallError != 0.0) {
            if (ae < exit_.smallError) {
                smallMs_ += ms;
                bigMs_ = 0.0;  // big doesn't run while small is counting
                if (smallMs_ > exit_.smallTimeMs) { resetTimers(); return ExitReason::SmallError; }
            } else {
                smallMs_ = 0.0;
            }
        } else if (exit_.bigError != 0.0 && exit_.bigTimeMs != 0) {
            if (ae < exit_.bigError) {
                bigMs_ += ms;
                if (bigMs_ > exit_.bigTimeMs) { resetTimers(); return ExitReason::BigError; }
            } else {
                bigMs_ = 0.0;
            }
        }

        if (exit_.velocityTimeMs != 0) {
            if (std::fabs(derivative_) <= kVelocityZero) {
                velMs_ += ms;
                if (velMs_ > exit_.velocityTimeMs) { resetTimers(); return ExitReason::Velocity; }
            } else {
                velMs_ = 0.0;
            }
        }

        if (timeoutMs > 0.0 && elapsedMs_ > timeoutMs) { resetTimers(); return ExitReason::Timeout; }
        return ExitReason::Running;
    }

private:
    PidGains gains_{};
    ExitConditions exit_{};
    double target_ = 0.0;
    double error_ = 0.0, prevError_ = 0.0, cur_ = 0.0, prevCurrent_ = 0.0;
    double derivative_ = 0.0, integral_ = 0.0, output_ = 0.0;
    bool first_ = true, textbookIReset_ = false;
    double smallMs_ = 0.0, bigMs_ = 0.0, velMs_ = 0.0, elapsedMs_ = 0.0;
};

}  // namespace visbot
