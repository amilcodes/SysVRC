// visbot/pid.hpp — a faithful port of ez::PID (EZ-Template v3.2.2).
//
// This is deliberately a *port*, not an improvement: the point of the testbed
// is that what you tune here is what the V5 brain does. Where EZ has a quirk,
// this has the same quirk, marked `EZ quirk:` with what the textbook version
// would be. Verified against EZ-Template v3.2.2 src/EZ-Template/PID.cpp.
//
// Rate independence: EZ ticks at util::DELAY_TIME = 10 ms and its kD is a raw
// per-tick delta (not divided by dt), while its integral is a raw per-tick
// sum. To run the same gains in a 120 Hz loop, the derivative is rescaled to
// a 10 ms tick and the integral accumulates per 10 ms-equivalent. Pass the
// real loop period as `dtSeconds` and the controller behaves as if it were
// the 100 Hz brain loop. See test_pid.cpp::derivative_is_rate_invariant.
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

/// Passed is not an EZ exit: it marks a pid_wait_until / pid_wait_quick that
/// released because the robot crossed its target, not because a PID settled.
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
    /// ez::util::DELAY_TIME — the rate EZ's gains were tuned at.
    static constexpr double kEzTickSec = 0.010;
    /// ez::PID::velocity_zero_main — |derivative| below this counts as stopped.
    static constexpr double kVelocityZero = 0.05;

    Pid() = default;
    explicit Pid(PidGains g, ExitConditions e = {}) : gains_(g), exit_(e) {}

    void setGains(PidGains g) { gains_ = g; }
    void setExit(ExitConditions e) { exit_ = e; }
    const PidGains& gains() const { return gains_; }
    const ExitConditions& exitConditions() const { return exit_; }

    /// ez::PID::target_set — note EZ does NOT clear the integral or the
    /// previous measurement here, so a re-target mid-motion keeps its state.
    /// That matters for motion chaining (pid_wait_quick_chain moves the
    /// target while the loop is running).
    void setTarget(double t) { target_ = t; }
    double target() const { return target_; }
    void addToTarget(double d) { target_ += d; }

    /// ez::PID::reset_variables — called when a *new* motion starts.
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

    /// ez::PID::compute_error(err, current) — used where the caller wraps the
    /// error itself (turns, odom angular). `current` is still required: the
    /// derivative is taken on the measurement, not the error.
    double computeError(double error, double current, double dtSeconds = kEzTickSec) {
        const double tickScale = dtSeconds / kEzTickSec;  // 1.0 at 100 Hz, 0.8333 at 120 Hz
        error_ = error;
        cur_ = current;
        if (first_) { prevCurrent_ = cur_; prevError_ = error_; first_ = false; }

        // Derivative on measurement, not error, to avoid derivative kick when
        // the target moves. Rescaled to a 10 ms tick for rate independence.
        derivative_ = (cur_ - prevCurrent_) / tickScale;

        if (gains_.kI != 0.0) {
            if (std::fabs(error_) < gains_.startI) integral_ += error_ * tickScale;

            // EZ quirk: the sign-flip reset compares the error's sign against
            // the sign of the previous *measurement*, not the previous error.
            // Textbook would be sgn(error) != sgn(prevError). Replicated
            // because this is what runs on the robot; see setTextbookIReset().
            const double against = textbookIReset_ ? prevError_ : prevCurrent_;
            if (sgn(error_) != sgn(against)) integral_ = 0.0;
        }

        output_ = (error_ * gains_.kP) + (integral_ * gains_.kI) - (derivative_ * gains_.kD);

        prevCurrent_ = cur_;
        prevError_ = error_;
        elapsedMs_ += dtSeconds * 1000.0;
        return output_;
    }

    /// Opt out of the EZ integral-reset quirk (off by default: match the robot).
    void setTextbookIReset(bool on) { textbookIReset_ = on; }

    double output() const { return output_; }
    double error() const { return error_; }
    double derivative() const { return derivative_; }
    double integral() const { return integral_; }
    double elapsedMs() const { return elapsedMs_; }

    /// ez::PID::exit_condition — call once per tick after compute().
    ///
    /// EZ quirk: small and big are `if / else if`. When small_error is set
    /// (which every motion in autons.cpp does), the big-error branch is dead
    /// code and BigError can never be returned. Replicated exactly; the
    /// textbook reading — "big is a looser fallback that also applies" — is
    /// what a naive port produces and it makes motions exit early.
    ///
    /// `timeoutMs` is not an EZ feature; EZ relies on velocity/mA exits. It
    /// is kept here as a backstop for headless runs and is reported
    /// separately so it can't be mistaken for robot behaviour.
    ExitReason exitCondition(double dtSeconds = kEzTickSec, double timeoutMs = 0.0) {
        if (!exit_.any()) return ExitReason::NoConstants;
        const double ms = dtSeconds * 1000.0;
        const double ae = std::fabs(error_);

        if (exit_.smallError != 0.0) {
            if (ae < exit_.smallError) {
                smallMs_ += ms;
                bigMs_ = 0.0;  // while small is running, big does not
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
