// Each test here pins one semantic of ez::PID. Several pin behaviour that is
// arguably wrong but is what runs on the robot; those say so.
#include "check.hpp"
#include "visbot/pid.hpp"

using namespace visbot;

TEST(proportional_first_tick) {
    Pid p({2.0, 0.0, 0.0, 0.0});
    p.setTarget(10.0);
    EXPECT_NEAR(p.compute(0.0), 20.0, 1e-9);
}

TEST(derivative_is_on_measurement_not_error) {
    // Moving the target must NOT produce a derivative spike. This is why EZ
    // differentiates the measurement; differentiating the error would kick.
    Pid p({0.0, 0.0, 10.0, 0.0});
    p.setTarget(0.0);
    p.compute(5.0);
    p.compute(5.0);                       // settled, measurement not moving
    p.setTarget(100.0);                   // huge target jump
    const double out = p.compute(5.0);    // measurement still 5
    EXPECT_NEAR(out, 0.0, 1e-9);          // no derivative contribution at all
}

TEST(derivative_opposes_motion) {
    // Measurement climbing toward the target => derivative term subtracts.
    Pid p({0.0, 0.0, 1.0, 0.0});
    p.setTarget(100.0);
    p.compute(0.0);
    EXPECT_NEAR(p.compute(2.0), -2.0, 1e-9);   // -(cur - prev) * kD
}

TEST(derivative_is_rate_invariant) {
    // The same physical ramp sampled at 100 Hz and 120 Hz must give the same
    // kD contribution — that is what lets EZ's gains run in a 120 Hz loop.
    Pid a({0.0, 0.0, 1.0, 0.0}), b({0.0, 0.0, 1.0, 0.0});
    a.setTarget(0);
    b.setTarget(0);
    double outA = 0, outB = 0;
    const double slope = 50.0;  // measurement units per second
    for (int i = 0; i <= 12; ++i) outA = a.compute(slope * i * 0.010, 0.010);
    for (int i = 0; i <= 12; ++i) outB = b.compute(slope * i * (1.0 / 120.0), 1.0 / 120.0);
    EXPECT_NEAR(outA, outB, 1e-9);
    EXPECT_NEAR(outA, -0.5, 1e-9);  // kD * (measurement change per 10 ms)
}

TEST(integral_is_rate_invariant) {
    Pid a({0.0, 1.0, 0.0, 100.0}), b({0.0, 1.0, 0.0, 100.0});
    a.setTarget(10);
    b.setTarget(10);
    double outA = 0, outB = 0;
    for (int i = 0; i < 100; ++i) outA = a.compute(0.0, 0.010);          // 1.0 s
    for (int i = 0; i < 120; ++i) outB = b.compute(0.0, 1.0 / 120.0);    // 1.0 s
    EXPECT_NEAR(outA, outB, 1e-9);
}

TEST(start_i_gates_integral) {
    Pid p({0.0, 1.0, 0.0, 5.0});
    p.setTarget(0);
    p.compute(20.0);  // |error| = 20 > startI: nothing accumulates
    EXPECT_NEAR(p.compute(20.0), 0.0, 1e-9);
}

TEST(ez_quirk_integral_resets_against_previous_measurement) {
    // EZ compares sgn(error) to sgn(prev_current) — the previous
    // *measurement* — not sgn(prev_error). Whenever the measurement sits on
    // the opposite side of zero from the error, that test is true on every
    // tick and the integral is wiped before it can ever do anything.
    //
    // Here: target 0, measurement -3, so error is +3 forever. The integral
    // never accumulates, so kI is effectively dead for this motion.
    Pid p({0.0, 1.0, 0.0, 100.0});
    p.setTarget(0);
    for (int i = 0; i < 20; ++i) p.compute(-3.0);
    EXPECT_NEAR(p.integral(), 0.0, 1e-9);

    // The textbook rule (reset when the *error* changes sign) accumulates
    // normally here, which is why this is opt-in rather than the default:
    // turning it on changes how the robot behaves.
    Pid q({0.0, 1.0, 0.0, 100.0});
    q.setTextbookIReset(true);
    q.setTarget(0);
    for (int i = 0; i < 20; ++i) q.compute(-3.0);
    EXPECT_NEAR(q.integral(), 60.0, 1e-9);

    // Both agree on the common case: approaching a positive target from
    // below, then overshooting, dumps the integral.
    Pid z({0.0, 1.0, 0.0, 100.0});
    z.setTarget(10);
    z.compute(2.0);
    z.compute(4.0);
    EXPECT_TRUE(z.integral() > 0.0);
    // EZ accumulates first and resets second, so the overshoot tick's own
    // contribution is wiped along with the rest.
    z.compute(12.0);  // overshoot: error goes negative, measurement positive
    EXPECT_NEAR(z.integral(), 0.0, 1e-9);
}

TEST(exit_small_error_needs_time) {
    Pid p({1, 0, 0, 0}, {100, 1.0, 250, 3.0, 0});
    p.setTarget(0);
    ExitReason r = ExitReason::Running;
    int ticks = 0;
    while (r == ExitReason::Running && ticks < 100) {
        p.compute(0.5);
        r = p.exitCondition();
        ++ticks;
    }
    EXPECT_TRUE(r == ExitReason::SmallError);
    EXPECT_TRUE(ticks == 11);  // strictly greater than 100 ms at 10 ms/tick
}

TEST(ez_quirk_big_error_is_dead_when_small_is_set) {
    // EZ's small/big branches are `if / else if`. Every motion in autons.cpp
    // sets small_error, so BigError can never be returned on the robot. A
    // naive port runs both and exits motions early.
    Pid p({1, 0, 0, 0}, {100, 1.0, 250, 3.0, 0});
    p.setTarget(0);
    ExitReason r = ExitReason::Running;
    for (int i = 0; i < 200 && r == ExitReason::Running; ++i) {
        p.compute(2.0);  // inside big (3.0), outside small (1.0)
        r = p.exitCondition();
    }
    EXPECT_TRUE(r == ExitReason::Running);  // never exits, exactly like EZ

    // With small disabled, the big branch does run.
    Pid q({1, 0, 0, 0}, {0, 0.0, 250, 3.0, 0});
    q.setTarget(0);
    r = ExitReason::Running;
    for (int i = 0; i < 200 && r == ExitReason::Running; ++i) {
        q.compute(2.0);
        r = q.exitCondition();
    }
    EXPECT_TRUE(r == ExitReason::BigError);
}

TEST(velocity_exit_fires_when_measurement_stops) {
    // ez::PID::velocity_zero_main == 0.05, tested on the derivative alone —
    // there is no "and still far from target" condition.
    Pid p({1, 0, 0, 0}, {100, 1.0, 250, 3.0, 200});
    p.setTarget(100);
    ExitReason r = ExitReason::Running;
    int ticks = 0;
    while (r == ExitReason::Running && ticks < 100) {
        p.compute(10.0);  // stuck: measurement never changes
        r = p.exitCondition();
        ++ticks;
    }
    EXPECT_TRUE(r == ExitReason::Velocity);
    EXPECT_TRUE(ticks == 21);  // > 200 ms
}

TEST(velocity_exit_does_not_fire_while_moving) {
    Pid p({1, 0, 0, 0}, {100, 1.0, 250, 3.0, 200});
    p.setTarget(1000);
    ExitReason r = ExitReason::Running;
    for (int i = 0; i < 100 && r == ExitReason::Running; ++i) {
        p.compute(i * 1.0);  // 1.0 per tick, far above the 0.05 threshold
        r = p.exitCondition();
    }
    EXPECT_TRUE(r == ExitReason::Running);
}

TEST(no_constants_is_reported) {
    Pid p({1, 0, 0, 0});
    p.setTarget(0);
    p.compute(5);
    EXPECT_TRUE(p.exitCondition() == ExitReason::NoConstants);
}

TEST(exit_timeout_backstop) {
    Pid p({1, 0, 0, 0}, {100, 1.0, 0, 0.0, 0});
    p.setTarget(0);
    ExitReason r = ExitReason::Running;
    for (int i = 0; i < 200 && r == ExitReason::Running; ++i) {
        p.compute(100);
        r = p.exitCondition(0.010, 500);
    }
    EXPECT_TRUE(r == ExitReason::Timeout);
}

TEST(retarget_keeps_state_for_chaining) {
    // pid_wait_quick_chain moves the target mid-motion; EZ does not reset
    // the integral or previous measurement when it does.
    Pid p({1.0, 0.0, 1.0, 0.0});
    p.setTarget(10);
    p.compute(0);
    p.compute(2);
    p.addToTarget(5);
    EXPECT_NEAR(p.target(), 15.0, 1e-9);
    const double out = p.compute(4);
    EXPECT_NEAR(out, (15.0 - 4.0) * 1.0 - 2.0, 1e-9);  // continuous derivative
}

TEST_MAIN()
