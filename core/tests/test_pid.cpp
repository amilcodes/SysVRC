#include "check.hpp"
#include "visbot/pid.hpp"
using namespace visbot;

TEST(proportional_first_tick) {
    Pid p({2.0, 0.0, 0.0, 0.0});
    p.setTarget(10.0);
    EXPECT_NEAR(p.compute(0.0), 20.0, 1e-9);  // no derivative kick on first tick
}

TEST(derivative_is_rate_invariant) {
    // Same physical error ramp sampled at 100 Hz and 120 Hz must yield the
    // same kD contribution: that is what lets EZ gains run in a 120 Hz loop.
    Pid a({0.0, 0.0, 1.0, 0.0}), b({0.0, 0.0, 1.0, 0.0});
    a.setTarget(0); b.setTarget(0);
    double outA = 0, outB = 0;
    const double slope = -50.0;  // error units per second
    for (int i = 0; i <= 12; ++i) outA = a.compute(-slope * i * 0.010, 0.010);
    for (int i = 0; i <= 12; ++i) outB = b.compute(-slope * i * (1.0 / 120.0), 1.0 / 120.0);
    EXPECT_NEAR(outA, outB, 1e-9);
    EXPECT_NEAR(outA, -0.5, 1e-9);  // kD * (error change per 10 ms) = 1 * (-50 * 0.010)
}

TEST(start_i_gates_integral) {
    Pid p({0.0, 1.0, 0.0, 5.0});
    p.setTarget(0);
    p.compute(-20.0);          // |error| = 20 > startI: no integral
    EXPECT_NEAR(p.compute(-20.0), 0.0, 1e-9);
    p.compute(-2.0);           // now inside startI
    EXPECT_NEAR(p.compute(-2.0), 4.0, 1e-9);  // integral = 2 + 2
}

TEST(sign_flip_resets_integral) {
    Pid p({0.0, 1.0, 0.0, 0.0});
    p.setTarget(0);
    p.compute(-3.0); p.compute(-3.0);
    EXPECT_NEAR(p.compute(1.0), 0.0, 1e-9);  // crossed zero -> integral dumped
}

TEST(exit_small_error_needs_time) {
    Pid p({1, 0, 0, 0}, {100, 1.0, 250, 3.0, 0});
    p.setTarget(0);
    ExitReason r = ExitReason::Running;
    int ticks = 0;
    while (r == ExitReason::Running && ticks < 100) { p.compute(0.5); r = p.exitCondition(); ++ticks; }
    EXPECT_TRUE(r == ExitReason::SmallError);
    EXPECT_TRUE(ticks == 11);  // >100 ms at 10 ms/tick
}

TEST(exit_timeout) {
    Pid p({1, 0, 0, 0});
    p.setTarget(0);
    ExitReason r = ExitReason::Running;
    for (int i = 0; i < 60 && r == ExitReason::Running; ++i) { p.compute(100); r = p.exitCondition(0.010, 500); }
    EXPECT_TRUE(r == ExitReason::Timeout);
}

TEST_MAIN()
