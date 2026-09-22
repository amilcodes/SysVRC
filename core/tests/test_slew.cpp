// Pins ez::slew's exact ramp: a line in error space from min_speed at the
// start of the motion to max_speed after `distance_to_travel`, then latched
// off for the rest of the motion.
#include "check.hpp"
#include "visbot/slew.hpp"

using namespace visbot;

TEST(ramps_from_min_to_max_over_distance) {
    Slew s(3.0, 70.0);
    s.initialize(true, 127.0, /*target=*/24.0, /*current=*/0.0);
    EXPECT_NEAR(s.iterate(0.0), 70.0, 1e-9);      // starts at min_speed
    EXPECT_NEAR(s.iterate(1.5), 98.5, 1e-9);      // linear halfway
    EXPECT_NEAR(s.iterate(2.9), 125.1, 1e-9);
    EXPECT_NEAR(s.iterate(3.0), 127.0, 1e-9);     // intercept reached
}

TEST(latches_off_after_the_ramp) {
    Slew s(3.0, 70.0);
    s.initialize(true, 127.0, 24.0, 0.0);
    s.iterate(3.5);
    EXPECT_TRUE(!s.enabled());
    EXPECT_NEAR(s.iterate(0.0), 127.0, 1e-9);  // does not re-engage on backslide
}

TEST(works_in_reverse) {
    Slew s(3.0, 70.0);
    s.initialize(true, 127.0, /*target=*/-24.0, /*current=*/0.0);
    EXPECT_NEAR(s.iterate(0.0), 70.0, 1e-9);
    EXPECT_NEAR(s.iterate(-1.5), 98.5, 1e-9);
    s.iterate(-3.5);
    EXPECT_TRUE(!s.enabled());
}

TEST(ez_quirk_disabled_when_max_speed_below_min) {
    // A motion slower than min_speed is not ramped at all — EZ would
    // otherwise start it *faster* than its requested cap.
    Slew s(3.0, 70.0);
    s.initialize(true, 50.0, 24.0, 0.0);
    EXPECT_TRUE(!s.enabled());
    EXPECT_NEAR(s.iterate(0.0), 50.0, 1e-9);
}

TEST(disabled_returns_max_speed) {
    Slew s(3.0, 70.0);
    s.initialize(false, 127.0, 24.0, 0.0);
    EXPECT_TRUE(!s.enabled());
    EXPECT_NEAR(s.iterate(0.0), 127.0, 1e-9);
}

TEST(respects_a_start_offset) {
    // Mid-auton the encoder is not at zero; the ramp is relative to wherever
    // the motion started.
    Slew s(3.0, 70.0);
    s.initialize(true, 127.0, /*target=*/124.0, /*current=*/100.0);
    EXPECT_NEAR(s.iterate(100.0), 70.0, 1e-9);
    EXPECT_NEAR(s.iterate(101.5), 98.5, 1e-9);
}

TEST_MAIN()
