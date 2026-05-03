#include "check.hpp"
#include "visbot/cheesy_drive.hpp"
using namespace visbot;

TEST(idle_is_zero) {
    CheesyDrive cd;
    auto [l, r] = cd.update(0, 0);
    EXPECT_NEAR(l, 0, 1e-12); EXPECT_NEAR(r, 0, 1e-12);
}

TEST(straight_throttle_is_symmetric_and_slewed) {
    CheesyDrive cd;
    auto [l, r] = cd.update(1.0, 0.0);
    EXPECT_NEAR(l, r, 1e-12);
    EXPECT_NEAR(l, 0.02, 1e-12);  // DRIVE_SLEW per tick from rest
    for (int i = 0; i < 100; ++i) std::tie(l, r) = cd.update(1.0, 0.0);
    EXPECT_NEAR(l, 1.0, 1e-9);
}

TEST(reverse_slew_is_twice_as_fast) {
    CheesyDrive cd;
    for (int i = 0; i < 100; ++i) cd.update(1.0, 0.0);
    auto [l, r] = cd.update(0.0, 0.0);
    EXPECT_NEAR(l, 1.0 - 0.04, 1e-9);
}

TEST(turn_in_place_is_antisymmetric) {
    CheesyDrive cd;
    auto [l, r] = cd.update(0.0, 0.6);
    EXPECT_NEAR(l, -r, 1e-12);
    EXPECT_TRUE(l > 0);           // positive turn = right turn = left wheel forward
    EXPECT_TRUE(l < 1.0);
    auto [ls, rs] = cd.update(0.0, 0.1);
    EXPECT_TRUE(ls < 0.1 * 0.6);  // squared remap: fine control near centre
}

TEST(neg_inertia_adds_turn_on_stick_change) {
    CheesyDrive cd;
    for (int i = 0; i < 60; ++i) cd.update(1.0, 0.0);
    auto [l1, r1] = cd.update(1.0, 0.3);   // stick moved: neg-inertia boost
    auto [l2, r2] = cd.update(1.0, 0.3);   // stick steady
    EXPECT_TRUE((l1 - r1) > (l2 - r2));
}

TEST_MAIN()
