#include "check.hpp"
#include "visbot/odometry.hpp"
#include "visbot/plant.hpp"
using namespace visbot;

TEST(wrap_deg_range_is_half_open_at_the_top) {
    // (-180, 180]: a robot facing the far wall reads +180, not -180.
    EXPECT_NEAR(wrapDeg(180.0), 180.0, 1e-12);
    EXPECT_NEAR(wrapDeg(-180.0), 180.0, 1e-12);
    EXPECT_NEAR(wrapDeg(540.0), 180.0, 1e-12);
    EXPECT_NEAR(wrapDeg(0.0), 0.0, 1e-12);
    EXPECT_NEAR(wrapDeg(360.0), 0.0, 1e-12);
    EXPECT_NEAR(wrapDeg(90.0), 90.0, 1e-12);
    EXPECT_NEAR(wrapDeg(-90.0), -90.0, 1e-12);
    EXPECT_NEAR(wrapDeg(181.0), -179.0, 1e-12);
    EXPECT_NEAR(wrapDeg(-181.0), 179.0, 1e-12);
    for (double a = -720.0; a <= 720.0; a += 3.0) {
        const double w = wrapDeg(a);
        EXPECT_TRUE(w > -180.0 && w <= 180.0);
    }
}

TEST(unwrap_into_follows_a_continuous_series) {
    // A wrapped source (Gazebo quaternion) must fold into continuous heading.
    double cont = 170.0;
    cont = unwrapInto(cont, -175.0);   // crossed 180 going up
    EXPECT_NEAR(cont, 185.0, 1e-9);
    cont = unwrapInto(cont, -150.0);
    EXPECT_NEAR(cont, 210.0, 1e-9);
    cont = unwrapInto(cont, 170.0);    // came back down
    EXPECT_NEAR(cont, 170.0, 1e-9);
}

TEST(straight_line) {
    RobotParams rp;
    Odometry od(rp);
    od.reset({0, 0, 0}, {0, 0}, 0);
    const double deg = 24.0 / rp.inchesPerWheelDegree();
    od.update({deg, deg}, 0);
    EXPECT_NEAR(od.pose().y, 24.0, 1e-9);
    EXPECT_NEAR(od.pose().x, 0.0, 1e-9);
}

TEST(heading_from_imu_with_offset) {
    Odometry od;
    od.reset({0, 0, 45}, {0, 0}, 10);  // IMU reads 10 when robot faces 45
    od.update({0, 0}, 40);
    EXPECT_NEAR(od.pose().theta, 75, 1e-9);
}

TEST(arc_matches_plant_truth) {
    // Drive the plant on a constant-radius arc and check odometry tracks it.
    RobotParams rp;
    PlantParams pp; pp.encoderStepDeg = 0; pp.imuNoiseStdDeg = 0;
    DiffDrivePlant plant(rp, pp);
    plant.reset({0, 0, 0});
    Odometry od(rp);
    auto s0 = plant.sensors();
    od.reset({0, 0, 0}, s0.enc, s0.imuHeadingDeg);
    for (int i = 0; i < 60; ++i) {
        plant.step({100, 60}, 1.0 / 120.0);
        auto s = plant.sensors();
        od.update(s.enc, s.imuHeadingDeg);
    }
    EXPECT_NEAR(od.pose().x, plant.truth().x, 0.05);
    EXPECT_NEAR(od.pose().y, plant.truth().y, 0.05);
    EXPECT_NEAR(od.pose().theta, plant.truth().theta, 1e-6);
    EXPECT_TRUE(plant.truth().theta > 20.0);  // it actually turned
}

TEST_MAIN()
