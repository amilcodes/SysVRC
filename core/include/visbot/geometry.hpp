// visbot/geometry.hpp — units, angles, and 2D pose.
//
// Conventions match EZ-Template / the V5 code so gains port 1:1:
//   * distances in inches, angles in degrees
//   * heading is a compass heading: 0 deg = field-forward (+y), clockwise positive
//   * x is field-right, y is field-forward
// The ROS 2 side converts to REP-103 (metres, radians, CCW) at the node boundary
// and nowhere else.
#pragma once

#include <cmath>

namespace visbot {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInchPerMetre = 39.3700787402;

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }
constexpr double in2m(double in) { return in / kInchPerMetre; }
constexpr double m2in(double m) { return m * kInchPerMetre; }

/// Wrap an angle in degrees to (-180, 180].
/// The `<= 0` is deliberate: it puts exactly 180 (and -180) at +180 rather
/// than -180, which is what the stated range says and what keeps a robot
/// facing the far wall from reading as -180.
inline double wrapDeg(double a) {
    a = std::fmod(a + 180.0, 360.0);
    if (a <= 0) a += 360.0;
    return a - 180.0;
}

/// Wrap an angle in radians to (-pi, pi]. See wrapDeg for the `<= 0`.
inline double wrapRad(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0) a += 2.0 * kPi;
    return a - kPi;
}

template <typename T>
constexpr T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

template <typename T>
constexpr int sgn(T v) { return (T(0) < v) - (v < T(0)); }

/// Compass heading -> ROS yaw (rad, CCW from +x).
inline double headingToYaw(double headingDeg) { return deg2rad(90.0 - headingDeg); }
/// ROS yaw (rad, CCW from +x) -> compass heading (deg, CW from +y).
inline double yawToHeading(double yawRad) { return wrapDeg(90.0 - rad2deg(yawRad)); }

/// Fold a freshly-measured wrapped angle into a continuous series, given the
/// previous continuous value. Lets a wrapped source (a Gazebo quaternion) and
/// a continuous one (the V5 inertial) both drive continuous heading state.
inline double unwrapInto(double continuousPrev, double measuredWrapped) {
    return continuousPrev + wrapDeg(measuredWrapped - continuousPrev);
}

struct Pose {
    double x = 0.0;      // inches, field-right
    double y = 0.0;      // inches, field-forward
    /// Compass heading in degrees, CONTINUOUS — it is not wrapped to ±180.
    /// EZ-Template's drive_imu_get() behaves the same way (the V5 inertial
    /// reports cumulative rotation), and every turn in EZ depends on it: a
    /// turn from 170 to -170 is issued as a target of 190, and wrapping the
    /// measurement would turn that 20 degree error into 370. Use
    /// wrappedTheta() for display or when comparing against a bearing.
    double theta = 0.0;

    double distanceTo(const Pose& o) const { return std::hypot(o.x - x, o.y - y); }

    /// Heading folded into (-180, 180] — for display and bearing comparisons.
    double wrappedTheta() const { return wrapDeg(theta); }

    /// Compass heading from this pose to another point.
    double headingTo(double tx, double ty) const {
        return wrapDeg(rad2deg(std::atan2(tx - x, ty - y)));
    }
};

/// Left/right drive command in V5 "voltage units", -127..127, exactly what
/// pros::Motor::move() takes on the brain.
struct WheelCmd {
    double left = 0.0;
    double right = 0.0;

    WheelCmd clipped(double limit = 127.0) const {
        return {clamp(left, -limit, limit), clamp(right, -limit, limit)};
    }
};

}  // namespace visbot
