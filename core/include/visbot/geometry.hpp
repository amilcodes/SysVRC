// visbot/geometry.hpp: units, angles, 2D pose.
//
// Same conventions as EZ-Template so nothing needs converting: inches,
// degrees, x right, y forward, heading clockwise from +y. The ROS nodes
// convert to REP-103 (metres, radians, CCW) at the edge and nowhere else.
#pragma once

#include <cmath>

namespace visbot {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInchPerMetre = 39.3700787402;

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }
constexpr double in2m(double in) { return in / kInchPerMetre; }
constexpr double m2in(double m) { return m * kInchPerMetre; }

/// Wrap degrees to (-180, 180]. `<= 0` so that 180 stays 180 instead of -180.
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
constexpr T clamp(T v, T lo, T hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

template <typename T>
constexpr int sgn(T v) { return (T(0) < v) - (v < T(0)); }

/// Compass heading -> ROS yaw (rad, CCW from +x).
inline double headingToYaw(double headingDeg) { return deg2rad(90.0 - headingDeg); }
/// ROS yaw (rad, CCW from +x) -> compass heading (deg, CW from +y).
inline double yawToHeading(double yawRad) { return wrapDeg(90.0 - rad2deg(yawRad)); }

/// Fold a wrapped reading into a continuous heading. Gazebo gives us a
/// wrapped yaw, the V5 IMU a continuous one; this makes them look the same.
inline double unwrapInto(double continuousPrev, double measuredWrapped) {
    return continuousPrev + wrapDeg(measuredWrapped - continuousPrev);
}

struct Pose {
    double x = 0.0;      // inches, field-right
    double y = 0.0;      // inches, field-forward
    /// Compass heading in degrees, continuous (not wrapped), like
    /// drive_imu_get() on the brain. EZ turns rely on this: 170 -> -170 is a
    /// target of 190, and wrapping the measurement would make that a 370 deg
    /// error. Use wrappedTheta() for display.
    double theta = 0.0;

    double distanceTo(const Pose& o) const { return std::hypot(o.x - x, o.y - y); }

    /// Heading in (-180, 180], for display and bearings.
    double wrappedTheta() const { return wrapDeg(theta); }

    /// Compass heading from this pose to another point.
    double headingTo(double tx, double ty) const {
        return wrapDeg(rad2deg(std::atan2(tx - x, ty - y)));
    }
};

/// Move a pose from the routine's own frame (wherever odom_xyt_set put the
/// origin) onto the field, given where the routine really starts.
inline Pose codeToField(const Pose& p, const Pose& codeStart, const Pose& fieldStart) {
    const double phi = deg2rad(fieldStart.theta - codeStart.theta);  // clockwise
    const double dx = p.x - codeStart.x, dy = p.y - codeStart.y;
    return {fieldStart.x + dx * std::cos(phi) + dy * std::sin(phi),
            fieldStart.y - dx * std::sin(phi) + dy * std::cos(phi),
            p.theta + (fieldStart.theta - codeStart.theta)};
}

/// Left/right drive command in -127..127, same units as pros::Motor::move().
struct WheelCmd {
    double left = 0.0;
    double right = 0.0;

    WheelCmd clipped(double limit = 127.0) const {
        return {clamp(left, -limit, limit), clamp(right, -limit, limit)};
    }
};

}  // namespace visbot
