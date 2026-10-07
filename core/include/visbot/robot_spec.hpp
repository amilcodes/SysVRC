// visbot/robot_spec.hpp — a robot.json, as far as the drivetrain sim cares.
//
// robot.json describes one physical robot: what the CAD says it is (drive
// geometry, gearing, footprint, mechanisms) and what the code tells EZ-Template
// it is (the ez::Drive constructor). tools/robot_studio writes it from a CAD
// export plus the team's code; docs/robot_spec.md has the whole format.
//
// Only the drivetrain is read here. Mechanisms and the bindings from action()
// lines to them are passed through untouched to the report, which runs the
// game-element sim.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "visbot/constants.hpp"
#include "visbot/json.hpp"

namespace visbot {

struct RobotSpec {
    std::string name;
    /// The robot as built: drives the plant.
    RobotParams physical;
    /// What the code tells EZ (wheel size and rpm in the ez::Drive
    /// constructor): drives the controller's odometry. When the two disagree,
    /// odometry is off on the real robot too.
    RobotParams declared;
    /// Drive response time constant, from the file or estimated from mass,
    /// motor count and top speed.
    double motorTauSec = 0.12;
    /// Encoder counts as the controller converts them, relative to the truth
    /// (declared ratio over physical ratio). 1 when code and CAD agree.
    double encoderScale = 1.0;
    /// The file itself, re-serialised compactly, for the report.
    std::string json;
};

struct RobotSpecResult {
    RobotSpec spec;
    std::vector<std::string> errors;
    bool ok() const { return errors.empty(); }
};

namespace robot_spec_detail {

inline void writeString(std::ostream& o, const std::string& str) {
    o << '"';
    for (char c : str) {
        switch (c) {
            case '"': o << "\\\""; break;
            case '\\': o << "\\\\"; break;
            case '\n': o << "\\n"; break;
            case '\r': o << "\\r"; break;
            case '\t': o << "\\t"; break;
            case '<': o << "\\u003c"; break;   // safe inside the report's <script> block
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char b[8];
                    std::snprintf(b, sizeof b, "\\u%04x", static_cast<unsigned>(c));
                    o << b;
                } else {
                    o << c;
                }
        }
    }
    o << '"';
}

inline void writeCompact(std::ostream& o, const Json& v) {
    switch (v.type()) {
        case Json::Type::Null: o << "null"; break;
        case Json::Type::Bool: o << (v.boolean() ? "true" : "false"); break;
        case Json::Type::Number: {
            const double d = v.number();
            if (!std::isfinite(d)) { o << "null"; break; }
            if (d == std::floor(d) && std::fabs(d) < 1e15) { o << static_cast<long long>(d); break; }
            std::ostringstream n;
            n.precision(10);
            n << d;
            o << n.str();
            break;
        }
        case Json::Type::String: writeString(o, v.string()); break;
        case Json::Type::Array: {
            o << '[';
            for (size_t i = 0; i < v.items().size(); ++i) {
                if (i) o << ',';
                writeCompact(o, v.items()[i]);
            }
            o << ']';
            break;
        }
        case Json::Type::Object: {
            o << '{';
            for (size_t i = 0; i < v.members().size(); ++i) {
                if (i) o << ',';
                writeString(o, v.members()[i].first);
                o << ':';
                writeCompact(o, v.members()[i].second);
            }
            o << '}';
            break;
        }
    }
}

}  // namespace robot_spec_detail

/// Parse robot.json text. Missing optional fields fall back to the defaults in
/// RobotParams (which match the team's own robot).
inline RobotSpecResult parseRobotSpec(const std::string& text) {
    RobotSpecResult r;
    std::string err;
    const Json j = Json::parse(text, &err);
    if (!err.empty()) { r.errors.push_back("not valid JSON: " + err); return r; }
    if (!j.isObject()) { r.errors.push_back("expected a JSON object"); return r; }

    RobotSpec& s = r.spec;
    s.name = j["name"].isString() ? j["name"].string() : "robot";
    const Json& d = j["drive"];
    if (!d.isObject()) { r.errors.push_back("missing \"drive\""); return r; }

    auto positive = [&](const Json& v, const char* what, double fallback) {
        if (v.isNull()) return fallback;
        if (!v.isNumber() || !(v.number() > 0)) { r.errors.push_back(std::string(what) + " must be a positive number"); return fallback; }
        return v.number();
    };

    RobotParams& p = s.physical;
    p.wheelDiameterIn = positive(d["wheel_diameter"], "drive.wheel_diameter", p.wheelDiameterIn);
    const double cartridge = positive(d["cartridge_rpm"], "drive.cartridge_rpm", 600.0);
    if (d["wheel_rpm"].isNumber()) p.wheelRpm = positive(d["wheel_rpm"], "drive.wheel_rpm", p.wheelRpm);
    else if (d["ratio"].isNumber()) p.wheelRpm = cartridge * positive(d["ratio"], "drive.ratio", 0.75);
    p.trackWidthIn = positive(d["track_width"], "drive.track_width", p.trackWidthIn);
    const Json& fp = j["footprint"];
    p.robotWidthIn = positive(fp["width"], "footprint.width", p.robotWidthIn);
    p.robotLengthIn = positive(fp["length"], "footprint.length", p.robotLengthIn);
    if (j["mass_lb"].isNumber()) p.massKg = positive(j["mass_lb"], "mass_lb", 16.5) * 0.45359237;
    if (p.trackWidthIn > p.robotWidthIn + 1.0)
        r.errors.push_back("drive.track_width is wider than footprint.width");

    // What the code declares. Absent means the code matches the robot.
    s.declared = p;
    const Json& c = j["code"];
    double declaredCartridge = cartridge;
    if (c.isObject()) {
        s.declared.wheelDiameterIn = positive(c["wheel_diameter"], "code.wheel_diameter", p.wheelDiameterIn);
        s.declared.wheelRpm = positive(c["wheel_rpm"], "code.wheel_rpm", p.wheelRpm);
        declaredCartridge = positive(c["cartridge_rpm"], "code.cartridge_rpm", cartridge);
    }
    // EZ turns motor-encoder degrees into wheel degrees with the ratio it was
    // given; the motor actually turned by the real ratio.
    s.encoderScale = (s.declared.wheelRpm / declaredCartridge) / (p.wheelRpm / cartridge);

    // Drive response. A DC-motor drivetrain's time constant goes roughly as
    // mass * top_speed^2 / power; anchored at 0.12 s for a 15 lb, six 11 W
    // motor, 3.25 in @ 450 rpm drive, which is what the default plant uses.
    if (d["tau_s"].isNumber()) {
        s.motorTauSec = positive(d["tau_s"], "drive.tau_s", 0.12);
    } else {
        const int motors = 2 * static_cast<int>(std::lround(d["motors_per_side"].number(3)));
        const double watts = (d["motor"].isString() && d["motor"].string() == "5.5W") ? 5.5 : 11.0;
        const double massLb = p.massKg / 0.45359237;
        const double v = p.maxWheelSpeedInPerSec();
        const double vRef = 450.0 / 60.0 * 3.25 * kPi;
        const double tau = 0.12 * (massLb / 15.0) * (v / vRef) * (v / vRef) * (6 * 11.0) / (std::max(1, motors) * watts);
        s.motorTauSec = std::clamp(tau, 0.05, 0.5);
    }

    std::ostringstream o;
    robot_spec_detail::writeCompact(o, j);
    s.json = o.str();
    return r;
}

inline RobotSpecResult loadRobotSpec(const std::string& path) {
    std::ifstream f(path);
    if (!f) { RobotSpecResult r; r.errors.push_back("can't open " + path); return r; }
    std::stringstream ss;
    ss << f.rdbuf();
    return parseRobotSpec(ss.str());
}

}  // namespace visbot
