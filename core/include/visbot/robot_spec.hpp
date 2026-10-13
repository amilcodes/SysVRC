// visbot/robot_spec.hpp: the drivetrain parts of robot.json (format in
// docs/robot_spec.md). Mechanisms and bindings aren't read here; they go to
// the report untouched, where web/game.js uses them.
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

constexpr double kKgPerLb = 0.45359237;

struct RobotSpec {
    std::string name;
    RobotParams physical;       // the robot as built -> plant
    RobotParams declared;       // what the code tells EZ -> controller odometry
    double motorTauSec = 0.12;  // from the file, or estimated
    double encoderScale = 1.0;  // != 1 when code and robot disagree on the gearing
    std::string json;           // the whole file, compacted, for the report
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

/// Anything optional that's missing falls back to RobotParams (our robot).
inline RobotSpecResult parseRobotSpec(const std::string& text) {
    RobotSpecResult r;
    std::string err;
    const Json j = Json::parse(text, &err);
    if (!err.empty()) { r.errors.push_back("not valid JSON: " + err); return r; }
    if (!j.isObject()) { r.errors.emplace_back("expected a JSON object"); return r; }

    RobotSpec& s = r.spec;
    s.name = j["name"].isString() ? j["name"].string() : "robot";
    const Json& d = j["drive"];
    if (!d.isObject()) { r.errors.emplace_back("missing \"drive\""); return r; }

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
    if (j["mass_lb"].isNumber()) p.massKg = positive(j["mass_lb"], "mass_lb", 16.5) * kKgPerLb;
    if (p.trackWidthIn > p.robotWidthIn + 1.0)
        r.errors.emplace_back("drive.track_width is wider than footprint.width");

    // no "code" block = the code agrees with the robot
    s.declared = p;
    const Json& c = j["code"];
    double declaredCartridge = cartridge;
    if (c.isObject()) {
        s.declared.wheelDiameterIn = positive(c["wheel_diameter"], "code.wheel_diameter", p.wheelDiameterIn);
        s.declared.wheelRpm = positive(c["wheel_rpm"], "code.wheel_rpm", p.wheelRpm);
        declaredCartridge = positive(c["cartridge_rpm"], "code.cartridge_rpm", cartridge);
    }
    // EZ converts motor degrees with the ratio it was told; the motor turned by the real one
    s.encoderScale = (s.declared.wheelRpm / declaredCartridge) / (p.wheelRpm / cartridge);

    // tau ~ mass * top_speed^2 / power, anchored at 0.12 s for a 15 lb,
    // six 11 W motor, 3.25 in @ 450 rpm drive (the default plant)
    if (d["tau_s"].isNumber()) {
        s.motorTauSec = positive(d["tau_s"], "drive.tau_s", 0.12);
    } else {
        const int motors = 2 * static_cast<int>(std::lround(d["motors_per_side"].number(3)));
        const double watts = (d["motor"].isString() && d["motor"].string() == "5.5W") ? 5.5 : 11.0;
        const double massLb = p.massKg / kKgPerLb;
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
    if (!f) {
        RobotSpecResult r;
        r.errors.push_back("can't open " + path);
        return r;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return parseRobotSpec(ss.str());
}

}  // namespace visbot
