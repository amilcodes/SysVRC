// visbot/auton_file.hpp: read and write .auton files.
//
// An .auton file is a routine with the C++ taken out: one drive library call
// per line, arguments already evaluated. tools/auton_import.py generates them
// from autons.cpp; editing by hand is fine.
//
//     # worldsMogoRush(isBlue=true)
//     odom_xyt_set(0, 0, -89)           @1976
//     pid_drive_set(36, 127)            @1977  # set_drive(32 + 4, 2500, 126, 127)
//     delay(100)                        @1978
//     action("intake.move(-127)")       @1979
//     pid_wait_until(6)                 @1984
//
// `@N` is the line in the original source. `#` starts a comment.
//
// Calls (EZ names, EZ argument order):
//
//     pid_drive_set(inches, speed[, slew])
//     pid_turn_set(heading, speed[, shortest|longest|cw|ccw|raw])
//     pid_turn_to_point(x, y, speed[, fwd|rev])   EZ: pid_turn_set({x, y}, fwd, speed)
//     pid_turn_relative_set(delta, speed)
//     pid_swing_set(left|right, heading, speed[, opposite_speed])
//     pid_odom_set(x, y, speed[, fwd|rev])
//     drive_set(left, right)
//     pid_wait()  pid_wait_until(value)  pid_wait_quick()  pid_wait_quick_chain()
//     delay(ms)
//     pid_speed_max_set(speed)  pid_drive_chain_constant_set(inches)
//     pid_turn_chain_constant_set(deg)  slew_drive_constants_set(inches, min_speed)
//     odom_look_ahead_set(inches)
//     odom_xyt_set(x, y, heading)   first one before any motion = start pose
//     action("anything")            a mechanism call, logged but not simulated
//
// `# @field_start x y heading` gives the real start on the field (enables walls).
//
// LemLib routines start with `# @lib lemlib` and use LemLib's own calls, with
// the params struct written out the way it is in C++:
//
//     # @lib lemlib
//     # @lemlib drivetrain 10 4 360 2          track width, wheel, rpm, horizontal drift
//     # @lemlib lateral 10 0 3 3 1 100 3 500 20   ControllerSettings, same order
//     # @lemlib angular 2 0 10 3 1 100 3 500 0
//     # @lemlib vertical1 pod -2.5             tracking wheel: pod OFFSET, or
//     # @lemlib horizontal1 pod -5.75          left_drive / right_drive OFFSET DIAMETER RPM
//     # @lemlib imu on
//     # @lemlib throttle_curve 3 10 1.019      ExpoDriveCurve; also steer_curve
//     setPose(0, 0, 0)
//     moveToPose(20, 15, 90, 4000, {.forwards = false, .lead = 0.3})
//     turnToHeading(90, 1000, {.direction = CW_CLOCKWISE, .minSpeed = 100}, false)
//     swingToHeading(45, LEFT, 800)
//     follow(skills1_txt, 15, 4000)
//     waitUntil(10)  waitUntilDone()  cancelMotion()  cancelAllMotions()
//     tank(127, 127)  arcade(127, 0)
//
// follow() paths are carried in the file, between `# @path NAME` and `# @end`,
// one `#   x, y, speed` per line, as path.jerryio writes them.
#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "visbot/missions.hpp"
#include "visbot/motion.hpp"

namespace visbot {

struct AutonParseResult {
    MissionDef def;
    std::vector<std::string> errors;  // "line N: message"
    bool ok() const { return errors.empty(); }
};

namespace auton_detail {

inline std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

inline std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

/// Cut a line at the first `#` that isn't inside a quoted string.
inline std::string stripComment(const std::string& line) {
    bool inStr = false;
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"' && (i == 0 || line[i - 1] != '\\')) inStr = !inStr;
        if (line[i] == '#' && !inStr) return line.substr(0, i);
    }
    return line;
}

/// Split "a, {.b = 1, .c = 2}, \"d, e\"" on top-level commas.
inline std::vector<std::string> splitArgs(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    bool inStr = false;
    int depth = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (c == '"' && (i == 0 || s[i - 1] != '\\')) inStr = !inStr;
        if (!inStr && c == '{') ++depth;
        if (!inStr && c == '}') --depth;
        if (c == ',' && !inStr && depth == 0) { out.push_back(trim(cur)); cur.clear(); continue; }
        cur += c;
    }
    if (!trim(cur).empty() || !out.empty()) out.push_back(trim(cur));
    return out;
}

inline bool toNumber(const std::string& s, double& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end && *end == '\0';
}

inline std::string unquote(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
        std::string out;
        for (size_t i = 1; i + 1 < s.size(); ++i) {
            if (s[i] == '\\' && i + 2 < s.size()) { out += s[++i]; continue; }
            out += s[i];
        }
        return out;
    }
    return s;
}

inline std::string fmt(double v) {
    std::ostringstream o;
    o << v;
    return o.str();
}

inline bool endsWith(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}

/// "fwd" / "rev" (case-insensitive). False if it's neither.
inline bool parseDirection(const std::string& arg, DriveDirection& d) {
    const std::string k = lower(arg);
    if (k == "fwd") d = DriveDirection::Forward;
    else if (k == "rev") d = DriveDirection::Reverse;
    else return false;
    return true;
}

/// Everything except settings, delays and mechanism calls. The first one of
/// these ends the "setup" part where odom_xyt_set means the start pose.
inline bool movesRobot(Instr::Op op) {
    switch (op) {
        case Instr::Op::Action:
        case Instr::Op::Delay:
        case Instr::Op::TurnChainConstant:
        case Instr::Op::SlewDriveConstants:
        case Instr::Op::OdomLookAhead:
        case Instr::Op::DriveChainConstant:
        case Instr::Op::SpeedMax:
            return false;
        default:
            return true;
    }
}

/// "lemlib::DriveSide::LEFT" -> "LEFT"
inline std::string lastName(const std::string& s) {
    const size_t k = s.rfind("::");
    return trim(k == std::string::npos ? s : s.substr(k + 2));
}

inline bool parseBool(const std::string& s, bool& out) {
    const std::string k = lower(trim(s));
    if (k == "true" || k == "1") { out = true; return true; }
    if (k == "false" || k == "0") { out = false; return true; }
    return false;
}

inline const char* lemDirectionName(LemDirection d) {
    switch (d) {
        case LemDirection::CW: return "CW_CLOCKWISE";
        case LemDirection::CCW: return "CCW_COUNTERCLOCKWISE";
        case LemDirection::Auto: break;
    }
    return "AUTO";
}

/// The fields of each LemLib params struct, in declaration order.
inline std::vector<std::string> lemParamFields(Instr::Op op) {
    using Op = Instr::Op;
    switch (op) {
        case Op::LemMoveToPoint: return {"forwards", "maxSpeed", "minSpeed", "earlyExitRange"};
        case Op::LemMoveToPose: return {"forwards", "horizontalDrift", "lead", "maxSpeed", "minSpeed", "earlyExitRange"};
        case Op::LemTurnToPoint:
        case Op::LemSwingToPoint: return {"forwards", "direction", "maxSpeed", "minSpeed", "earlyExitRange"};
        case Op::LemTurnToHeading:
        case Op::LemSwingToHeading: return {"direction", "maxSpeed", "minSpeed", "earlyExitRange"};
        default: return {};
    }
}

/// `{.forwards = false, .maxSpeed = 80}` into `a`. Returns an error, or "".
inline std::string parseLemParams(const std::string& text, Instr::Op op, LemArgs& a) {
    const std::string t = trim(text);
    if (t.size() < 2 || t.front() != '{' || t.back() != '}') return "params must look like {.maxSpeed = 80}";
    const std::vector<std::string> fields = lemParamFields(op);
    for (const std::string& item : splitArgs(t.substr(1, t.size() - 2))) {
        if (item.empty()) continue;
        const size_t eq = item.find('=');
        if (item[0] != '.' || eq == std::string::npos) return "params must be designated, e.g. {.maxSpeed = 80}";
        const std::string key = trim(item.substr(1, eq - 1));
        const std::string val = trim(item.substr(eq + 1));
        if (std::find(fields.begin(), fields.end(), key) == fields.end()) {
            Instr named;
            named.op = op;
            return std::string(named.name()) + " has no ." + key;
        }
        double v = 0;
        if (key == "forwards") {
            if (!parseBool(val, a.forwards)) return ".forwards must be true or false";
        } else if (key == "direction") {
            const std::string d = lastName(val);
            if (d == "CW_CLOCKWISE") a.direction = LemDirection::CW;
            else if (d == "CCW_COUNTERCLOCKWISE") a.direction = LemDirection::CCW;
            else if (d == "AUTO") a.direction = LemDirection::Auto;
            else return "unknown direction '" + val + "'";
        } else if (!toNumber(val, v)) {
            return "." + key + " must be a number";
        } else if (key == "maxSpeed") a.maxSpeed = v;
        else if (key == "minSpeed") a.minSpeed = v;
        else if (key == "earlyExitRange") a.earlyExitRange = v;
        else if (key == "lead") a.lead = v;
        else if (key == "horizontalDrift") a.horizontalDrift = v;
    }
    return "";
}

/// The params struct back out, leaving out anything at its default.
inline std::string lemParamsText(const Instr& in) {
    const LemArgs d;
    std::vector<std::string> parts;
    for (const std::string& k : lemParamFields(in.op)) {
        if (k == "forwards" && in.lem.forwards != d.forwards) parts.emplace_back(".forwards = false");
        if (k == "direction" && in.lem.direction != d.direction)
            parts.push_back(std::string(".direction = ") + lemDirectionName(in.lem.direction));
        if (k == "horizontalDrift" && in.lem.horizontalDrift != d.horizontalDrift)
            parts.push_back(".horizontalDrift = " + fmt(in.lem.horizontalDrift));
        if (k == "lead" && in.lem.lead != d.lead) parts.push_back(".lead = " + fmt(in.lem.lead));
        if (k == "maxSpeed" && in.lem.maxSpeed != d.maxSpeed) parts.push_back(".maxSpeed = " + fmt(in.lem.maxSpeed));
        if (k == "minSpeed" && in.lem.minSpeed != d.minSpeed) parts.push_back(".minSpeed = " + fmt(in.lem.minSpeed));
        if (k == "earlyExitRange" && in.lem.earlyExitRange != d.earlyExitRange)
            parts.push_back(".earlyExitRange = " + fmt(in.lem.earlyExitRange));
    }
    std::string out = "{";
    for (size_t i = 0; i < parts.size(); ++i) out += (i ? ", " : "") + parts[i];
    return out + "}";
}

inline std::string wheelText(const LemConfig::Wheel& w) {
    using Kind = LemConfig::Wheel::Kind;
    switch (w.kind) {
        case Kind::Pod: return "pod " + fmt(w.offset);
        case Kind::LeftDrive: return "left_drive " + fmt(w.offset) + " " + fmt(w.diameter) + " " + fmt(w.rpm);
        case Kind::RightDrive: return "right_drive " + fmt(w.offset) + " " + fmt(w.diameter) + " " + fmt(w.rpm);
        case Kind::None: break;
    }
    return "none";
}

/// One `# @lemlib ...` header line. Returns an error, or "".
inline std::string parseLemDirective(const std::string& rest, LemConfig& c) {
    std::istringstream in(rest);
    std::string key;
    in >> key;
    std::vector<double> v;
    auto numbers = [&](size_t n) {
        double x = 0;
        while (in >> x) v.push_back(x);
        return v.size() == n;
    };
    if (key == "drivetrain") {
        if (!numbers(4)) return "@lemlib drivetrain wants track_width wheel_diameter rpm horizontal_drift";
        c.trackWidth = v[0];
        c.wheelDiameter = v[1];
        c.rpm = v[2];
        c.horizontalDrift = v[3];
    } else if (key == "lateral" || key == "angular") {
        if (!numbers(9)) return "@lemlib " + key + " wants the 9 ControllerSettings values";
        LemConfig::Controller& k = key == "lateral" ? c.lateral : c.angular;
        k = LemConfig::Controller{v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]};
    } else if (key == "vertical1" || key == "vertical2" || key == "horizontal1" || key == "horizontal2") {
        LemConfig::Wheel& w = key == "vertical1" ? c.vertical1 : key == "vertical2" ? c.vertical2
                            : key == "horizontal1" ? c.horizontal1 : c.horizontal2;
        std::string kind;
        in >> kind;
        using Kind = LemConfig::Wheel::Kind;
        if (kind == "none") {
            w = LemConfig::Wheel{};
        } else if (kind == "pod") {
            if (!numbers(1) && v.size() != 2) return "@lemlib " + key + " pod wants OFFSET [DIAMETER]";
            w = LemConfig::Wheel{Kind::Pod, v[0], v.size() > 1 ? v[1] : 2.75, 0.0};
        } else if (kind == "left_drive" || kind == "right_drive") {
            if (!numbers(3)) return "@lemlib " + key + " " + kind + " wants OFFSET DIAMETER RPM";
            w = LemConfig::Wheel{kind == "left_drive" ? Kind::LeftDrive : Kind::RightDrive, v[0], v[1], v[2]};
        } else {
            return "@lemlib " + key + " wants pod, left_drive, right_drive or none";
        }
    } else if (key == "imu") {
        std::string onOff;
        in >> onOff;
        if (onOff != "on" && onOff != "off") return "@lemlib imu wants on or off";
        c.imu = onOff == "on";
    } else if (key == "throttle_curve" || key == "steer_curve") {
        if (!numbers(3)) return "@lemlib " + key + " wants deadband min_output gain";
        LemConfig::Curve& curve = key == "throttle_curve" ? c.throttle : c.steer;
        curve = LemConfig::Curve{v[0], v[1], v[2]};
    } else {
        return "unknown @lemlib setting '" + key + "'";
    }
    return "";
}

}  // namespace auton_detail

/// Parse the text of an .auton file.
inline AutonParseResult parseAuton(const std::string& text, const std::string& fallbackName = "auton") {
    using namespace auton_detail;
    AutonParseResult r;
    r.def.name = fallbackName;
    bool sawMotion = false;
    std::string pathName;               // inside a `# @path` block
    std::vector<std::pair<std::string, int>> follows;  // checked once every path is known

    std::istringstream in(text);
    std::string raw;
    int lineNo = 0;
    while (std::getline(in, raw)) {
        ++lineNo;
        auto headerError = [&](const std::string& m) {
            r.errors.push_back("line " + std::to_string(lineNo) + ": " + m);
        };
        // header lines: `# @name foo`, `# @field_start x y h`, `# @lib`, `# @lemlib`, `# @path`
        {
            const std::string t = trim(raw);
            if (!pathName.empty()) {
                if (t.rfind("# @end", 0) == 0) { pathName.clear(); continue; }
                const std::vector<std::string> xyv = splitArgs(trim(t.substr(t.empty() ? 0 : 1)));
                double x = 0, y = 0, v = 0;
                if (t.empty() || t[0] != '#' || xyv.size() != 3 || !toNumber(xyv[0], x) || !toNumber(xyv[1], y) ||
                    !toNumber(xyv[2], v)) {
                    headerError("path points are `#   x, y, speed` (did the @path block miss its # @end?)");
                    pathName.clear();
                    continue;
                }
                r.def.lem.paths[pathName].push_back({x, y, v});
                continue;
            }
            if (t.rfind("# @name", 0) == 0) { r.def.name = trim(t.substr(7)); continue; }
            if (t.rfind("# @field_start", 0) == 0) {
                std::istringstream fs(t.substr(14));
                Pose p;
                if (fs >> p.x >> p.y >> p.theta) { r.def.fieldStart = p; r.def.hasFieldStart = true; }
                else headerError("@field_start wants x y heading");
                continue;
            }
            if (t.rfind("# @lib", 0) == 0) {
                const std::string lib = lower(trim(t.substr(6)));
                if (lib == "lemlib") r.def.lib = Library::LemLib;
                else if (lib == "ez-template" || lib == "ez") r.def.lib = Library::EzTemplate;
                else headerError("@lib is lemlib or ez-template");
                continue;
            }
            if (t.rfind("# @lemlib", 0) == 0) {
                const std::string e = parseLemDirective(t.substr(9), r.def.lem);
                if (!e.empty()) headerError(e);
                continue;
            }
            if (t.rfind("# @path", 0) == 0) {
                pathName = trim(t.substr(7));
                if (pathName.empty()) headerError("@path needs a name");
                r.def.lem.paths[pathName].clear();
                continue;
            }
        }
        std::string line = trim(stripComment(raw));
        if (line.empty()) continue;

        auto err = [&](const std::string& m) {
            r.errors.push_back("line " + std::to_string(lineNo) + ": " + m + "  [" + line + "]");
        };

        // trailing @N = source line (only outside quotes)
        int src = 0;
        const size_t at = line.rfind('@');
        if (at != std::string::npos && at > 0 && line.find('"', at) == std::string::npos) {
            double v = 0;
            if (toNumber(trim(line.substr(at + 1)), v)) {
                src = static_cast<int>(v);
                line = trim(line.substr(0, at));
            }
        }

        const size_t open = line.find('(');
        const size_t close = line.rfind(')');
        if (open == std::string::npos || close == std::string::npos || close < open) {
            err("expected name(args)");
            continue;
        }
        const std::string name = lower(trim(line.substr(0, open)));
        const std::vector<std::string> args = splitArgs(line.substr(open + 1, close - open - 1));

        std::vector<double> num(args.size(), 0.0);
        std::vector<bool> isNum(args.size(), false);
        for (size_t i = 0; i < args.size(); ++i) isNum[i] = toNumber(args[i], num[i]);

        auto need = [&](size_t lo, size_t hi) {
            if (args.size() < lo || args.size() > hi) {
                err(name + " takes " + std::to_string(lo) +
                    (hi != lo ? "-" + std::to_string(hi) : std::string()) + " arguments");
                return false;
            }
            return true;
        };
        auto numArg = [&](size_t i) {
            if (!isNum[i]) { err("argument " + std::to_string(i + 1) + " of " + name + " must be a number"); return false; }
            return true;
        };

        Instr ins;
        bool emit = true;
        const bool lem = r.def.lib == Library::LemLib;
        // a LemLib motion: the fixed arguments, then {params}, then async
        auto lemMotion = [&](Instr::Op op, size_t fixed) {
            ins.op = op;
            if (args.size() < fixed || args.size() > fixed + 2) {
                err(name + " takes " + std::to_string(fixed) + " arguments, then {params} and async");
                return false;
            }
            size_t k = fixed;
            if (k < args.size() && !args[k].empty() && args[k][0] == '{') {
                const std::string e = parseLemParams(args[k], op, ins.lem);
                if (!e.empty()) { err(e); return false; }
                ++k;
            }
            if (k < args.size() && !parseBool(args[k++], ins.lem.async)) { err("async must be true or false"); return false; }
            if (k != args.size()) { err("{params} goes before async"); return false; }
            return true;
        };
        auto swingSide = [&](size_t i) {
            const std::string k = lastName(args[i]);
            if (k == "LEFT") { ins.lem.lockedSide = SwingSide::Left; return true; }
            if (k == "RIGHT") { ins.lem.lockedSide = SwingSide::Right; return true; }
            err("lockedSide must be LEFT or RIGHT");
            return false;
        };
        if (name == "delay") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::delay(num[0]);
        } else if (name == "action") {
            if (!need(1, 1)) continue;
            ins = Instr::act(unquote(args[0]));
        } else if (lem) {
            using Op = Instr::Op;
            if (name == "setpose") {
                if (!need(3, 3) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
                if (!sawMotion) {
                    r.def.start = {num[0], num[1], num[2]};  // the start pose, like odom_xyt_set
                    emit = false;
                } else {
                    ins.op = Op::LemSetPose;
                    ins.a = num[0];
                    ins.b = num[1];
                    ins.lem.theta = num[2];
                }
            } else if (name == "movetopoint") {
                if (!lemMotion(Op::LemMoveToPoint, 3) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
                ins.a = num[0];
                ins.b = num[1];
                ins.lem.timeoutMs = num[2];
            } else if (name == "movetopose") {
                if (!lemMotion(Op::LemMoveToPose, 4) || !numArg(0) || !numArg(1) || !numArg(2) || !numArg(3)) continue;
                ins.a = num[0];
                ins.b = num[1];
                ins.lem.theta = num[2];
                ins.lem.timeoutMs = num[3];
            } else if (name == "turntoheading") {
                if (!lemMotion(Op::LemTurnToHeading, 2) || !numArg(0) || !numArg(1)) continue;
                ins.lem.theta = num[0];
                ins.lem.timeoutMs = num[1];
            } else if (name == "turntopoint") {
                if (!lemMotion(Op::LemTurnToPoint, 3) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
                ins.a = num[0];
                ins.b = num[1];
                ins.lem.timeoutMs = num[2];
            } else if (name == "swingtoheading") {
                if (!lemMotion(Op::LemSwingToHeading, 3) || !numArg(0) || !swingSide(1) || !numArg(2)) continue;
                ins.lem.theta = num[0];
                ins.lem.timeoutMs = num[2];
            } else if (name == "swingtopoint") {
                if (!lemMotion(Op::LemSwingToPoint, 4) || !numArg(0) || !numArg(1) || !swingSide(2) || !numArg(3)) continue;
                ins.a = num[0];
                ins.b = num[1];
                ins.lem.timeoutMs = num[3];
            } else if (name == "follow") {
                // follow(path, lookahead, timeout[, forwards][, async])
                if (!need(3, 5) || !numArg(1) || !numArg(2)) continue;
                ins.op = Op::LemFollow;
                ins.lem.path = args[0];
                ins.lem.lookahead = num[1];
                ins.lem.timeoutMs = num[2];
                if (args.size() > 3 && !parseBool(args[3], ins.lem.forwards)) { err("forwards must be true or false"); continue; }
                if (args.size() > 4 && !parseBool(args[4], ins.lem.async)) { err("async must be true or false"); continue; }
                follows.emplace_back(ins.lem.path, lineNo);
            } else if (name == "waituntil") {
                if (!need(1, 1) || !numArg(0)) continue;
                ins.op = Op::LemWaitUntil;
                ins.a = num[0];
            } else if (name == "waituntildone" || name == "cancelmotion" || name == "cancelallmotions") {
                if (!need(0, 0)) continue;
                ins.op = name == "waituntildone" ? Op::LemWaitUntilDone
                       : name == "cancelmotion" ? Op::LemCancelMotion : Op::LemCancelAllMotions;
            } else if (name == "tank" || name == "arcade") {
                const bool tank = name == "tank";
                if (!need(2, tank ? 3 : 4) || !numArg(0) || !numArg(1)) continue;
                ins.op = tank ? Op::LemTank : Op::LemArcade;
                ins.a = num[0];
                ins.b = num[1];
                if (args.size() > 2 && !parseBool(args[2], ins.lem.disableDriveCurve)) {
                    err("disableDriveCurve must be true or false");
                    continue;
                }
                if (args.size() > 3) {
                    if (!numArg(3)) continue;
                    ins.lem.desaturateBias = num[3];
                }
            } else {
                err("unknown LemLib call '" + name + "'");
                continue;
            }
        } else if (name == "pid_drive_set") {
            if (!need(2, 3) || !numArg(0) || !numArg(1)) continue;
            bool slew = true;
            if (args.size() == 3) slew = lower(args[2]) != "false" && args[2] != "0";
            ins = Instr::driveSet(num[0], num[1], slew);
        } else if (name == "pid_turn_set") {
            if (!need(2, 3) || !numArg(0) || !numArg(1)) continue;
            AngleBehavior b = AngleBehavior::Shortest;
            if (args.size() == 3) {
                const std::string k = lower(args[2]);
                if (k == "longest") b = AngleBehavior::Longest;
                else if (k == "cw") b = AngleBehavior::CW;
                else if (k == "ccw") b = AngleBehavior::CCW;
                else if (k == "raw") b = AngleBehavior::Raw;
                else if (k != "shortest") { err("unknown turn behaviour '" + args[2] + "'"); continue; }
            }
            ins = Instr::turnSet(num[0], num[1], b);
        } else if (name == "pid_turn_to_point") {
            if (!need(3, 4) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
            DriveDirection d = DriveDirection::Forward;
            if (args.size() == 4 && !parseDirection(args[3], d)) { err("direction must be fwd or rev"); continue; }
            ins = Instr::turnToPoint(num[0], num[1], num[2], d);
        } else if (name == "pid_turn_relative_set") {
            if (!need(2, 2) || !numArg(0) || !numArg(1)) continue;
            ins = Instr::turnRelative(num[0], num[1]);
        } else if (name == "pid_turn_chain_constant_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::turnChainConstant(num[0]);
        } else if (name == "slew_drive_constants_set") {
            if (!need(2, 2) || !numArg(0) || !numArg(1)) continue;
            ins = Instr::slewDriveConstants(num[0], num[1]);
        } else if (name == "odom_look_ahead_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::odomLookAhead(num[0]);
        } else if (name == "pid_swing_set") {
            if (!need(3, 4) || !numArg(1) || !numArg(2)) continue;
            const std::string side = lower(args[0]);
            if (side != "left" && side != "right") { err("swing side must be left or right"); continue; }
            double opp = 0.0;
            if (args.size() == 4) { if (!numArg(3)) continue; opp = num[3]; }
            ins = Instr::swingSet(side == "left" ? SwingSide::Left : SwingSide::Right, num[1], num[2], opp);
        } else if (name == "pid_odom_set") {
            if (!need(3, 4) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
            DriveDirection d = DriveDirection::Forward;
            if (args.size() == 4 && !parseDirection(args[3], d)) { err("direction must be fwd or rev"); continue; }
            ins = Instr::odomSet(num[0], num[1], num[2], d);
        } else if (name == "drive_set") {
            if (!need(2, 2) || !numArg(0) || !numArg(1)) continue;
            ins = Instr::driveRaw(num[0], num[1]);
        } else if (name == "pid_wait") {
            if (!need(0, 0)) continue;
            ins = Instr::wait();
        } else if (name == "pid_wait_until") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::waitUntil(num[0]);
        } else if (name == "pid_wait_quick") {
            if (!need(0, 0)) continue;
            ins = Instr::waitQuick();
        } else if (name == "pid_wait_quick_chain") {
            if (!need(0, 0)) continue;
            ins = Instr::waitQuickChain();
        } else if (name == "pid_speed_max_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::speedMax(num[0]);
        } else if (name == "pid_drive_chain_constant_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::driveChainConstant(num[0]);
        } else if (name == "odom_xyt_set") {
            if (!need(3, 3) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
            if (!sawMotion) {
                r.def.start = {num[0], num[1], num[2]};  // before any motion it's the start pose, not a reset
                emit = false;
            } else {
                ins = Instr::odomReset(num[0], num[1], num[2]);
            }
        } else {
            err("unknown call '" + name + "'" +
                (name == "movetopoint" || name == "movetopose" ? " (LemLib? add # @lib lemlib)" : ""));
            continue;
        }

        if (!emit) continue;
        if (movesRobot(ins.op)) sawMotion = true;
        ins.sourceLine = src;
        r.def.mission.push_back(std::move(ins));
    }
    for (const auto& [path, line] : follows) {
        const auto it = r.def.lem.paths.find(path);
        if (it == r.def.lem.paths.end() || it->second.empty())
            r.errors.push_back("line " + std::to_string(line) + ": follow(" + path + ", ...) but there's no # @path " +
                               path + " block with its points");
    }
    if (r.def.mission.empty() && r.errors.empty()) r.errors.emplace_back("no instructions");
    return r;
}

/// Read and parse an .auton file. Named after the file unless it has `# @name`.
inline AutonParseResult loadAutonFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        AutonParseResult r;
        r.errors.push_back("can't open " + path);
        return r;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const size_t slash = path.find_last_of("/\\");
    std::string stem = slash == std::string::npos ? path : path.substr(slash + 1);
    if (auton_detail::endsWith(stem, ".auton")) stem.resize(stem.size() - 6);
    return parseAuton(ss.str(), stem);
}

/// Look up a routine the way a user would name it: built-in (`skills`), a path
/// (`autons/foo.auton`), or a bare name in autonsDir (`worlds_mogo_rush.blue`).
inline AutonParseResult resolveMission(const std::string& nameOrPath, const std::string& autonsDir) {
    if (missions::isBuiltin(nameOrPath)) {
        AutonParseResult r;
        r.def = missions::byName(nameOrPath);
        return r;
    }
    const bool looksLikePath = nameOrPath.find('/') != std::string::npos || auton_detail::endsWith(nameOrPath, ".auton");
    if (looksLikePath) return loadAutonFile(nameOrPath);
    AutonParseResult r = loadAutonFile(autonsDir + "/" + nameOrPath + ".auton");
    if (!r.ok() && r.errors.size() == 1 && r.errors[0].rfind("can't open", 0) == 0)
        r.errors[0] = "no routine called '" + nameOrPath + "' (not built in, and not in " + autonsDir + ")";
    return r;
}

/// Where the robot really starts: @field_start if given, else odom_xyt_set.
inline Pose physicalStart(const MissionDef& def) { return def.hasFieldStart ? def.fieldStart : def.start; }

/// One instruction as a line of .auton text, e.g. `pid_drive_set(36, 127)`.
inline std::string toAutonLine(const Instr& in) {
    using auton_detail::fmt;
    auto side = [](const Instr& i) { return i.lem.lockedSide == SwingSide::Left ? "LEFT" : "RIGHT"; };
    // `, {params}` and `, false` for a synchronous call, only when they say something
    auto lemTail = [](const Instr& i) {
        const std::string p = auton_detail::lemParamsText(i);
        std::string t;
        if (p != "{}" || !i.lem.async) t += ", " + p;
        if (!i.lem.async) t += ", false";
        return t;
    };
    auto behaviour = [](AngleBehavior b) {
        switch (b) {
            case AngleBehavior::Shortest: return "shortest";
            case AngleBehavior::Longest:  return "longest";
            case AngleBehavior::CW:       return "cw";
            case AngleBehavior::CCW:      return "ccw";
            case AngleBehavior::Raw:      return "raw";
        }
        return "shortest";
    };
    std::ostringstream o;
    o << in.name() << "(";
    switch (in.op) {
        case Instr::Op::DriveSet:
            o << fmt(in.a) << ", " << fmt(in.speed) << (in.slew ? "" : ", false"); break;
        case Instr::Op::TurnSet:
            o << fmt(in.a) << ", " << fmt(in.speed);
            if (in.behavior != AngleBehavior::Shortest) o << ", " << behaviour(in.behavior);
            break;
        case Instr::Op::TurnToPoint:
            o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.speed)
              << (in.dir == DriveDirection::Reverse ? ", rev" : ""); break;
        case Instr::Op::TurnRelative:
            o << fmt(in.a) << ", " << fmt(in.speed); break;
        case Instr::Op::SwingSet:
            o << (in.side == SwingSide::Left ? "left" : "right") << ", " << fmt(in.a) << ", " << fmt(in.speed);
            if (in.b != 0.0) o << ", " << fmt(in.b);
            break;
        case Instr::Op::OdomSet:
            o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.speed)
              << (in.dir == DriveDirection::Reverse ? ", rev" : ""); break;
        case Instr::Op::DriveRaw:
        case Instr::Op::SlewDriveConstants: o << fmt(in.a) << ", " << fmt(in.b); break;
        case Instr::Op::WaitUntil:
        case Instr::Op::Delay:
        case Instr::Op::DriveChainConstant:
        case Instr::Op::TurnChainConstant:
        case Instr::Op::OdomLookAhead:      o << fmt(in.a); break;
        case Instr::Op::SpeedMax:           o << fmt(in.speed); break;
        case Instr::Op::OdomReset:          o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.speed); break;
        case Instr::Op::Action: {
            o << '"';
            for (char c : in.label) { if (c == '"' || c == '\\') o << '\\'; o << c; }
            o << '"';
            break;
        }
        case Instr::Op::Wait:
        case Instr::Op::WaitQuick:
        case Instr::Op::WaitQuickChain:     break;
        // LemLib: x, y in a, b; the rest in lem
        case Instr::Op::LemMoveToPoint:
        case Instr::Op::LemTurnToPoint:
            o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.lem.timeoutMs) << lemTail(in); break;
        case Instr::Op::LemMoveToPose:
            o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.lem.theta) << ", " << fmt(in.lem.timeoutMs)
              << lemTail(in); break;
        case Instr::Op::LemTurnToHeading:
            o << fmt(in.lem.theta) << ", " << fmt(in.lem.timeoutMs) << lemTail(in); break;
        case Instr::Op::LemSwingToHeading:
            o << fmt(in.lem.theta) << ", " << side(in) << ", " << fmt(in.lem.timeoutMs) << lemTail(in); break;
        case Instr::Op::LemSwingToPoint:
            o << fmt(in.a) << ", " << fmt(in.b) << ", " << side(in) << ", " << fmt(in.lem.timeoutMs) << lemTail(in);
            break;
        case Instr::Op::LemFollow:
            o << in.lem.path << ", " << fmt(in.lem.lookahead) << ", " << fmt(in.lem.timeoutMs);
            if (!in.lem.forwards || !in.lem.async) o << ", " << (in.lem.forwards ? "true" : "false");
            if (!in.lem.async) o << ", false";
            break;
        case Instr::Op::LemWaitUntil:       o << fmt(in.a); break;
        case Instr::Op::LemSetPose:         o << fmt(in.a) << ", " << fmt(in.b) << ", " << fmt(in.lem.theta); break;
        case Instr::Op::LemTank:
        case Instr::Op::LemArcade:
            o << fmt(in.a) << ", " << fmt(in.b);
            if (in.lem.disableDriveCurve || in.lem.desaturateBias != 0.5) o << ", " << (in.lem.disableDriveCurve ? "true" : "false");
            if (in.op == Instr::Op::LemArcade && in.lem.desaturateBias != 0.5) o << ", " << fmt(in.lem.desaturateBias);
            break;
        case Instr::Op::LemWaitUntilDone:
        case Instr::Op::LemCancelMotion:
        case Instr::Op::LemCancelAllMotions: break;
    }
    o << ")";
    return o.str();
}

/// Write a mission back out in .auton form.
inline std::string toAutonText(const MissionDef& def) {
    using auton_detail::fmt;
    std::ostringstream o;
    o << "# @name " << def.name << "\n";
    if (def.hasFieldStart)
        o << "# @field_start " << fmt(def.fieldStart.x) << " " << fmt(def.fieldStart.y) << " "
          << fmt(def.fieldStart.theta) << "\n";
    const bool lem = def.lib == Library::LemLib;
    if (lem) {
        const LemConfig& c = def.lem;
        auto controller = [&](const char* name, const LemConfig::Controller& k) {
            o << "# @lemlib " << name << " " << fmt(k.kP) << " " << fmt(k.kI) << " " << fmt(k.kD) << " "
              << fmt(k.windupRange) << " " << fmt(k.smallError) << " " << fmt(k.smallErrorTimeout) << " "
              << fmt(k.largeError) << " " << fmt(k.largeErrorTimeout) << " " << fmt(k.slew) << "\n";
        };
        o << "# @lib lemlib\n";
        o << "# @lemlib drivetrain " << fmt(c.trackWidth) << " " << fmt(c.wheelDiameter) << " " << fmt(c.rpm) << " "
          << fmt(c.horizontalDrift) << "\n";
        controller("lateral", c.lateral);
        controller("angular", c.angular);
        const std::pair<const char*, const LemConfig::Wheel*> wheels[] = {
            {"vertical1", &c.vertical1}, {"vertical2", &c.vertical2},
            {"horizontal1", &c.horizontal1}, {"horizontal2", &c.horizontal2}};
        for (const auto& [name, w] : wheels)
            if (w->kind != LemConfig::Wheel::Kind::None) o << "# @lemlib " << name << " " << auton_detail::wheelText(*w) << "\n";
        o << "# @lemlib imu " << (c.imu ? "on" : "off") << "\n";
        o << "# @lemlib throttle_curve " << fmt(c.throttle.deadband) << " " << fmt(c.throttle.minOutput) << " "
          << fmt(c.throttle.gain) << "\n";
        o << "# @lemlib steer_curve " << fmt(c.steer.deadband) << " " << fmt(c.steer.minOutput) << " "
          << fmt(c.steer.gain) << "\n";
        for (const auto& [name, pts] : c.paths) {
            o << "# @path " << name << "\n";
            for (const auto& pt : pts) o << "#   " << fmt(pt.x) << ", " << fmt(pt.y) << ", " << fmt(pt.speed) << "\n";
            o << "# @end\n";
        }
        o << "setPose(";
    } else {
        o << "odom_xyt_set(";
    }
    o << fmt(def.start.x) << ", " << fmt(def.start.y) << ", " << fmt(def.start.theta) << ")\n";
    for (const Instr& in : def.mission) {
        o << toAutonLine(in);
        if (in.sourceLine > 0) o << "  @" << in.sourceLine;
        o << "\n";
    }
    return o.str();
}

}  // namespace visbot
