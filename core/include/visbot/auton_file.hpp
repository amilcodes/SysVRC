// visbot/auton_file.hpp — read and write `.auton` files.
//
// An .auton file is an EZ-Template routine with the C++ taken out: one EZ call
// per line, arguments already evaluated. tools/ez_import.py writes them
// straight from autons.cpp, and they can be edited by hand.
//
//     # worldsMogoRush(isBlue=true)
//     odom_xyt_set(0, 0, -89)           @1976
//     pid_drive_set(36, 127)            @1977  # set_drive(32 + 4, 2500, 126, 127)
//     delay(100)                        @1978
//     action("intake.move(-127)")       @1979
//     pid_wait_until(6)                 @1984
//
// `@N` is the line in the original source, so a report can point back at the
// exact line of the auton that misbehaved. Anything after `#` is a comment.
//
// Recognised calls (EZ names, EZ argument order):
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
// Optional header: `# @field_start x y heading` gives the robot's real start
// in field coordinates, which lets the sim model the perimeter walls.
#pragma once

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

/// Split "a, b, \"c, d\"" on top-level commas.
inline std::vector<std::string> splitArgs(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    bool inStr = false;
    for (size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (c == '"' && (i == 0 || s[i - 1] != '\\')) inStr = !inStr;
        if (c == ',' && !inStr) { out.push_back(trim(cur)); cur.clear(); continue; }
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

}  // namespace auton_detail

/// Parse the text of an .auton file.
inline AutonParseResult parseAuton(const std::string& text, const std::string& fallbackName = "auton") {
    using namespace auton_detail;
    AutonParseResult r;
    r.def.name = fallbackName;
    bool sawMotion = false;

    std::istringstream in(text);
    std::string raw;
    int lineNo = 0;
    while (std::getline(in, raw)) {
        ++lineNo;
        // `# @name foo` sets the routine name.
        {
            const std::string t = trim(raw);
            if (t.rfind("# @name", 0) == 0) { r.def.name = trim(t.substr(7)); continue; }
            if (t.rfind("# @field_start", 0) == 0) {
                std::istringstream fs(t.substr(14));
                Pose p;
                if (fs >> p.x >> p.y >> p.theta) { r.def.fieldStart = p; r.def.hasFieldStart = true; }
                else r.errors.push_back("line " + std::to_string(lineNo) + ": @field_start wants x y heading");
                continue;
            }
        }
        std::string line = trim(stripComment(raw));
        if (line.empty()) continue;

        auto err = [&](const std::string& m) {
            r.errors.push_back("line " + std::to_string(lineNo) + ": " + m + "  [" + line + "]");
        };

        // Trailing `@N` source-line marker.
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
        if (name == "pid_drive_set") {
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
            if (args.size() == 4) {
                const std::string k = lower(args[3]);
                if (k == "rev") d = DriveDirection::Reverse;
                else if (k != "fwd") { err("direction must be fwd or rev"); continue; }
            }
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
            if (args.size() == 4) {
                const std::string k = lower(args[3]);
                if (k == "rev") d = DriveDirection::Reverse;
                else if (k != "fwd") { err("direction must be fwd or rev"); continue; }
            }
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
        } else if (name == "delay") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::delay(num[0]);
        } else if (name == "pid_speed_max_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::speedMax(num[0]);
        } else if (name == "pid_drive_chain_constant_set") {
            if (!need(1, 1) || !numArg(0)) continue;
            ins = Instr::driveChainConstant(num[0]);
        } else if (name == "odom_xyt_set") {
            if (!need(3, 3) || !numArg(0) || !numArg(1) || !numArg(2)) continue;
            if (!sawMotion) {
                r.def.start = {num[0], num[1], num[2]};
                emit = false;
            } else {
                ins = Instr::odomReset(num[0], num[1], num[2]);
            }
        } else if (name == "action") {
            if (!need(1, 1)) continue;
            ins = Instr::act(unquote(args[0]));
        } else {
            err("unknown call '" + name + "'");
            continue;
        }

        if (!emit) continue;
        const bool setting = ins.op == Instr::Op::Action || ins.op == Instr::Op::Delay ||
                             ins.op == Instr::Op::TurnChainConstant || ins.op == Instr::Op::SlewDriveConstants ||
                             ins.op == Instr::Op::OdomLookAhead || ins.op == Instr::Op::DriveChainConstant ||
                             ins.op == Instr::Op::SpeedMax;
        if (!setting) sawMotion = true;
        ins.sourceLine = src;
        r.def.mission.push_back(std::move(ins));
    }
    if (r.def.mission.empty() && r.errors.empty()) r.errors.push_back("no instructions");
    return r;
}

/// Read and parse an .auton file. The routine is named after the file unless
/// the file says otherwise with `# @name`.
inline AutonParseResult loadAutonFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        AutonParseResult r;
        r.errors.push_back("can't open " + path);
        return r;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string stem = path.substr(path.find_last_of("/\\") == std::string::npos ? 0 : path.find_last_of("/\\") + 1);
    if (stem.size() > 6 && stem.compare(stem.size() - 6, 6, ".auton") == 0) stem.resize(stem.size() - 6);
    return parseAuton(ss.str(), stem);
}

/// Find a routine by what a user would type: a built-in name (`skills`), a
/// path (`autons/foo.auton`), or a bare name looked up in `autonsDir`
/// (`worlds_mogo_rush.blue`).
inline AutonParseResult resolveMission(const std::string& nameOrPath, const std::string& autonsDir) {
    if (missions::isBuiltin(nameOrPath)) {
        AutonParseResult r;
        r.def = missions::byName(nameOrPath);
        return r;
    }
    const bool looksLikePath = nameOrPath.find('/') != std::string::npos ||
                               (nameOrPath.size() > 6 && nameOrPath.compare(nameOrPath.size() - 6, 6, ".auton") == 0);
    if (looksLikePath) return loadAutonFile(nameOrPath);
    AutonParseResult r = loadAutonFile(autonsDir + "/" + nameOrPath + ".auton");
    if (!r.ok() && r.errors.size() == 1 && r.errors[0].rfind("can't open", 0) == 0)
        r.errors[0] = "no routine called '" + nameOrPath + "' (not built in, and not in " + autonsDir + ")";
    return r;
}

/// Where the robot physically starts: the file's @field_start if it has one,
/// otherwise the pose the code declares.
inline Pose physicalStart(const MissionDef& def) { return def.hasFieldStart ? def.fieldStart : def.start; }

/// One instruction as a line of .auton text, e.g. `pid_drive_set(36, 127)`.
inline std::string toAutonLine(const Instr& in) {
    using auton_detail::fmt;
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
        case Instr::Op::DriveRaw:           o << fmt(in.a) << ", " << fmt(in.b); break;
        case Instr::Op::WaitUntil:          o << fmt(in.a); break;
        case Instr::Op::Delay:              o << fmt(in.a); break;
        case Instr::Op::SpeedMax:           o << fmt(in.speed); break;
        case Instr::Op::DriveChainConstant:
        case Instr::Op::TurnChainConstant:
        case Instr::Op::OdomLookAhead:      o << fmt(in.a); break;
        case Instr::Op::SlewDriveConstants: o << fmt(in.a) << ", " << fmt(in.b); break;
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
    o << "odom_xyt_set(" << fmt(def.start.x) << ", " << fmt(def.start.y) << ", " << fmt(def.start.theta) << ")\n";
    for (const Instr& in : def.mission) {
        o << toAutonLine(in);
        if (in.sourceLine > 0) o << "  @" << in.sourceLine;
        o << "\n";
    }
    return o.str();
}

}  // namespace visbot
