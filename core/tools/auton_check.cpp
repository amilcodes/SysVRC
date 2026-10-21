// auton_check: how repeatable is an auton?
//
//     auton_check autons/worlds_mogo_rush.blue.auton
//     auton_check autons/skills.auton --budget 60 --runs 500 --html skills.html
//
// Runs the routine once with nothing disturbed (the nominal run), then N more
// times with a random draw of what changes between real runs: placement,
// battery, left/right mismatch, wheel slip, gyro drift, motor response. Each
// run records when the routine finished and where the robot really was when
// each mechanism call fired. From that:
//
//   - does it finish inside the auton period, and how often
//   - which mechanism calls land far from where they do in the nominal run
//     (a clamp that fires 4 in from the goal misses the goal)
//   - which disturbance is to blame
//
// The disturbance sizes are guesses, not measurements. They're all flags so a
// team can match their own robot; the defaults are in --help and the report.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "report_template.hpp"
#include "visbot/auton_file.hpp"
#include "visbot/controllers.hpp"
#include "visbot/plant.hpp"
#include "visbot/robot_spec.hpp"

using namespace visbot;

namespace {

// ------------------------------------------------------------------ options
struct Range {
    double lo = 0, hi = 0;
};

struct Disturbances {
    double placeXySigma = 0.5;    // in, 1-sigma per axis: hand placement with a line-up tool
    double placeDegSigma = 1.0;   // deg, 1-sigma
    Range battery{0.85, 1.0};     // achievable speed vs a full battery
    double mismatchSigma = 0.02;  // per-side speed gain, 1-sigma
    Range slip{0.0, 0.03};        // fraction of wheel travel lost
    double driftSigma = 0.02;     // gyro drift, deg/s, 1-sigma
    Range tau{0.10, 0.14};        // motor time constant, s
    double tauNominal = 0.12;     // used by the clean run and whenever tau isn't the factor
};

struct Options {
    std::string auton;
    int runs = 300;
    double budget = -1;           // s; -1 = 15, or 60 if the name mentions skills
    uint64_t seed = 1;
    Disturbances d;
    std::string jsonPath, htmlPath;
    std::vector<std::pair<std::string, double>> gainSets;
    bool sensitivity = true;
    bool hasFieldStart = false;
    Pose fieldStart;
    bool brief = false;
    std::string field;            // report's default field layer; empty = guess from the name
    std::string robotPath;        // robot.json; empty = the defaults in constants.hpp
    bool tauGiven = false;        // --tau: don't derive the range from the robot
    bool ideal = false;
};

[[noreturn]] void usage(int code) {
    std::printf(
        "usage: auton_check <file.auton | skills | square | chain> [options]\n"
        "\n"
        "  --runs N           disturbed runs (default 300)\n"
        "  --budget S         time limit in seconds (default 15, or 60 for skills)\n"
        "  --seed N           random seed (default 1); same seed, same report\n"
        "  --html FILE        write a self-contained HTML report\n"
        "  --json FILE        write the raw numbers\n"
        "  --set KEY=VALUE    override a gain, e.g. --set drive.kp=14 --set turn.kd=25\n"
        "                     (LemLib: lateral.kp, angular.kd, lateral.slew, drivetrain.horizontal_drift ...)\n"
        "  --no-sensitivity   skip the one-factor-at-a-time breakdown\n"
        "  --brief            one summary line (for checking a folder of autons)\n"
        "  --field NAME       field drawn in the report: high-stakes, high-stakes-skills,\n"
        "                     override, blank (default: from the routine's name)\n"
        "  --robot FILE       robot.json from tools/robot_studio: drivetrain, footprint,\n"
        "                     mechanisms (default: the drivetrain in constants.hpp)\n"
        "  --field-start X,Y,H  where the robot really starts, in field inches/degrees\n"
        "                     (origin at field centre, +y toward the far wall). Turns on\n"
        "                     the perimeter walls; needed for routines that push into a\n"
        "                     wall with drive_set. Overrides `# @field_start` in the file.\n"
        "\n"
        "disturbances (what changes between real runs; defaults in brackets):\n"
        "  --place-xy IN      start placement error, 1-sigma per axis [0.5]\n"
        "  --place-deg DEG    start heading error, 1-sigma [1.0]\n"
        "  --battery LO:HI    achievable speed vs full battery, uniform [0.85:1.0]\n"
        "  --mismatch F       left/right drivetrain gain, 1-sigma [0.02]\n"
        "  --slip LO:HI       wheel slip fraction, uniform [0:0.03]\n"
        "  --drift DEG/S      gyro drift, 1-sigma [0.02]\n"
        "  --tau LO:HI        motor time constant in s, uniform [0.10:0.14]\n"
        "  --ideal            turn every disturbance off (sanity check)\n");
    std::exit(code);
}

Range parseRange(const char* s) {
    Range r;
    const char* colon = std::strchr(s, ':');
    r.lo = std::atof(s);
    r.hi = colon ? std::atof(colon + 1) : r.lo;
    if (r.hi < r.lo) std::swap(r.lo, r.hi);
    return r;
}

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto val = [&]() -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", a.c_str()); usage(2); }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") usage(0);
        else if (a == "--runs") o.runs = std::max(1, std::atoi(val()));
        else if (a == "--budget") o.budget = std::atof(val());
        else if (a == "--seed") o.seed = std::strtoull(val(), nullptr, 10);
        else if (a == "--html") o.htmlPath = val();
        else if (a == "--json") o.jsonPath = val();
        else if (a == "--no-sensitivity") o.sensitivity = false;
        else if (a == "--brief") o.brief = true;
        else if (a == "--field") o.field = val();
        else if (a == "--robot") o.robotPath = val();
        else if (a == "--field-start") {
            const char* v = val();
            if (std::sscanf(v, "%lf,%lf,%lf", &o.fieldStart.x, &o.fieldStart.y, &o.fieldStart.theta) != 3) {
                std::fprintf(stderr, "--field-start wants X,Y,HEADING\n");
                usage(2);
            }
            o.hasFieldStart = true;
        }
        else if (a == "--place-xy") o.d.placeXySigma = std::atof(val());
        else if (a == "--place-deg") o.d.placeDegSigma = std::atof(val());
        else if (a == "--battery") o.d.battery = parseRange(val());
        else if (a == "--mismatch") o.d.mismatchSigma = std::atof(val());
        else if (a == "--slip") o.d.slip = parseRange(val());
        else if (a == "--drift") o.d.driftSigma = std::atof(val());
        else if (a == "--tau") { o.d.tau = parseRange(val()); o.tauGiven = true; }
        else if (a == "--ideal") { o.d = {0, 0, {1, 1}, 0, {0, 0}, 0, {0.12, 0.12}, 0.12}; o.ideal = true; }
        else if (a == "--set") {
            const std::string kv = val();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--set wants KEY=VALUE\n"); usage(2); }
            o.gainSets.emplace_back(kv.substr(0, eq), std::atof(kv.c_str() + eq + 1));
        } else if (!a.empty() && a[0] == '-') {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            usage(2);
        } else if (o.auton.empty()) {
            o.auton = a;
        } else {
            usage(2);
        }
    }
    if (o.auton.empty()) usage(2);
    return o;
}

bool applyGain(DriveGains& g, const std::string& key, double v) {
    std::map<std::string, PidGains*> pids = {
        {"drive", &g.drive}, {"heading", &g.heading}, {"turn", &g.turn},
        {"swing", &g.swing}, {"odom_angular", &g.odomAngular},
    };
    const size_t dot = key.find('.');
    if (dot != std::string::npos) {
        auto it = pids.find(key.substr(0, dot));
        const std::string f = key.substr(dot + 1);
        if (it != pids.end()) {
            if (f == "kp") { it->second->kP = v; return true; }
            if (f == "ki") { it->second->kI = v; return true; }
            if (f == "kd") { it->second->kD = v; return true; }
            if (f == "start_i") { it->second->startI = v; return true; }
        }
    }
    if (key == "odom_turn_bias") { g.odomTurnBias = v; return true; }
    if (key == "odom_look_ahead") { g.odomLookAheadIn = v; return true; }
    if (key == "slew_drive.distance") { g.slewDriveDistanceIn = v; return true; }
    if (key == "slew_drive.min_speed") { g.slewDriveMinSpeed = v; return true; }
    return false;
}

/// --set for a LemLib routine: ControllerSettings by name.
bool applyLemGain(LemConfig& c, const std::string& key, double v) {
    if (key == "drivetrain.horizontal_drift") { c.horizontalDrift = v; return true; }
    const size_t dot = key.find('.');
    if (dot == std::string::npos) return false;
    const std::string which = key.substr(0, dot), field = key.substr(dot + 1);
    if (which != "lateral" && which != "angular") return false;
    LemConfig::Controller& k = which == "lateral" ? c.lateral : c.angular;
    const std::pair<const char*, double*> fields[] = {
        {"kp", &k.kP}, {"ki", &k.kI}, {"kd", &k.kD}, {"windup", &k.windupRange},
        {"small_error", &k.smallError}, {"small_timeout", &k.smallErrorTimeout},
        {"large_error", &k.largeError}, {"large_timeout", &k.largeErrorTimeout}, {"slew", &k.slew}};
    for (const auto& [name, ptr] : fields) {
        if (field == name) { *ptr = v; return true; }
    }
    return false;
}

// ------------------------------------------------------------------ one run
struct Draw {
    Pose placement;          // offset added to the start pose (x, y in, theta deg)
    PlantParams plant;
};

struct ActionHit {
    bool fired = false;
    Pose at;                 // true pose when the call fired
    double t = 0;
};

struct Run {
    Draw draw;
    double motionEnd = 0;    // when the last motion finished
    double total = 0;        // when the whole routine finished (trailing delays too)
    bool finished = false;
    Pose end;                // true pose at motionEnd
    std::vector<ActionHit> actions;          // indexed like actionPcs
    std::vector<ExitReason> exits;           // per blocking instruction
    int pcAtBudget = -1;                     // what was running when time ran out
    std::vector<Pose> traj;                  // true pose every `sampleTicks` ticks
    std::vector<double> instrStart;          // when each instruction became current
};

struct Plan {
    MissionDef def;
    RobotParams physical;                    // the robot as built (plant)
    RobotParams declared;                    // what the code tells EZ (controller)
    double encoderScale = 1.0;
    std::vector<int> actionPcs;              // pc of every Action instruction
    std::vector<int> waitPcs;                // pc of every blocking instruction
    int lastMotionWaitPc = -1;               // after this, only delays/actions remain
};

// The *_set motions. They keep running on the brain after the call returns,
// until they settle or something replaces them.
bool isMotionSet(Instr::Op op) {
    return op == Instr::Op::DriveSet || op == Instr::Op::TurnSet || op == Instr::Op::SwingSet ||
           op == Instr::Op::OdomSet || op == Instr::Op::TurnToPoint || op == Instr::Op::TurnRelative;
}

// The motion gave up (stopped moving, or ran out of time) instead of settling.
bool gaveUp(ExitReason e) { return e == ExitReason::Velocity || e == ExitReason::Timeout; }

Plan makePlan(const MissionDef& def) {
    Plan p;
    p.def = def;
    const int n = static_cast<int>(def.mission.size());
    auto at = [&](int i) -> const Instr& { return def.mission[static_cast<size_t>(i)]; };
    if (def.lib == Library::LemLib) {
        // LemLib motions run beside the auton, so each motion call is where an
        // exit gets recorded, and the routine's motion ends when the last one does.
        auto pushes = [](Instr::Op op) { return op == Instr::Op::LemTank || op == Instr::Op::LemArcade; };
        for (int i = 0; i < n; ++i) {
            const Instr& in = at(i);
            if (in.op == Instr::Op::Action) p.actionPcs.push_back(i);
            if (in.lemMotion()) p.waitPcs.push_back(i);
            if (in.lemMotion() || pushes(in.op)) p.lastMotionWaitPc = i;
        }
        for (int i = p.lastMotionWaitPc + 1; i < n; ++i)
            if (pushes(at(i).op)) p.lastMotionWaitPc = i + 1;
        return p;
    }
    for (int i = 0; i < n; ++i) {
        const Instr& in = at(i);
        if (in.op == Instr::Op::Action) p.actionPcs.push_back(i);
        if (in.blocking() && in.op != Instr::Op::Delay) {
            p.waitPcs.push_back(i);
            p.lastMotionWaitPc = i;
        }
        if (in.op == Instr::Op::DriveRaw) p.lastMotionWaitPc = i;
    }
    // drive_set then a delay is a timed push, so that delay counts as motion.
    for (int i = p.lastMotionWaitPc + 1; i < n; ++i)
        if (at(i).op == Instr::Op::DriveRaw) p.lastMotionWaitPc = i + 1;
    // A motion with no wait after it still runs to the end on the brain, so
    // the routine's motion ends when the controller does.
    for (int i = p.lastMotionWaitPc + 1; i < n; ++i) {
        if (isMotionSet(at(i).op)) {
            p.lastMotionWaitPc = n;
            break;
        }
    }
    return p;
}

// sampleTicks: record the true pose every N control ticks (0 = don't).
Run simulate(const Plan& plan, const DriveGains& gains, const Draw& draw, double budget, int sampleTicks) {
    PlantParams pp = draw.plant;
    pp.walls = plan.def.hasFieldStart;
    pp.robotHalfIn = std::max(plan.physical.robotWidthIn, plan.physical.robotLengthIn) / 2;
    pp.encoderScale = plan.encoderScale;
    DiffDrivePlant plant(plan.physical, pp);
    const std::unique_ptr<AutonController> controller = makeController(plan.def, plan.declared, gains);
    AutonController& ctrl = *controller;
    // The robot is where the student put it; the code believes odom_xyt_set.
    const Pose& real = plan.def.hasFieldStart ? plan.def.fieldStart : plan.def.start;
    plant.reset({real.x + draw.placement.x, real.y + draw.placement.y, real.theta + draw.placement.theta});
    ctrl.resetPose(plan.def.start, plant.sensors());
    ctrl.setMission(plan.def.mission);

    Run r;
    r.draw = draw;
    r.instrStart.assign(plan.def.mission.size() + 1, -1.0);
    if (!plan.def.mission.empty()) r.instrStart[0] = 0.0;
    r.actions.resize(plan.actionPcs.size());
    r.exits.assign(plan.waitPcs.size(), ExitReason::Running);
    std::map<int, size_t> waitIndex;
    for (size_t i = 0; i < plan.waitPcs.size(); ++i) waitIndex[plan.waitPcs[i]] = i;

    const double dt = 1.0 / 120.0;
    const double limit = std::max(120.0, budget * 3);
    double t = 0;
    int pc = ctrl.status().pc;
    int seenActions = 0;
    int seenExits = 0;
    const bool lem = plan.def.lib == Library::LemLib;
    long tick = 0;
    bool motionDone = plan.lastMotionWaitPc < 0;
    if (sampleTicks > 0) r.traj.push_back(plant.truth());
    while (!ctrl.status().done && t < limit) {
        const WheelCmd c = ctrl.tick(plant.sensors(), dt);
        plant.step(c, dt);
        t += dt;
        const MotionStatus& st = ctrl.status();

        // The controller only counts actions, and several can fire in one
        // tick. They fire in plan order, so the count is the index.
        while (seenActions < st.actionCount) {
            ActionHit& h = r.actions[static_cast<size_t>(seenActions)];
            h.fired = true;
            h.at = plant.truth();
            h.t = t;
            ++seenActions;
        }
        if (lem && st.exitCount != seenExits) {
            // a LemLib motion finished; it says which one
            seenExits = st.exitCount;
            auto it = waitIndex.find(st.exitPc);
            if (it != waitIndex.end()) r.exits[it->second] = st.lastExit;
        }
        if (st.pc != pc) {
            // EZ: blocking instructions we left behind, record how they ended
            for (int k = pc; !lem && k < st.pc && k >= 0; ++k) {
                auto it = waitIndex.find(k);
                if (it != waitIndex.end()) r.exits[it->second] = st.lastExit;
            }
            // Everything we passed over this tick starts now.
            for (int k = std::max(pc + 1, 0); k <= st.pc && k < static_cast<int>(r.instrStart.size()); ++k) {
                double& start = r.instrStart[static_cast<size_t>(k)];
                if (start < 0) start = t;
            }
            pc = st.pc;
        }
        if (!motionDone && ((st.pc > plan.lastMotionWaitPc && !st.busy) || st.done)) {
            motionDone = true;
            r.motionEnd = t;
            r.end = plant.truth();
        }
        if (r.pcAtBudget < 0 && t >= budget) r.pcAtBudget = st.activePc >= 0 ? st.activePc : st.pc;
        if (sampleTicks > 0 && ++tick % sampleTicks == 0) r.traj.push_back(plant.truth());
    }
    r.total = t;
    r.finished = ctrl.status().done;
    if (!motionDone) { r.motionEnd = t; r.end = plant.truth(); }
    return r;
}

// ------------------------------------------------------------------ draws
struct Sampler {
    std::mt19937_64 rng;
    explicit Sampler(uint64_t seed) : rng(seed) {}
    double normal(double sigma) { return sigma > 0 ? std::normal_distribution<double>(0, sigma)(rng) : 0.0; }
    double uniform(Range r) { return r.hi > r.lo ? std::uniform_real_distribution<double>(r.lo, r.hi)(rng) : r.lo; }
};

enum Factor { kPlaceXy, kPlaceDeg, kBattery, kMismatch, kSlip, kDrift, kTau, kFactorCount };
const char* const kFactorNames[kFactorCount] = {"placement (x/y)", "placement (heading)", "battery", "L/R mismatch",
                                                "wheel slip", "gyro drift", "motor response"};

// onlyFactor >= 0 varies just that one and leaves the rest nominal.
Draw randomDraw(Sampler& s, const Disturbances& d, int onlyFactor = -1) {
    auto on = [&](int f) { return onlyFactor < 0 || onlyFactor == f; };
    Draw w;
    w.plant.seed = static_cast<uint32_t>(s.rng());
    if (on(kPlaceXy)) { w.placement.x = s.normal(d.placeXySigma); w.placement.y = s.normal(d.placeXySigma); }
    if (on(kPlaceDeg)) w.placement.theta = s.normal(d.placeDegSigma);
    w.plant.batteryScale = on(kBattery) ? s.uniform(d.battery) : 1.0;
    if (on(kMismatch)) {
        const double m = s.normal(d.mismatchSigma);
        w.plant.leftGain = 1.0 + m / 2;
        w.plant.rightGain = 1.0 - m / 2;
    }
    w.plant.slipFraction = on(kSlip) ? s.uniform(d.slip) : 0.0;
    w.plant.imuDriftDegPerS = on(kDrift) ? s.normal(d.driftSigma) : 0.0;
    w.plant.motorTauSec = on(kTau) ? s.uniform(d.tau) : d.tauNominal;
    return w;
}

// The report animates the clean run at 20 Hz and draws up to kGhostRuns of
// the varied runs as "ghosts" at 10 Hz (control ticks are 120 Hz).
constexpr int kNominalSampleTicks = 6;
constexpr int kGhostSampleTicks = 12;
constexpr int kGhostRuns = 60;

Draw nominalDraw(const Disturbances& d) {
    Draw w;
    w.plant.imuNoiseStdDeg = 0.0;
    w.plant.motorTauSec = d.tauNominal;
    return w;
}

// ------------------------------------------------------------------ helpers
double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const double idx = p * (static_cast<double>(v.size()) - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = std::min(v.size() - 1, lo + 1);
    return v[lo] + (v[hi] - v[lo]) * (idx - static_cast<double>(lo));
}

double dist(const Pose& a, const Pose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else if (c == '<') o += "\\u003c";   // safe to inline in <script>
        else o += c;
    }
    return o;
}

std::string num(double v, int prec = 3) {
    if (!std::isfinite(v)) return "null";
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.*f", prec, v);
    return buf;
}

bool isSkills(const std::string& name) { return name.find("skills") != std::string::npos; }

// Routine names end in .red / .blue when they're alliance specific.
std::string allianceOf(const std::string& name) {
    if (name.find(".red") != std::string::npos) return "red";
    if (name.find(".blue") != std::string::npos) return "blue";
    return "";
}

std::string atLine(const Instr& in) {
    return in.sourceLine ? " at line " + std::to_string(in.sourceLine) : std::string();
}

// How an instruction shows up in the terminal: `intake.move(127)  (line 1214)`
std::string describe(const Instr& in) {
    std::string s = in.op == Instr::Op::Action ? in.label : in.name();
    if (in.sourceLine > 0) s += "  (line " + std::to_string(in.sourceLine) + ")";
    return s;
}

const char* kindOf(const Instr& in) {
    switch (in.op) {
        case Instr::Op::Action: return "action";
        case Instr::Op::Delay: return "delay";
        case Instr::Op::Wait: case Instr::Op::WaitUntil: case Instr::Op::WaitQuick:
        case Instr::Op::WaitQuickChain: return "wait";
        case Instr::Op::DriveSet: case Instr::Op::TurnSet: case Instr::Op::TurnToPoint:
        case Instr::Op::TurnRelative: case Instr::Op::SwingSet: case Instr::Op::OdomSet:
        case Instr::Op::DriveRaw: return "motion";
        case Instr::Op::LemWaitUntil: case Instr::Op::LemWaitUntilDone: return "wait";
        case Instr::Op::LemTank: case Instr::Op::LemArcade: return "motion";
        default: return in.lemMotion() ? "motion" : "set";
    }
}

// ------------------------------------------------------------------ setup
std::optional<MissionDef> loadRoutine(const std::string& name) {
    if (missions::isBuiltin(name)) return missions::byName(name);
    const AutonParseResult r = loadAutonFile(name);
    if (r.ok()) return r.def;
    for (const auto& e : r.errors) std::fprintf(stderr, "%s: %s\n", name.c_str(), e.c_str());
    return std::nullopt;
}

// odom_xyt_set(0, 0, h) says nothing about where the robot is, but a nonzero
// start inside the field means the team wrote field coordinates (the sim
// spawns there too). Any of the field's symmetric frames puts the walls in the
// same place, so that's enough to simulate wall pushes.
void inferFieldStart(MissionDef& def) {
    if (def.hasFieldStart) return;
    const Pose s = def.start;
    if ((s.x != 0.0 || s.y != 0.0) && std::fabs(s.x) < 72.0 && std::fabs(s.y) < 72.0) {
        def.hasFieldStart = true;
        def.fieldStart = s;
    }
}

// Timed pushes (drive_set, tank) only mean something if there's a wall to push into.
std::vector<std::string> pushWarnings(const MissionDef& def) {
    if (def.hasFieldStart) return {};
    std::string lines;
    for (const Instr& in : def.mission) {
        if (in.op != Instr::Op::DriveRaw && in.op != Instr::Op::LemTank && in.op != Instr::Op::LemArcade) continue;
        if (!lines.empty()) lines += ", ";
        lines += in.sourceLine ? std::to_string(in.sourceLine) : "?";
    }
    if (lines.empty()) return {};
    const char* how = def.lib == Library::LemLib ? "tank()/arcade()" : "drive_set";
    return {std::string("This routine pushes with ") + how + " (line " + lines + ") but the sim doesn't know where "
            "the walls are, so the push drives through open field. Everything after it is not meaningful. Pass "
            "--field-start X,Y,HEADING (or add # @field_start to the file)."};
}

// LemLib's distTraveled starts at 0, not -1, so waiting before the first
// motion never returns on the robot either.
std::vector<std::string> lemWarnings(const MissionDef& def) {
    if (def.lib != Library::LemLib) return {};
    for (const Instr& in : def.mission) {
        if (in.lemMotion()) break;
        if (in.op == Instr::Op::LemWaitUntil || in.op == Instr::Op::LemWaitUntilDone)
            return {std::string(in.name()) + "() at line " + std::to_string(in.sourceLine) +
                    " comes before any motion. LemLib's distTraveled starts at 0, so it waits forever "
                    "(the rest of the routine never runs)."};
    }
    return {};
}

// Also narrows the motor response range to this robot's. False if robot.json is bad.
bool loadRobot(Options& o, RobotSpec& robot) {
    const RobotSpecResult rr = loadRobotSpec(o.robotPath);
    if (!rr.ok()) {
        for (const auto& e : rr.errors) std::fprintf(stderr, "%s: %s\n", o.robotPath.c_str(), e.c_str());
        return false;
    }
    robot = rr.spec;
    const double t = robot.motorTauSec;
    o.d.tauNominal = t;
    if (o.ideal) o.d.tau = {t, t};
    else if (!o.tauGiven) o.d.tau = {t * 0.83, t * 1.17};  // same spread as the 0.10-0.14 default
    return true;
}

// ------------------------------------------------------------------ results
struct ActionStat {
    int pc;
    double p50, p95, max;    // distance from where the nominal run fired it
    int fired;
    double nomT;
    Pose nom;
};

// One factor varied at a time, everything else nominal.
struct Sens {
    int factor;
    double endP95;
    double timeP95;
    double onTimePct;
};

// Everything one check works out. The printers and the JSON writer only read it.
struct Check {
    Options opt;
    Plan plan;
    RobotSpec robot;
    bool haveRobot = false;
    DriveGains gains;
    std::vector<std::string> warnings;

    Run nominal;
    std::vector<Run> runs;
    std::vector<double> motionEnd, endErr, endHeadErr;   // per run
    int onTime = 0, interfered = 0, unfinished = 0;
    std::vector<ActionStat> actionStats;
    std::vector<int> stalls;                             // per wait: runs where it gave up
    std::vector<Sens> byEnd, byTime;                     // empty with --no-sensitivity
    int overPc = -1, overN = 0;                          // what was usually running at the buzzer
    std::string overWhere;

    const MissionDef& def() const { return plan.def; }
    double onTimePct() const { return 100.0 * onTime / static_cast<double>(runs.size()); }
};

// Two questions, because they have different answers: what moves the robot
// (end point), and what makes it late (finish time). A flat battery can barely
// move the end point and still be why a routine misses the buzzer.
std::vector<Sens> sensitivity(const Check& c) {
    const Options& o = c.opt;
    std::vector<Sens> sens;
    const int per = std::max(30, o.runs / 4);
    for (int f = 0; f < kFactorCount; ++f) {
        Sampler s(o.seed * 7919 + static_cast<uint64_t>(f));
        std::vector<double> e, tt;
        int ok = 0;
        for (int i = 0; i < per; ++i) {
            Draw w = randomDraw(s, o.d, f);
            w.plant.imuNoiseStdDeg = 0.0;  // otherwise gyro noise shows up in every factor
            const Run r = simulate(c.plan, c.gains, w, o.budget, 0);
            e.push_back(dist(r.end, c.nominal.end));
            tt.push_back(r.motionEnd);
            if (r.motionEnd <= o.budget) ++ok;
        }
        sens.push_back({f, percentile(e, 0.95), percentile(tt, 0.95), 100.0 * ok / per});
    }
    return sens;
}

void simulateAll(Check& c) {
    const Options& o = c.opt;
    // Nothing disturbed. Everything else is measured against this run.
    c.nominal = simulate(c.plan, c.gains, nominalDraw(o.d), o.budget, kNominalSampleTicks);

    Sampler sampler(o.seed);
    c.runs.reserve(static_cast<size_t>(o.runs));
    for (int i = 0; i < o.runs; ++i)
        c.runs.push_back(simulate(c.plan, c.gains, randomDraw(sampler, o.d), o.budget,
                                  i < kGhostRuns ? kGhostSampleTicks : 0));

    if (o.sensitivity) {
        c.byEnd = c.byTime = sensitivity(c);
        std::sort(c.byEnd.begin(), c.byEnd.end(), [](auto& a, auto& b) { return a.endP95 > b.endP95; });
        std::sort(c.byTime.begin(), c.byTime.end(), [](auto& a, auto& b) { return a.timeP95 > b.timeP95; });
    }
}

// For runs that went over: what was running when the clock hit the limit?
void findLateInstr(Check& c) {
    const MissionDef& def = c.def();
    std::map<int, int> overAt;
    for (const Run& r : c.runs)
        if (r.motionEnd > c.opt.budget && r.pcAtBudget >= 0) ++overAt[r.pcAtBudget];
    for (const auto& [pc, n] : overAt) {
        if (n > c.overN) { c.overPc = pc; c.overN = n; }
    }

    if (c.overPc >= static_cast<int>(def.mission.size())) {
        // Past the last instruction: still finishing a motion nothing waits for.
        for (auto it = def.mission.rbegin(); it != def.mission.rend(); ++it) {
            if (!isMotionSet(it->op)) continue;
            c.overWhere = std::string("the last motion, ") + it->name() + atLine(*it) + " (nothing waits for it)";
            break;
        }
    } else if (c.overPc >= 0) {
        const Instr& in = def.mission[static_cast<size_t>(c.overPc)];
        c.overWhere = in.op == Instr::Op::Action ? in.label : in.name();
        if (in.op == Instr::Op::Delay) c.overWhere += " " + std::to_string(static_cast<int>(in.a)) + " ms";
        c.overWhere += atLine(in);
    }
}

void summarize(Check& c) {
    const Run& nominal = c.nominal;
    for (const Run& r : c.runs) {
        c.motionEnd.push_back(r.motionEnd);
        c.endErr.push_back(dist(r.end, nominal.end));
        c.endHeadErr.push_back(std::fabs(r.end.theta - nominal.end.theta));
        if (r.motionEnd <= c.opt.budget) ++c.onTime;
        if (!r.finished) ++c.unfinished;
        if (std::any_of(r.exits.begin(), r.exits.end(), gaveUp)) ++c.interfered;
    }

    const std::vector<int>& actionPcs = c.plan.actionPcs;
    for (size_t a = 0; a < actionPcs.size(); ++a) {
        std::vector<double> d;
        int fired = 0;
        for (const Run& r : c.runs) {
            if (!r.actions[a].fired || !nominal.actions[a].fired) continue;
            ++fired;
            d.push_back(dist(r.actions[a].at, nominal.actions[a].at));
        }
        const double worst = d.empty() ? 0 : *std::max_element(d.begin(), d.end());
        c.actionStats.push_back({actionPcs[a], percentile(d, 0.5), percentile(d, 0.95), worst, fired,
                                 nominal.actions[a].t, nominal.actions[a].at});
    }

    c.stalls.assign(c.plan.waitPcs.size(), 0);
    for (size_t w = 0; w < c.stalls.size(); ++w)
        for (const Run& r : c.runs)
            if (gaveUp(r.exits[w])) ++c.stalls[w];

    findLateInstr(c);
}

// ------------------------------------------------------------------ terminal
// One line per routine, for running over a whole folder.
void printBrief(const Check& c) {
    const ActionStat* worst = nullptr;
    for (const auto& a : c.actionStats)
        if (!worst || a.p95 > worst->p95) worst = &a;
    std::string worstS = "-";
    if (worst) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%.1f in @%d", worst->p95,
                      c.def().mission[static_cast<size_t>(worst->pc)].sourceLine);
        worstS = buf;
    }
    std::string note;
    if (!c.warnings.empty()) note = "needs --field-start";
    else if (!c.overWhere.empty()) note = "over at " + c.overWhere;

    std::printf("%-30s %6.2f s %6.2f s %6.1f%% %7.1f in  %-16s %-20s %s\n", c.def().name.c_str(),
                c.nominal.motionEnd, percentile(c.motionEnd, 0.95), c.onTimePct(), percentile(c.endErr, 0.95),
                worstS.c_str(), c.byEnd.empty() ? "-" : kFactorNames[c.byEnd.front().factor], note.c_str());
}

void printReport(const Check& c) {
    const Options& o = c.opt;
    const MissionDef& def = c.def();
    const Plan& plan = c.plan;
    const Run& nominal = c.nominal;

    std::printf("%s: %zu instructions, %zu mechanism calls, %d runs\n", def.name.c_str(), def.mission.size(),
                plan.actionPcs.size(), o.runs);
    if (c.haveRobot) {
        const RobotParams& rp = plan.physical;
        std::printf("robot   %s: %.2f in wheels @ %.0f rpm (%.0f in/s), track %.1f in, %.1fx%.1f in, response %.2f s\n",
                    c.robot.name.c_str(), rp.wheelDiameterIn, rp.wheelRpm, rp.maxWheelSpeedInPerSec(),
                    rp.trackWidthIn, rp.robotWidthIn, rp.robotLengthIn, c.robot.motorTauSec);
        if (std::fabs(plan.encoderScale - 1.0) > 0.005)
            std::printf("        the code tells %s %.2f in @ %.0f rpm, so odometry reads %.1f%% %s than the robot moves\n",
                        def.lib == Library::LemLib ? "LemLib" : "EZ", plan.declared.wheelDiameterIn, plan.declared.wheelRpm, std::fabs(plan.encoderScale - 1.0) * 100,
                        plan.encoderScale < 1 ? "shorter" : "longer");
    }
    std::printf("\n");
    std::printf("time    nominal %.2f s   p50 %.2f s   p95 %.2f s   worst %.2f s   (limit %.0f s)\n",
                nominal.motionEnd, percentile(c.motionEnd, 0.5), percentile(c.motionEnd, 0.95),
                *std::max_element(c.motionEnd.begin(), c.motionEnd.end()), o.budget);
    std::printf("        finishes in time in %.1f%% of runs\n", c.onTimePct());
    if (!c.overWhere.empty())
        std::printf("        when time ran out it was usually on: %s (%d runs)\n", c.overWhere.c_str(), c.overN);
    if (nominal.total - nominal.motionEnd > 0.5)
        std::printf("        (then %.1f s of trailing delay the field clock will cut off)\n",
                    nominal.total - nominal.motionEnd);
    std::printf("end     p50 %.2f in   p95 %.2f in from nominal,  heading p95 %.1f deg\n", percentile(c.endErr, 0.5),
                percentile(c.endErr, 0.95), percentile(c.endHeadErr, 0.95));
    if (c.interfered) std::printf("        %d runs had a motion end on a velocity/timeout exit\n", c.interfered);
    if (c.unfinished) std::printf("        %d runs never finished\n", c.unfinished);

    if (!c.actionStats.empty()) {
        auto sorted = c.actionStats;
        std::sort(sorted.begin(), sorted.end(), [](auto& a, auto& b) { return a.p95 > b.p95; });
        std::printf("\nleast repeatable mechanism calls (distance from the nominal run when it fired):\n");
        for (size_t i = 0; i < std::min<size_t>(5, sorted.size()); ++i) {
            const Instr& in = def.mission[static_cast<size_t>(sorted[i].pc)];
            std::printf("  p95 %5.2f in   at %5.2f s   %s\n", sorted[i].p95, sorted[i].nomT, describe(in).c_str());
        }
    }
    if (!c.byEnd.empty()) {
        std::printf("\nwhat moves the end point most (p95, one factor at a time):\n");
        for (const auto& x : c.byEnd) std::printf("  %-20s %5.2f in\n", kFactorNames[x.factor], x.endP95);
        std::printf("\nwhat makes it late (p95 finish time, one factor at a time; nominal %.2f s):\n",
                    nominal.motionEnd);
        for (const auto& x : c.byTime)
            std::printf("  %-20s %+5.2f s   in time %5.1f%%\n", kFactorNames[x.factor],
                        x.timeP95 - nominal.motionEnd, x.onTimePct);
    }
    if (std::any_of(c.stalls.begin(), c.stalls.end(), [](int n) { return n > 0; })) {
        std::printf("\nmotions that sometimes exit on velocity/timeout instead of reaching target:\n");
        for (size_t w = 0; w < c.stalls.size(); ++w) {
            if (!c.stalls[w]) continue;
            const Instr& in = def.mission[static_cast<size_t>(plan.waitPcs[w])];
            std::printf("  %4.1f%%  %s\n", 100.0 * c.stalls[w] / c.runs.size(), describe(in).c_str());
        }
    }
    for (const auto& w : c.warnings) std::printf("\nWARNING: %s\n", w.c_str());
    std::printf("\ndisturbances: placement %.2f in / %.2f deg (1-sigma), battery %.2f-%.2f, mismatch %.3f,\n"
                "              slip %.3f-%.3f, gyro drift %.3f deg/s, motor tau %.2f-%.2f s\n",
                o.d.placeXySigma, o.d.placeDegSigma, o.d.battery.lo, o.d.battery.hi, o.d.mismatchSigma,
                o.d.slip.lo, o.d.slip.hi, o.d.driftSigma, o.d.tau.lo, o.d.tau.hi);
}

// ------------------------------------------------------------------ json
void pose3(std::ostringstream& j, const Pose& p, int prec) {
    j << "[" << num(p.x, prec) << "," << num(p.y, prec) << "," << num(wrapDeg(p.theta), 1) << "]";
}

// The instruction the report flags as still running at the buzzer.
int lateInstr(const Check& c) {
    const Mission& m = c.def().mission;
    if (c.overPc < static_cast<int>(m.size())) return c.overPc;
    for (int i = static_cast<int>(m.size()) - 1; i >= 0; --i)
        if (std::strcmp(kindOf(m[static_cast<size_t>(i)]), "motion") == 0) return i;
    return -1;
}

// Shaped for the report's field view: everything is a pose sample, an
// instruction row, or a cloud of points, in the routine's own frame (or
// field coordinates when a field start was given).
std::string reportJson(const Check& c) {
    const Options& o = c.opt;
    const MissionDef& def = c.def();
    const Plan& plan = c.plan;
    const Run& nominal = c.nominal;
    const std::vector<Run>& runs = c.runs;
    const RobotParams& rp = plan.physical;
    const std::string alliance = allianceOf(def.name);
    std::string field = o.field;
    if (field.empty()) field = isSkills(def.name) ? "high-stakes-skills" : "high-stakes";

    std::ostringstream j;
    j << "{\"v\":2,\"name\":\"" << jsonEscape(def.name) << "\",\"source\":\"" << jsonEscape(o.auton) << "\""
      << ",\"runs\":" << o.runs << ",\"seed\":" << o.seed << ",\"budget\":" << num(o.budget, 1)
      << ",\"field\":\"" << field << "\",\"alliance\":" << (alliance.empty() ? "null" : "\"" + alliance + "\"")
      << ",\"robot\":[" << num(rp.robotWidthIn, 1) << "," << num(rp.robotLengthIn, 1) << "]"
      << ",\"spec\":" << (c.haveRobot ? c.robot.json : std::string("null"))
      << (def.lib == Library::LemLib ? ",\"lib\":\"lemlib\"" : "")
      << ",\"start\":";
    pose3(j, def.start, 2);
    j << ",\"field_start\":";
    if (def.hasFieldStart) pose3(j, def.fieldStart, 2); else j << "null";
    j << ",\"warnings\":[";
    for (size_t i = 0; i < c.warnings.size(); ++i) j << (i ? "," : "") << "\"" << jsonEscape(c.warnings[i]) << "\"";
    j << "],\"stats\":{\"on_time\":" << num(c.onTimePct(), 1) << ",\"p50\":" << num(percentile(c.motionEnd, 0.5), 2)
      << ",\"p95\":" << num(percentile(c.motionEnd, 0.95), 2)
      << ",\"worst\":" << num(*std::max_element(c.motionEnd.begin(), c.motionEnd.end()), 2)
      << ",\"nominal\":" << num(nominal.motionEnd, 2) << ",\"total\":" << num(nominal.total, 2)
      << ",\"end_p95\":" << num(percentile(c.endErr, 0.95), 2)
      << ",\"heading_p95\":" << num(percentile(c.endHeadErr, 0.95), 1)
      << ",\"late_i\":" << lateInstr(c) << ",\"late_runs\":" << c.overN << "}";

    j << ",\"finish\":[";
    for (size_t i = 0; i < c.motionEnd.size(); ++i) j << (i ? "," : "") << num(c.motionEnd[i], 2);
    j << "],\"dt\":" << num(kNominalSampleTicks / 120.0, 4) << ",\"path\":[";
    for (size_t i = 0; i < nominal.traj.size(); ++i) { if (i) j << ","; pose3(j, nominal.traj[i], 2); }
    // Only the first kGhostRuns runs kept a trajectory.
    j << "],\"gdt\":" << num(kGhostSampleTicks / 120.0, 4) << ",\"ghosts\":[";
    for (size_t i = 0, n = 0; i < runs.size(); ++i) {
        if (runs[i].traj.empty()) continue;
        j << (n++ ? "," : "") << "[";
        for (size_t k = 0; k < runs[i].traj.size(); ++k) { if (k) j << ","; pose3(j, runs[i].traj[k], 1); }
        j << "]";
    }
    // When each ghost fired each mechanism call (-1 = never), for the report's
    // game element sim.
    j << "],\"gact\":[";
    for (size_t i = 0, n = 0; i < runs.size(); ++i) {
        if (runs[i].traj.empty()) continue;
        j << (n++ ? "," : "") << "[";
        for (size_t a = 0; a < runs[i].actions.size(); ++a)
            j << (a ? "," : "") << (runs[i].actions[a].fired ? num(runs[i].actions[a].t, 2) : std::string("-1"));
        j << "]";
    }
    j << "],\"instrs\":[";
    std::map<int, size_t> waitIdx, actIdx;
    for (size_t w = 0; w < plan.waitPcs.size(); ++w) waitIdx[plan.waitPcs[w]] = w;
    for (size_t a = 0; a < plan.actionPcs.size(); ++a) actIdx[plan.actionPcs[a]] = a;
    for (size_t i = 0; i < def.mission.size(); ++i) {
        const Instr& in = def.mission[i];
        j << (i ? "," : "") << "{\"line\":" << in.sourceLine << ",\"text\":\"" << jsonEscape(toAutonLine(in))
          << "\",\"kind\":\"" << kindOf(in) << "\",\"t\":" << num(nominal.instrStart[i], 2);
        auto w = waitIdx.find(static_cast<int>(i));
        if (w != waitIdx.end()) {
            j << ",\"exit\":\"" << toString(nominal.exits[w->second]) << "\",\"stall\":"
              << num(100.0 * c.stalls[w->second] / runs.size(), 1);
        }
        auto a = actIdx.find(static_cast<int>(i));
        if (a != actIdx.end()) j << ",\"action\":" << a->second;
        j << "}";
    }
    j << "],\"actions\":[";
    for (size_t a = 0; a < c.actionStats.size(); ++a) {
        const auto& st = c.actionStats[a];
        j << (a ? "," : "") << "{\"i\":" << st.pc << ",\"t\":" << num(st.nomT, 2) << ",\"at\":[" << num(st.nom.x, 2)
          << "," << num(st.nom.y, 2) << "],\"p95\":" << num(st.p95, 2) << ",\"pts\":[";
        // 300 points is plenty to see the spread and keeps the page small
        for (size_t i = 0, n = 0; i < runs.size() && n < 300; ++i) {
            const ActionHit& h = runs[i].actions[a];
            if (!h.fired) continue;
            j << (n++ ? "," : "") << "[" << num(h.at.x, 1) << "," << num(h.at.y, 1) << "]";
        }
        j << "]}";
    }
    j << "],\"why\":{\"spread\":[";
    for (size_t i = 0; i < c.byEnd.size(); ++i)
        j << (i ? "," : "") << "[\"" << kFactorNames[c.byEnd[i].factor] << "\"," << num(c.byEnd[i].endP95, 2) << "]";
    j << "],\"late\":[";
    for (size_t i = 0; i < c.byTime.size(); ++i)
        j << (i ? "," : "") << "[\"" << kFactorNames[c.byTime[i].factor] << "\","
          << num(c.byTime[i].timeP95 - nominal.motionEnd, 2) << "," << num(c.byTime[i].onTimePct, 1) << "]";
    j << "]},\"disturb\":{\"place_xy\":" << num(o.d.placeXySigma) << ",\"place_deg\":" << num(o.d.placeDegSigma)
      << ",\"battery\":[" << num(o.d.battery.lo) << "," << num(o.d.battery.hi) << "],\"mismatch\":"
      << num(o.d.mismatchSigma) << ",\"slip\":[" << num(o.d.slip.lo) << "," << num(o.d.slip.hi) << "],\"drift\":"
      << num(o.d.driftSigma) << ",\"tau\":[" << num(o.d.tau.lo) << "," << num(o.d.tau.hi) << "]}}";
    return j.str();
}

bool writeOutputs(const Check& c) {
    const Options& o = c.opt;
    if (o.jsonPath.empty() && o.htmlPath.empty()) return true;
    const std::string json = reportJson(c);
    if (!o.jsonPath.empty()) {
        std::ofstream(o.jsonPath) << json << "\n";
        std::printf("\nwrote %s\n", o.jsonPath.c_str());
    }
    if (!o.htmlPath.empty()) {
        std::string html = reportTemplate();
        const std::string marker = "/*REPORT_DATA*/null";
        const size_t at = html.find(marker);
        if (at == std::string::npos) {
            std::fprintf(stderr, "report template is missing its data marker\n");
            return false;
        }
        html.replace(at, marker.size(), json);
        std::ofstream(o.htmlPath) << html;
        std::printf("%swrote %s\n", o.jsonPath.empty() ? "\n" : "", o.htmlPath.c_str());
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Check c;
    c.opt = parseArgs(argc, argv);
    Options& o = c.opt;

    std::optional<MissionDef> def = loadRoutine(o.auton);
    if (!def) return 1;
    if (o.budget < 0) o.budget = isSkills(def->name) ? 60.0 : 15.0;
    if (o.hasFieldStart) {
        def->hasFieldStart = true;
        def->fieldStart = o.fieldStart;
    }
    inferFieldStart(*def);
    c.warnings = pushWarnings(*def);
    for (std::string& w : lemWarnings(*def)) c.warnings.push_back(std::move(w));
    const bool lem = def->lib == Library::LemLib;

    c.haveRobot = !o.robotPath.empty();
    if (c.haveRobot && !loadRobot(o, c.robot)) return 2;
    for (const auto& [k, v] : o.gainSets) {
        if (!(lem ? applyLemGain(def->lem, k, v) : applyGain(c.gains, k, v))) {
            std::fprintf(stderr, "unknown gain '%s'\n", k.c_str());
            return 2;
        }
    }

    c.plan = makePlan(*def);
    if (lem) {
        // the drivetrain is what lemlib::Drivetrain says, unless robot.json knows better
        c.plan.declared = lemDrivetrain(def->lem, c.haveRobot ? c.robot.declared : RobotParams{});
        c.plan.physical = c.haveRobot ? c.robot.physical : c.plan.declared;
        if (c.haveRobot) c.plan.encoderScale = c.robot.encoderScale * (def->lem.rpm / c.robot.declared.wheelRpm);
    } else if (c.haveRobot) {
        c.plan.physical = c.robot.physical;
        c.plan.declared = c.robot.declared;
        c.plan.encoderScale = c.robot.encoderScale;
    }

    simulateAll(c);
    summarize(c);
    if (o.brief) {
        printBrief(c);
        return 0;
    }
    printReport(c);
    return writeOutputs(c) ? 0 : 1;
}
