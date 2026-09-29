// auton_check — how repeatable is an auton?
//
//     auton_check autons/worlds_mogo_rush.blue.auton
//     auton_check autons/skills.auton --budget 60 --runs 500 --html skills.html
//
// Runs the routine once with nothing disturbed (the "nominal" run), then N
// more times with a random draw of everything that changes between real runs:
// where the robot was placed, battery charge, left/right drivetrain mismatch,
// wheel slip, gyro drift, motor response. For each run it records when the
// routine finished and where the robot actually was when each mechanism call
// fired. The report answers:
//
//   * does it finish inside the auton period, and how often?
//   * which mechanism calls land far from where they land in the nominal run?
//     (a clamp that fires 4 in from the goal misses the goal)
//   * which disturbance is responsible? (placement vs battery vs slip ...)
//
// The disturbance sizes are assumptions, not measurements — they're flags so
// a team can set them to match their robot, and the defaults are listed in
// --help and in the report.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "report_template.hpp"
#include "visbot/auton_file.hpp"
#include "visbot/plant.hpp"

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
        "  --no-sensitivity   skip the one-factor-at-a-time breakdown\n"
        "  --brief            one summary line (for checking a folder of autons)\n"
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
        else if (a == "--tau") o.d.tau = parseRange(val());
        else if (a == "--ideal") o.d = {0, 0, {1, 1}, 0, {0, 0}, 0, {0.12, 0.12}};
        else if (a == "--set") {
            const std::string kv = val();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--set wants KEY=VALUE\n"); usage(2); }
            o.gainSets.push_back({kv.substr(0, eq), std::atof(kv.c_str() + eq + 1)});
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
    std::vector<Pose> path;                  // subsampled true pose
};

struct Plan {
    MissionDef def;
    std::vector<int> actionPcs;              // pc of every Action instruction
    std::vector<int> waitPcs;                // pc of every blocking instruction
    int lastMotionWaitPc = -1;               // after this, only delays/actions remain
};

Plan makePlan(const MissionDef& def) {
    Plan p{def, {}, {}, -1};
    for (int i = 0; i < static_cast<int>(def.mission.size()); ++i) {
        const Instr& in = def.mission[i];
        if (in.op == Instr::Op::Action) p.actionPcs.push_back(i);
        if (in.blocking() && in.op != Instr::Op::Delay) p.waitPcs.push_back(i);
        if (in.blocking() && in.op != Instr::Op::Delay) p.lastMotionWaitPc = i;
        if (in.op == Instr::Op::DriveRaw) p.lastMotionWaitPc = std::max(p.lastMotionWaitPc, i);
    }
    // A drive_set followed by a delay is a timed push: count that delay as motion.
    for (int i = p.lastMotionWaitPc + 1; i < static_cast<int>(def.mission.size()); ++i)
        if (def.mission[i].op == Instr::Op::DriveRaw) p.lastMotionWaitPc = i + 1;
    // A motion set after the last wait (no wait of its own) still runs to the
    // end on the brain, so the routine's motion ends when the controller does.
    for (int i = p.lastMotionWaitPc + 1; i < static_cast<int>(def.mission.size()); ++i) {
        const Instr::Op op = def.mission[static_cast<size_t>(i)].op;
        if (op == Instr::Op::DriveSet || op == Instr::Op::TurnSet || op == Instr::Op::SwingSet ||
            op == Instr::Op::OdomSet || op == Instr::Op::TurnToPoint || op == Instr::Op::TurnRelative)
            p.lastMotionWaitPc = static_cast<int>(def.mission.size());  // i.e. when done
    }
    return p;
}

Run simulate(const Plan& plan, const DriveGains& gains, const Draw& draw, double budget, bool keepPath) {
    RobotParams rp;
    PlantParams pp = draw.plant;
    pp.walls = plan.def.hasFieldStart;
    pp.robotHalfIn = rp.robotWidthIn / 2;
    DiffDrivePlant plant(rp, pp);
    MotionController ctrl(rp, gains);
    // The robot is where the student put it; the code believes odom_xyt_set.
    const Pose& real = plan.def.hasFieldStart ? plan.def.fieldStart : plan.def.start;
    plant.reset({real.x + draw.placement.x, real.y + draw.placement.y, real.theta + draw.placement.theta});
    ctrl.resetPose(plan.def.start, plant.sensors());
    ctrl.setMission(plan.def.mission);

    Run r;
    r.draw = draw;
    r.actions.resize(plan.actionPcs.size());
    r.exits.assign(plan.waitPcs.size(), ExitReason::Running);
    std::map<int, size_t> actionIndex, waitIndex;
    for (size_t i = 0; i < plan.actionPcs.size(); ++i) actionIndex[plan.actionPcs[i]] = i;
    for (size_t i = 0; i < plan.waitPcs.size(); ++i) waitIndex[plan.waitPcs[i]] = i;

    const double dt = 1.0 / 120.0;
    const double limit = std::max(120.0, budget * 3);
    double t = 0;
    int pc = ctrl.status().pc;
    int seenActions = 0;
    bool motionDone = plan.lastMotionWaitPc < 0;
    while (!ctrl.status().done && t < limit) {
        const WheelCmd c = ctrl.tick(plant.sensors(), dt);
        plant.step(c, dt);
        t += dt;
        const MotionStatus& st = ctrl.status();

        // Every action fired this tick (several can fire back to back).
        while (seenActions < st.actionCount) {
            // The controller only reports the latest; walk the plan in order.
            const int apc = plan.actionPcs[static_cast<size_t>(seenActions)];
            ActionHit& h = r.actions[actionIndex[apc]];
            h.fired = true;
            h.at = plant.truth();
            h.t = t;
            ++seenActions;
        }
        if (st.pc != pc) {
            // Blocking instructions we left behind: record how they ended.
            for (int k = pc; k < st.pc && k >= 0; ++k) {
                auto it = waitIndex.find(k);
                if (it != waitIndex.end()) r.exits[it->second] = st.lastExit;
            }
            pc = st.pc;
        }
        if (!motionDone && (st.pc > plan.lastMotionWaitPc || st.done)) {
            motionDone = true;
            r.motionEnd = t;
            r.end = plant.truth();
        }
        if (r.pcAtBudget < 0 && t >= budget) r.pcAtBudget = st.pc;
        if (keepPath && (r.path.empty() || static_cast<int>(t * 120) % 6 == 0)) r.path.push_back(plant.truth());
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
const char* kFactorNames[kFactorCount] = {"placement (x/y)", "placement (heading)", "battery", "L/R mismatch",
                                          "wheel slip", "gyro drift", "motor response"};

Draw draw(Sampler& s, const Disturbances& d, int onlyFactor = -1) {
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
    w.plant.motorTauSec = on(kTau) ? s.uniform(d.tau) : 0.12;
    return w;
}

Draw nominalDraw() {
    Draw w;
    w.plant.imuNoiseStdDeg = 0.0;
    return w;
}

// ------------------------------------------------------------------ stats
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

}  // namespace

int main(int argc, char** argv) {
    Options o = parseArgs(argc, argv);

    // Load the routine.
    MissionDef def;
    if (missions::isBuiltin(o.auton)) {
        def = missions::byName(o.auton);
    } else {
        const AutonParseResult r = loadAutonFile(o.auton);
        if (!r.ok()) {
            for (const auto& e : r.errors) std::fprintf(stderr, "%s: %s\n", o.auton.c_str(), e.c_str());
            return 1;
        }
        def = r.def;
    }
    if (o.budget < 0) o.budget = def.name.find("skills") != std::string::npos ? 60.0 : 15.0;
    if (o.hasFieldStart) { def.hasFieldStart = true; def.fieldStart = o.fieldStart; }

    // Timed pushes (drive_set) only mean something if there's a wall to push into.
    std::vector<std::string> warnings;
    if (!def.hasFieldStart) {
        std::string lines;
        for (const Instr& in : def.mission)
            if (in.op == Instr::Op::DriveRaw)
                lines += (lines.empty() ? "" : ", ") + (in.sourceLine ? std::to_string(in.sourceLine) : std::string("?"));
        if (!lines.empty())
            warnings.push_back("This routine pushes with drive_set (line " + lines + ") but the sim doesn't know where "
                               "the walls are, so the push drives through open field. Everything after it is not "
                               "meaningful. Pass --field-start X,Y,HEADING (or add # @field_start to the file).");
    }

    DriveGains gains;
    for (const auto& [k, v] : o.gainSets) {
        if (!applyGain(gains, k, v)) { std::fprintf(stderr, "unknown gain '%s'\n", k.c_str()); return 2; }
    }

    const Plan plan = makePlan(def);

    // Nominal run: nothing disturbed. Everything else is measured against it.
    const Run nominal = simulate(plan, gains, nominalDraw(), o.budget, true);

    // Disturbed runs.
    Sampler sampler(o.seed);
    std::vector<Run> runs;
    runs.reserve(static_cast<size_t>(o.runs));
    for (int i = 0; i < o.runs; ++i) runs.push_back(simulate(plan, gains, draw(sampler, o.d), o.budget, i < 40));

    // Timing.
    std::vector<double> motionEnd, endErr, endHeadErr;
    int onTime = 0, interfered = 0, unfinished = 0;
    for (const Run& r : runs) {
        motionEnd.push_back(r.motionEnd);
        endErr.push_back(dist(r.end, nominal.end));
        endHeadErr.push_back(std::fabs(r.end.theta - nominal.end.theta));
        if (r.motionEnd <= o.budget) ++onTime;
        if (!r.finished) ++unfinished;
        for (ExitReason e : r.exits)
            if (e == ExitReason::Velocity || e == ExitReason::Timeout) { ++interfered; break; }
    }

    // Per action: how far from where the nominal run fired it.
    struct ActionStat { int pc; double p50, p95, max; int fired; double nomT; Pose nom; };
    std::vector<ActionStat> actionStats;
    for (size_t a = 0; a < plan.actionPcs.size(); ++a) {
        std::vector<double> d;
        int fired = 0;
        for (const Run& r : runs) {
            if (!r.actions[a].fired || !nominal.actions[a].fired) continue;
            ++fired;
            d.push_back(dist(r.actions[a].at, nominal.actions[a].at));
        }
        actionStats.push_back({plan.actionPcs[a], percentile(d, 0.5), percentile(d, 0.95),
                               d.empty() ? 0 : *std::max_element(d.begin(), d.end()), fired,
                               nominal.actions[a].t, nominal.actions[a].at});
    }

    // Per blocking instruction: how often it ended on a velocity/timeout exit.
    std::vector<std::pair<int, int>> fragileWaits;  // (pc, count)
    for (size_t w = 0; w < plan.waitPcs.size(); ++w) {
        int bad = 0;
        for (const Run& r : runs)
            if (r.exits[w] == ExitReason::Velocity || r.exits[w] == ExitReason::Timeout) ++bad;
        if (bad) fragileWaits.push_back({plan.waitPcs[w], bad});
    }

    // Sensitivity: one factor at a time, everything else nominal. Two
    // questions, because they have different answers: what moves the robot
    // (end point), and what makes it late (finish time). A flat battery can
    // barely move the end point and still be why a routine misses the buzzer.
    struct Sens { int factor; double endP95; double timeP95; double onTimePct; };
    std::vector<Sens> sens;
    if (o.sensitivity) {
        const int per = std::max(30, o.runs / 4);
        for (int f = 0; f < kFactorCount; ++f) {
            Sampler s(o.seed * 7919 + static_cast<uint64_t>(f));
            std::vector<double> e, tt;
            int ok = 0;
            for (int i = 0; i < per; ++i) {
                Draw w = draw(s, o.d, f);
                w.plant.imuNoiseStdDeg = 0.0;  // isolate the factor
                const Run r = simulate(plan, gains, w, o.budget, false);
                e.push_back(dist(r.end, nominal.end));
                tt.push_back(r.motionEnd);
                if (r.motionEnd <= o.budget) ++ok;
            }
            sens.push_back({f, percentile(e, 0.95), percentile(tt, 0.95), 100.0 * ok / per});
        }
    }
    auto byEnd = sens, byTime = sens;
    std::sort(byEnd.begin(), byEnd.end(), [](auto& a, auto& b) { return a.endP95 > b.endP95; });
    std::sort(byTime.begin(), byTime.end(), [](auto& a, auto& b) { return a.timeP95 > b.timeP95; });

    // ------------------------------------------------------------ terminal
    const Instr* ins = nullptr;
    auto where = [&](int pc) {
        ins = &def.mission[static_cast<size_t>(pc)];
        std::string s = ins->op == Instr::Op::Action ? ins->label : ins->name();
        if (ins->sourceLine > 0) s += "  (line " + std::to_string(ins->sourceLine) + ")";
        return s;
    };
    const double pct = 100.0 * onTime / static_cast<double>(runs.size());

    // For runs that went over: what was running when the clock hit the limit?
    std::map<int, int> overAt;
    for (const Run& r : runs)
        if (r.motionEnd > o.budget && r.pcAtBudget >= 0) ++overAt[r.pcAtBudget];
    int overPc = -1, overN = 0;
    for (auto& [pc, n] : overAt) if (n > overN) { overPc = pc; overN = n; }
    std::string overWhere;
    if (overPc >= static_cast<int>(def.mission.size())) {
        // Past the last instruction: still finishing a motion with no wait after it.
        for (int i = static_cast<int>(def.mission.size()) - 1; i >= 0; --i) {
            const Instr& in = def.mission[static_cast<size_t>(i)];
            if (in.op == Instr::Op::DriveSet || in.op == Instr::Op::TurnSet || in.op == Instr::Op::SwingSet ||
                in.op == Instr::Op::OdomSet || in.op == Instr::Op::TurnToPoint || in.op == Instr::Op::TurnRelative) {
                overWhere = std::string("the last motion, ") + in.name() +
                            (in.sourceLine ? " at line " + std::to_string(in.sourceLine) : std::string()) +
                            " (nothing waits for it)";
                break;
            }
        }
    } else if (overPc >= 0) {
        const Instr& in = def.mission[static_cast<size_t>(overPc)];
        char buf[160];
        std::snprintf(buf, sizeof buf, "%s%s%s", in.op == Instr::Op::Action ? in.label.c_str() : in.name(),
                      in.op == Instr::Op::Delay ? (" " + std::to_string(static_cast<int>(in.a)) + " ms").c_str() : "",
                      in.sourceLine ? (" at line " + std::to_string(in.sourceLine)).c_str() : "");
        overWhere = buf;
    }
    if (o.brief) {
        const ActionStat* worstA = nullptr;
        for (const auto& a : actionStats) if (!worstA || a.p95 > worstA->p95) worstA = &a;
        std::string worstS = "-";
        if (worstA) {
            const Instr& in = def.mission[static_cast<size_t>(worstA->pc)];
            char buf[96];
            std::snprintf(buf, sizeof buf, "%.1f in @%d", worstA->p95, in.sourceLine);
            worstS = buf;
        }
        std::printf("%-30s %6.2f s %6.2f s %6.1f%% %7.1f in  %-16s %-20s %s\n", def.name.c_str(), nominal.motionEnd,
                    percentile(motionEnd, 0.95), pct, percentile(endErr, 0.95), worstS.c_str(),
                    byEnd.empty() ? "-" : kFactorNames[byEnd.front().factor],
                    !warnings.empty() ? "needs --field-start" : overWhere.empty() ? "" : ("over at " + overWhere).c_str());
        return 0;
    }
    std::printf("%s: %zu instructions, %zu mechanism calls, %d runs\n\n", def.name.c_str(), def.mission.size(),
                plan.actionPcs.size(), o.runs);
    std::printf("time    nominal %.2f s   p50 %.2f s   p95 %.2f s   worst %.2f s   (limit %.0f s)\n",
                nominal.motionEnd, percentile(motionEnd, 0.5), percentile(motionEnd, 0.95),
                *std::max_element(motionEnd.begin(), motionEnd.end()), o.budget);
    std::printf("        finishes in time in %.1f%% of runs\n", pct);
    if (!overWhere.empty())
        std::printf("        when time ran out it was usually on: %s (%d runs)\n", overWhere.c_str(), overN);
    if (nominal.total - nominal.motionEnd > 0.5)
        std::printf("        (then %.1f s of trailing delay the field clock will cut off)\n",
                    nominal.total - nominal.motionEnd);
    std::printf("end     p50 %.2f in   p95 %.2f in from nominal,  heading p95 %.1f deg\n", percentile(endErr, 0.5),
                percentile(endErr, 0.95), percentile(endHeadErr, 0.95));
    if (interfered) std::printf("        %d runs had a motion end on a velocity/timeout exit\n", interfered);
    if (unfinished) std::printf("        %d runs never finished\n", unfinished);

    if (!actionStats.empty()) {
        auto sorted = actionStats;
        std::sort(sorted.begin(), sorted.end(), [](auto& a, auto& b) { return a.p95 > b.p95; });
        std::printf("\nleast repeatable mechanism calls (distance from the nominal run when it fired):\n");
        for (size_t i = 0; i < std::min<size_t>(5, sorted.size()); ++i)
            std::printf("  p95 %5.2f in   at %5.2f s   %s\n", sorted[i].p95, sorted[i].nomT, where(sorted[i].pc).c_str());
    }
    if (!sens.empty()) {
        std::printf("\nwhat moves the end point most (p95, one factor at a time):\n");
        for (const auto& x : byEnd) std::printf("  %-20s %5.2f in\n", kFactorNames[x.factor], x.endP95);
        std::printf("\nwhat makes it late (p95 finish time, one factor at a time; nominal %.2f s):\n",
                    nominal.motionEnd);
        for (const auto& x : byTime)
            std::printf("  %-20s %+5.2f s   in time %5.1f%%\n", kFactorNames[x.factor],
                        x.timeP95 - nominal.motionEnd, x.onTimePct);
    }
    if (!fragileWaits.empty()) {
        std::printf("\nmotions that sometimes exit on velocity/timeout instead of reaching target:\n");
        for (auto& [pc, n] : fragileWaits)
            std::printf("  %4.1f%%  %s\n", 100.0 * n / runs.size(), where(pc).c_str());
    }
    for (const auto& w : warnings) std::printf("\nWARNING: %s\n", w.c_str());
    std::printf("\ndisturbances: placement %.2f in / %.2f deg (1-sigma), battery %.2f-%.2f, mismatch %.3f,\n"
                "              slip %.3f-%.3f, gyro drift %.3f deg/s, motor tau %.2f-%.2f s\n",
                o.d.placeXySigma, o.d.placeDegSigma, o.d.battery.lo, o.d.battery.hi, o.d.mismatchSigma,
                o.d.slip.lo, o.d.slip.hi, o.d.driftSigma, o.d.tau.lo, o.d.tau.hi);

    // ------------------------------------------------------------ json
    std::ostringstream j;
    j << "{\"name\":\"" << jsonEscape(def.name) << "\",\"source\":\"" << jsonEscape(o.auton) << "\","
      << "\"runs\":" << o.runs << ",\"budget\":" << num(o.budget) << ",\"seed\":" << o.seed << ","
      << "\"start\":[" << num(def.start.x) << "," << num(def.start.y) << "," << num(def.start.theta) << "],"
      << "\"walls\":" << (def.hasFieldStart ? "true" : "false") << ","
      << "\"over_at\":\"" << jsonEscape(overWhere) << "\",\"over_runs\":" << overN << ","
      << "\"warnings\":[";
    for (size_t i = 0; i < warnings.size(); ++i) j << (i ? "," : "") << "\"" << jsonEscape(warnings[i]) << "\"";
    j << "],"
      << "\"disturbances\":{\"place_xy\":" << num(o.d.placeXySigma) << ",\"place_deg\":" << num(o.d.placeDegSigma)
      << ",\"battery\":[" << num(o.d.battery.lo) << "," << num(o.d.battery.hi) << "],\"mismatch\":"
      << num(o.d.mismatchSigma) << ",\"slip\":[" << num(o.d.slip.lo) << "," << num(o.d.slip.hi) << "],\"drift\":"
      << num(o.d.driftSigma) << ",\"tau\":[" << num(o.d.tau.lo) << "," << num(o.d.tau.hi) << "]},"
      << "\"nominal\":{\"motion_end\":" << num(nominal.motionEnd) << ",\"total\":" << num(nominal.total)
      << ",\"end\":[" << num(nominal.end.x) << "," << num(nominal.end.y) << "," << num(nominal.end.theta) << "],"
      << "\"path\":[";
    for (size_t i = 0; i < nominal.path.size(); ++i)
        j << (i ? "," : "") << "[" << num(nominal.path[i].x, 2) << "," << num(nominal.path[i].y, 2) << "]";
    j << "]},\"time\":{\"p50\":" << num(percentile(motionEnd, 0.5)) << ",\"p95\":" << num(percentile(motionEnd, 0.95))
      << ",\"max\":" << num(*std::max_element(motionEnd.begin(), motionEnd.end())) << ",\"on_time_pct\":" << num(pct, 2)
      << ",\"all\":[";
    for (size_t i = 0; i < motionEnd.size(); ++i) j << (i ? "," : "") << num(motionEnd[i], 3);
    j << "]},\"end\":{\"p50\":" << num(percentile(endErr, 0.5)) << ",\"p95\":" << num(percentile(endErr, 0.95))
      << ",\"heading_p95\":" << num(percentile(endHeadErr, 0.95)) << ",\"points\":[";
    for (size_t i = 0; i < runs.size(); ++i)
        j << (i ? "," : "") << "[" << num(runs[i].end.x, 2) << "," << num(runs[i].end.y, 2) << "]";
    j << "]},\"interfered_runs\":" << interfered << ",\"paths\":[";
    for (size_t i = 0, n = 0; i < runs.size(); ++i) {
        if (runs[i].path.empty()) continue;
        j << (n++ ? "," : "") << "[";
        for (size_t k = 0; k < runs[i].path.size(); ++k)
            j << (k ? "," : "") << "[" << num(runs[i].path[k].x, 1) << "," << num(runs[i].path[k].y, 1) << "]";
        j << "]";
    }
    j << "],\"actions\":[";
    for (size_t a = 0; a < actionStats.size(); ++a) {
        const auto& s = actionStats[a];
        const Instr& in = def.mission[static_cast<size_t>(s.pc)];
        j << (a ? "," : "") << "{\"label\":\"" << jsonEscape(in.label) << "\",\"line\":" << in.sourceLine
          << ",\"pc\":" << s.pc << ",\"t\":" << num(s.nomT) << ",\"nominal\":[" << num(s.nom.x, 2) << ","
          << num(s.nom.y, 2) << "],\"p50\":" << num(s.p50) << ",\"p95\":" << num(s.p95) << ",\"max\":" << num(s.max)
          << ",\"points\":[";
        for (size_t i = 0, n = 0; i < runs.size() && n < 200; ++i) {
            if (!runs[i].actions[a].fired) continue;
            j << (n++ ? "," : "") << "[" << num(runs[i].actions[a].at.x, 2) << "," << num(runs[i].actions[a].at.y, 2)
              << "]";
        }
        j << "]}";
    }
    j << "],\"sensitivity\":[";
    for (size_t i = 0; i < byEnd.size(); ++i)
        j << (i ? "," : "") << "{\"factor\":\"" << kFactorNames[byEnd[i].factor] << "\",\"p95\":"
          << num(byEnd[i].endP95) << ",\"time_p95\":" << num(byEnd[i].timeP95) << ",\"late_s\":"
          << num(byEnd[i].timeP95 - nominal.motionEnd) << ",\"on_time_pct\":" << num(byEnd[i].onTimePct, 1) << "}";
    j << "],\"fragile\":[";
    for (size_t i = 0; i < fragileWaits.size(); ++i) {
        const Instr& in = def.mission[static_cast<size_t>(fragileWaits[i].first)];
        j << (i ? "," : "") << "{\"pc\":" << fragileWaits[i].first << ",\"what\":\"" << in.name() << "\",\"line\":"
          << in.sourceLine << ",\"pct\":" << num(100.0 * fragileWaits[i].second / runs.size(), 1) << "}";
    }
    j << "]}";
    const std::string json = j.str();

    if (!o.jsonPath.empty()) {
        std::ofstream(o.jsonPath) << json << "\n";
        std::printf("\nwrote %s\n", o.jsonPath.c_str());
    }
    if (!o.htmlPath.empty()) {
        std::string html = kReportTemplate;
        const std::string marker = "/*REPORT_DATA*/null";
        const size_t at = html.find(marker);
        if (at == std::string::npos) { std::fprintf(stderr, "report template is missing its data marker\n"); return 1; }
        html.replace(at, marker.size(), json);
        std::ofstream(o.htmlPath) << html;
        std::printf("%swrote %s\n", o.jsonPath.empty() ? "\n" : "", o.htmlPath.c_str());
    }
    return 0;
}
