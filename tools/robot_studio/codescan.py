"""What the team's code says about the robot.

Reads PROS (EZ-Template, LemLib) and VEXcode C++ for the things that pin a
robot down better than any CAD: the chassis constructor (wheel size, wheel rpm,
sometimes track width), every motor with its gearset, pistons and sensors, and
the helper functions the autons call (`setIntake(127)`, `ChangeLBState(...)`),
which is how an action("...") line maps onto a mechanism.

Regex, not a C++ parser: it reads the declarations real robot code uses and
says what it couldn't read rather than guessing.
"""
from __future__ import annotations

import pathlib
import re
from dataclasses import asdict, dataclass, field

from .parts import cartridge_rpm

SRC_EXT = {".cpp", ".cc", ".c", ".hpp", ".h"}


@dataclass
class Decl:
    name: str
    kind: str            # motor | motor_group | piston | imu | rotation | optical | distance | gps | vision
    ports: list = field(default_factory=list)
    cartridge_rpm: int | None = None
    where: str = ""
    text: str = ""


@dataclass
class CodeFacts:
    files: list[str] = field(default_factory=list)
    chassis: dict | None = None          # {"lib", "wheel_diameter", "wheel_rpm", "track_width", "left", "right", "where", "text"}
    decls: list[Decl] = field(default_factory=list)
    helpers: dict = field(default_factory=dict)      # name -> {"params", "body", "where", "calls": [obj.method]}
    constants: dict = field(default_factory=dict)    # NAME -> number (globals like STOP1 = 27)
    auton_setup: str = ""                            # body of autonomous(): what's switched on before the routine
    actions: list[str] = field(default_factory=list) # distinct action("...") texts from autons/*.auton

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


def _strip_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def _line(src: str, pos: int) -> int:
    return src.count("\n", 0, pos) + 1


def _ints(s: str) -> list[int]:
    return [int(x) for x in re.findall(r"-?\d+", s)]


GEARS = r"(?:pros::(?:v5::)?MotorGears?::\w+|pros::E_MOTOR_GEARSET_\d+|E_MOTOR_GEARSET_\d+|MotorGears::\w+|ratio\d+_1|\w*gear\w*)"


def scan(paths: list[str | pathlib.Path], autons_dir: str | pathlib.Path | None = None) -> CodeFacts:
    facts = CodeFacts()
    files = []
    for p in paths:
        p = pathlib.Path(p)
        if p.is_dir():
            files += sorted(f for f in p.rglob("*") if f.suffix in SRC_EXT and f.is_file())
        elif p.is_file():
            files.append(p)
    for f in files:
        try:
            raw = f.read_text(errors="replace")
        except OSError:
            continue
        src = _strip_comments(raw)
        rel = f.name
        facts.files.append(str(f))
        _chassis(src, raw, rel, facts)
        _decls(src, rel, facts)
        _helpers(src, rel, facts)
        _constants(src, facts)
    if autons_dir:
        seen = []
        for a in sorted(pathlib.Path(autons_dir).glob("*.auton")):
            for m in re.finditer(r'^action\("((?:[^"\\]|\\.)*)"\)', a.read_text(errors="replace"), re.M):
                t = m.group(1).replace('\\"', '"')
                if t not in seen:
                    seen.append(t)
        facts.actions = seen
    return facts


def _chassis(src, raw, rel, facts):
    # EZ-Template: ez::Drive chassis({left}, {right}, imu, wheel_diameter, wheel_rpm [, ...]);
    m = re.search(r"ez::Drive\s+\w+\s*\(\s*(\{[^}]*\})\s*,\s*(\{[^}]*\})\s*,\s*([^,]+),\s*([\d.]+)\s*,\s*([\d.]+)", src)
    if m and not facts.chassis:
        facts.chassis = {"lib": "EZ-Template", "left": _ints(m.group(1)), "right": _ints(m.group(2)),
                         "wheel_diameter": float(m.group(4)), "wheel_rpm": float(m.group(5)),
                         "where": f"{rel}:{_line(src, m.start())}", "text": " ".join(m.group(0).split())}
        return
    # LemLib: lemlib::Drivetrain dt(&left, &right, track_width, lemlib::Omniwheel::NEW_325, rpm, drift);
    m = re.search(r"lemlib::Drivetrain\s+\w+\s*\(\s*&?(\w+)\s*,\s*&?(\w+)\s*,\s*([\d.]+)\s*,\s*([\w:.]+)\s*,\s*([\d.]+)", src)
    if m and not facts.chassis:
        wheel = m.group(4)
        w = re.search(r"(\d)(\d{2,3})$", wheel.split("::")[-1])   # NEW_325 -> 3.25, OLD_4 -> 4
        dia = float(f"{w.group(1)}.{w.group(2)}") if w else (float(wheel) if re.fullmatch(r"[\d.]+", wheel) else None)
        if dia is None:
            n = re.search(r"_(\d)$", wheel)
            dia = float(n.group(1)) if n else None
        facts.chassis = {"lib": "LemLib", "left_group": m.group(1), "right_group": m.group(2),
                         "track_width": float(m.group(3)), "wheel_diameter": dia, "wheel_rpm": float(m.group(5)),
                         "where": f"{rel}:{_line(src, m.start())}", "text": " ".join(m.group(0).split())}


def _decls(src, rel, facts):
    pats = [
        ("motor_group", rf"pros::(?:v5::)?MotorGroup\s+(\w+)\s*\(\s*(\{{[^}}]*\}})\s*(?:,\s*({GEARS}))?"),
        ("motor", rf"pros::(?:v5::)?Motor\s+(\w+)\s*\(\s*(-?\d+)\s*(?:,\s*({GEARS}))?"),
        ("motor", r"\bmotor\s+(\w+)\s*=\s*motor\s*\(\s*PORT(\d+)\s*(?:,\s*(ratio\d+_1))?"),
        ("motor_group", r"\bmotor_group\s+(\w+)\s*=\s*motor_group\s*\(([^)]*)\)"),
        ("piston", r"pros::adi::(?:Pneumatics|DigitalOut)\s+(\w+)\s*\(\s*'?(\w)'?"),
        ("piston", r"ez::Piston\s+(\w+)\s*\(\s*'?(\w)'?"),
        ("piston", r"\bdigital_out\s+(\w+)\s*=\s*digital_out\s*\([^)]*\.(\w)\s*\)"),
        ("imu", r"pros::(?:v5::)?Imu\s+(\w+)\s*\(\s*(\d+)"),
        ("rotation", r"pros::(?:v5::)?Rotation\s+(\w+)\s*\(\s*(-?\d+)"),
        ("optical", r"pros::(?:v5::)?Optical\s+(\w+)\s*\(\s*(\d+)"),
        ("distance", r"pros::(?:v5::)?Distance\s+(\w+)\s*\(\s*(\d+)"),
        ("gps", r"pros::(?:v5::)?Gps\s+(\w+)\s*\(\s*(\d+)"),
        ("vision", r"pros::(?:v5::)?(?:Vision|AIVision)\s+(\w+)\s*\(\s*(\d+)"),
    ]
    have = {d.name for d in facts.decls}
    for kind, pat in pats:
        for m in re.finditer(pat, src):
            name = m.group(1)
            if name in have:
                continue
            ports = _ints(m.group(2)) if m.group(2) and kind not in ("piston",) else [m.group(2)]
            gear = m.group(3) if m.lastindex and m.lastindex >= 3 else None
            rpm = cartridge_rpm(gear) if gear else None
            if kind in ("motor", "motor_group") and gear is None:
                rpm = 200 if "pros::" in m.group(0) else None   # PROS default gearset is green
            facts.decls.append(Decl(name=name, kind=kind, ports=ports, cartridge_rpm=rpm,
                                    where=f"{rel}:{_line(src, m.start())}", text=" ".join(m.group(0).split())))
            have.add(name)


def _helpers(src, rel, facts):
    """Free functions with a body that touches a declared motor or piston."""
    for m in re.finditer(r"\b(?:void|int|bool|double|float)\s+(\w+)\s*\(([^)]*)\)\s*\{", src):
        name = m.group(1)
        # body: match braces
        i, depth = m.end(), 1
        while i < len(src) and depth:
            depth += {"{": 1, "}": -1}.get(src[i], 0)
            i += 1
        body = src[m.end():i - 1]
        if name == "autonomous" and not facts.auton_setup:
            facts.auton_setup = "\n".join(l for l in body.strip().splitlines() if l.strip())[:3000]
        if name in ("initialize", "autonomous", "opcontrol", "disabled", "competition_initialize", "main"):
            continue
        if len(body) > 6000:
            body = body[:6000] + "\n  ..."
        calls = sorted(set(re.findall(r"\b(\w+)\.(move\w*|toggle|extend|retract|set_value|brake|set_zero_position)\s*\(", body)))
        facts.helpers[name] = {"params": " ".join(m.group(2).split()), "body": body.strip(), "where": f"{rel}:{_line(src, m.start())}",
                               "calls": [f"{o}.{f}" for o, f in calls]}


def _constants(src, facts):
    for m in re.finditer(r"^\s*(?:const\s+)?(?:double|float|int|long)\s+([A-Z][A-Z0-9_]*)\s*=\s*([^;]+);", src, re.M):
        expr = m.group(2)
        # resolve simple arithmetic over numbers and already-known constants
        for k, v in sorted(facts.constants.items(), key=lambda kv: -len(kv[0])):
            expr = re.sub(rf"\b{k}\b", str(v), expr)
        if re.fullmatch(r"[\d.\s+\-*/()]+", expr):
            try:
                facts.constants[m.group(1)] = round(float(eval(expr, {"__builtins__": {}})), 4)   # digits and operators only
            except (SyntaxError, ZeroDivisionError, ValueError):
                pass
