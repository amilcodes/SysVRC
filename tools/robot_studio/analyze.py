"""CAD + code -> robot.json.

Two passes. heuristic() needs nothing but the files: the drivetrain from the
chassis constructor and the CAD geometry, mechanisms from what the code calls
its motors and pistons, bindings from helper functions whose bodies are plain
(`void setIntake(int p) { intake.move(p); }`). claude() then shows a model the
renders, the parts table, the measurements and the code, and has it fill in the
same schema: which motor is the intake, what the arm's positions are in
degrees, what `ChangeLBState(EXTENDED)` does. Its answer is validated against
the schema like a hand-written file would be.
"""
from __future__ import annotations

import base64
import json
import os
import pathlib
import re
import urllib.error
import urllib.request
from dataclasses import dataclass

import numpy as np

from . import cad, codescan, frame, parts, render, spec

DEFAULT_MODEL = os.environ.get("STUDIO_MODEL", "claude-opus-5-5")
API_URL = os.environ.get("ANTHROPIC_BASE_URL", "https://api.anthropic.com").rstrip("/") + "/v1/messages"


@dataclass
class Analysis:
    model: cad.Model | None
    comps: list
    frame: frame.Frame | None
    measures: frame.Measures | None
    index: dict            # component id -> tag number on the renders
    views: dict            # name -> PIL image
    code: codescan.CodeFacts

    def parts_table(self) -> list[dict]:
        out = []
        for c in self.comps:
            if c.id not in self.index:
                continue
            p = self.frame.to_robot(c.center)[0]
            lo = self.frame.to_robot(c.lo)[0]
            hi = self.frame.to_robot(c.hi)[0]
            size = np.abs(hi - lo)
            attrs = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in c.attrs.items() if k != "axis"}
            if "axis" in c.attrs:
                ax = self.frame.to_robot(self.frame.origin + np.asarray(c.attrs["axis"]))[0]
                attrs["axis"] = "xyz"[int(np.argmax(np.abs(ax)))]
            out.append({"tag": self.index[c.id], "kind": c.kind, "name": c.name, "path": "/".join(self.model.path(c.id)[:-1][-2:]),
                        "at": [round(float(v), 1) for v in p], "size": [round(float(v), 1) for v in size], **attrs})
        return out


def gather(cad_path: str | None, code_paths: list[str], autons_dir: str | None = None,
           up: str | None = None, forward: str | None = None, render_size: int = 800) -> Analysis:
    code = codescan.scan(code_paths, autons_dir) if code_paths or autons_dir else codescan.CodeFacts()
    if not cad_path:
        return Analysis(None, [], None, None, {}, {}, code)
    model = cad.load(cad_path)
    comps = parts.classify(model)
    fr = frame.infer(model, comps, up=up, forward=forward)
    ms = frame.measure(model, comps, fr)
    tagged = [c for c in comps if c.kind not in ("structure", "other")]
    tagged.sort(key=lambda c: (c.kind, -fr.to_robot(c.center)[0][1]))
    index = {c.id: i + 1 for i, c in enumerate(tagged)}
    views = render.render_all(model, comps, fr, index, size=render_size)
    return Analysis(model, comps, fr, ms, index, views, code)


# ---------------------------------------------------------------- heuristic
_KIND_BY_NAME = [
    ("intake", r"intake|conveyor|hook|roller|belt"),
    ("goal_clamp", r"clamp|mogo|goal"),
    ("wall_stake_arm", r"lady|^lb|lbmotor|wall|stake|arm(?!.*doink)"),
    ("doinker", r"doink|sweep|corner|swiper|rush"),
    ("lift", r"lift|raise|tilt"),
    ("hang", r"hang|climb|ptos?"),
    ("flywheel", r"fly"),
    ("catapult", r"cata|puncher"),
]


def _kind_for(name: str, is_piston: bool) -> str:
    low = name.lower()
    rules = _KIND_BY_NAME
    if is_piston:
        # a piston named intakeLift lifts the intake; it isn't the intake
        first = ("goal_clamp", "doinker", "lift", "hang")
        rules = [r for r in _KIND_BY_NAME if r[0] in first] + [r for r in _KIND_BY_NAME if r[0] not in first]
    for kind, pat in rules:
        if re.search(pat, low):
            if kind == "wall_stake_arm" and is_piston:
                continue
            return kind
    return "other"


def heuristic(an: Analysis, name: str = "robot") -> dict:
    ms, code = an.measures, an.code
    ev, qs = [], []
    fp = dict(ms.footprint) if ms else {"width": 15.0, "length": 15.0, "height": 18.0}
    if not ms:
        qs.append("No CAD: footprint and mechanism positions are defaults.")
    drive = dict(ms.drive) if ms else {"type": "tank"}
    drive.setdefault("type", "tank")
    ch = code.chassis or {}
    groups = {d.name: d for d in code.decls if d.kind in ("motor_group", "motor")}
    drive_ports = set(abs(p) for p in ch.get("left", []) + ch.get("right", []))
    if ch:
        if ch.get("wheel_diameter") and drive.get("wheel_diameter") and abs(ch["wheel_diameter"] - drive["wheel_diameter"]) > 0.05:
            qs.append(f"The code says {ch['wheel_diameter']} in wheels, the CAD shows {drive['wheel_diameter']} in.")
        drive.setdefault("wheel_diameter", ch.get("wheel_diameter"))
        if ch.get("track_width"):
            drive.setdefault("track_width", ch["track_width"])
        if ch.get("left"):
            drive.setdefault("motors_per_side", len(ch["left"]))
        ev.append({"about": "drive", "source": "code", "detail": f"{ch['text']} ({ch['where']})"})
    # drive cartridge from the motor groups the chassis uses
    drive_decls = [d for d in code.decls if d.kind in ("motor_group", "motor") and
                   (set(abs(p) for p in d.ports) & drive_ports or d.name in (ch.get("left_group"), ch.get("right_group")))]
    carts = [d.cartridge_rpm for d in drive_decls if d.cartridge_rpm]
    if carts and not drive.get("cartridge_rpm"):
        drive["cartridge_rpm"] = carts[0]
    if ch.get("wheel_rpm") and not drive.get("wheel_rpm"):
        drive["wheel_rpm"] = ch["wheel_rpm"]
    if drive.get("wheel_rpm") and ch.get("wheel_rpm") and abs(drive["wheel_rpm"] - ch["wheel_rpm"]) > 2:
        qs.append(f"The CAD's gearing gives {drive['wheel_rpm']} rpm at the wheels; the code tells EZ {ch['wheel_rpm']}.")
    drive.setdefault("wheel_diameter", 3.25)
    drive.setdefault("track_width", round(fp["width"] - 2.5, 2))
    if ms and ms.notes:
        ev += [{"about": "drive", "source": "cad", "detail": n} for n in ms.notes]

    # mechanisms from the code's objects, placed with the CAD when we have it
    mechs: dict[str, dict] = {}
    for d in code.decls:
        if d.kind in ("motor", "motor_group"):
            if set(abs(p) for p in d.ports) & drive_ports or d in drive_decls or re.search(r"drive|chassis|left|right", d.name, re.I):
                continue
            kind = _kind_for(d.name, False)
        elif d.kind == "piston":
            kind = _kind_for(d.name, True)
        else:
            continue
        if kind == "other":
            continue
        mid = {"intake": "intake", "goal_clamp": "clamp", "wall_stake_arm": "arm"}.get(kind, re.sub(r"\W+", "_", d.name.lower()))
        m = mechs.setdefault(mid, {"id": mid, "kind": kind, "name": d.name, "code_names": [],
                                   "actuator": "pneumatic" if d.kind == "piston" else "motor"})
        m["code_names"].append(d.name)
        if d.kind != "piston":
            m["motors"] = m.get("motors", 0) + max(1, len(d.ports))
            if d.cartridge_rpm:
                m["cartridge_rpm"] = d.cartridge_rpm
        ev.append({"about": mid, "source": "code", "detail": f"{d.text} ({d.where})"})
    if any(d.kind == "optical" for d in code.decls) and "intake" in mechs:
        mechs["color_sort"] = {"id": "color_sort", "kind": "color_sort", "name": "optical color sort"}
        # switched on in autonomous() before the routine?
        on = re.search(r"\b\w*(?:colou?r|sort|filter)\w*\s*=\s*true\b", code.auton_setup, re.I)
        if on:
            mechs["color_sort"]["start"] = True
            ev.append({"about": "color_sort", "source": "code", "detail": f"autonomous() sets {on.group(0)}"})

    w, l = fp["width"], fp["length"]
    front, back = (max(p[1] for p in ms.outline), min(p[1] for p in ms.outline)) if ms and ms.outline else (l / 2, -l / 2)
    if "intake" in mechs:
        xs = [r["at"][0] for r in an.parts_table() if r["kind"] == "flex_wheel"] if ms else []
        half = (max(abs(x) for x in xs) + 1.5) if xs else min(5.0, w / 2 - 2)
        mechs["intake"].update({"zone": [[-half, front - 3.5], [half, front + 1.5]], "capacity": 2, "transfer_s": 0.6})
    if "clamp" in mechs:
        mechs["clamp"].update({"zone": [[-4, back - 1.5], [4, back + 4.5]], "closed_when": True})
        qs.append("Does the clamp hold the goal with the piston extended (closed_when: true) or retracted?")
    if "arm" in mechs:
        mechs["arm"].update({"load_state": "LOAD", "states": {}, "score_deg": 150, "reach": [[-4, front - 1], [4, front + 9]]})
        qs.append("The arm's positions (states, in degrees) need filling in; the code's helpers define them.")

    # bindings: helpers that just pass their argument to a motor, or toggle a piston
    obj_mech = {n: m["id"] for m in mechs.values() for n in m.get("code_names", [])}
    used = {re.match(r"^(\w+)\(", a).group(1) for a in code.actions if re.match(r"^(\w+)\(", a)}
    binds = []
    for fn, h in code.helpers.items():
        if fn not in used:
            continue
        body = h["body"]
        params = [p.split()[-1].strip("&*") for p in h["params"].split(",") if p.strip() and p.strip() != "void"]
        mv = re.search(r"\b(\w+)\.(move|move_voltage|move_velocity)\s*\(\s*(\w+)\s*\)", body)
        if mv and mv.group(1) in obj_mech and params and mv.group(3) == params[0]:
            scale = {"move": 127, "move_voltage": 12000, "move_velocity": 600}[mv.group(2)]
            binds.append({"match": rf"^{fn}\((.+)\)$", "mech": obj_mech[mv.group(1)], "do": "speed", "value": "$1", "scale": scale})
            continue
        tg = re.search(r"\b(\w+)\.toggle\s*\(\s*\)", body)
        if tg and tg.group(1) in obj_mech and not params and len(re.findall(r";", body)) <= 2:
            binds.append({"match": rf"^{fn}\(\)$", "mech": obj_mech[tg.group(1)], "do": "toggle"})

    out = {"name": name, "footprint": fp, "mass_lb": 16.0, "drive": drive, "mechanisms": list(mechs.values()),
           "bindings": binds, "preload": True, "evidence": ev, "questions": qs}
    if ms:
        out["outline"] = ms.outline
    if ch:
        out["code"] = {k: ch[k] for k in ("wheel_diameter", "wheel_rpm", "track_width") if ch.get(k)}
        if carts:
            out["code"]["cartridge_rpm"] = carts[0]
        out["code"]["where"] = ch["where"]
    out = spec.normalize(out)
    left = spec.unbound(out, code.actions)
    if left:
        out["questions"].append(f"{len(left)} kinds of action line don't move anything yet, e.g. {', '.join(left[:4])}")
    return out


# ---------------------------------------------------------------- the model
SYSTEM = """You turn a VEX V5 Robotics Competition robot into a simulation model.

You get renders of the robot's CAD (parts coloured by type: motors red, wheels dark grey, gears yellow, pneumatics blue, flex wheels and chain cyan, sensors purple, structure light grey; motors, pistons, flex wheels and sensors carry numbered tags that match the parts table), measurements already taken from the CAD, facts read from the team's code, the distinct action("...") lines their autonomous routines use, and a draft made without you. Fill in robot.json with submit_robot.

Frame: inches, origin on the floor under the middle of the drivetrain, +x right, +y forward, +z up. The top view is drawn with the front at the top.

How to decide:
- The code is exact about what it states: the chassis constructor's wheel size and rpm, each motor's gearset, which motors and pistons exist and their names. The CAD is right about geometry: positions, sizes, track width, gear tooth counts. drive.* is the robot as built; code.* is what the code tells the drive library. If they disagree, keep both and say so in questions; that disagreement is a real bug on the robot.
- One mechanism per job (two lady brown motors are one wall_stake_arm). Use only the listed kinds. Give each the code_names of the objects that drive it.
- Zones are where things happen, from the CAD: intake.zone is where a ring on the floor gets grabbed (the front mouth, as wide as the rollers); goal_clamp.zone is where a mobile goal's stake sits once clamped; wall_stake_arm.reach is where the arm's tip is when it scores on a wall stake.
- autonomous_setup is what autonomous() does before the routine runs: mechanisms it switches on there (a colour sort, an arm's state) go in that mechanism's start.
- An arm's states are arm angles in degrees. Teams often measure the motor, not the arm: convert through the gear ratio. Read the named positions from the code's constants and helper functions.
- Every action line should end up driving a mechanism through code_names (plain obj.move(...), obj.toggle()) or a binding (team helpers, state setters). Bindings are JavaScript regexes over the exact text inside action("..."), anchored ^...$; value can be a literal or $1. Lines that only set sensor flags or brake modes can stay unbound; say which in evidence.
- High Stakes facts: a robot may hold 2 rings and 1 mobile goal; the preload is one alliance ring; clamps usually grab a goal at the back; intakes usually run a hook conveyor up to the goal and to a lady brown that scores on wall stakes.
- Be honest. What you inferred rather than read is source "guess" in evidence, and if it changes what the sim does, it belongs in questions, most important first. Do not invent mechanisms the CAD and code don't show."""


def _prompt(an: Analysis, draft: dict) -> list:
    content = []
    if an.views:
        sheet = render.contact_sheet(an.views, size=next(iter(an.views.values())).size[0])
        content.append({"type": "image", "source": {"type": "base64", "media_type": "image/png",
                                                    "data": base64.b64encode(render.png_bytes(sheet, 1568)).decode()}})
    code = an.code
    used = {re.match(r"^(\w+)", a).group(1) for a in code.actions if re.match(r"^(\w+)", a)}
    helpers = {k: v for k, v in code.helpers.items() if k in used or any(k in a for a in code.actions)}
    # constants the helpers mention
    consts = {k: v for k, v in code.constants.items() if any(re.search(rf"\b{k}\b", h["body"]) for h in helpers.values())
              or any(k in a for a in code.actions)}
    facts = {
        "cad": None if not an.model else {
            "file": pathlib.Path(an.model.source).name, "named_parts": an.model.named,
            "frame": an.frame.describe() | {"how": an.frame.why},
            "footprint": an.measures.footprint, "outline": an.measures.outline,
            "wheels": an.measures.wheels, "drive_from_cad": an.measures.drive, "notes": an.measures.notes,
            "parts": an.parts_table(),
            "unrecognised_parts": sum(1 for c in an.comps if c.kind == "other"),
        },
        "code": {"chassis": code.chassis, "declarations": [d.__dict__ for d in code.decls],
                 "helpers": helpers, "constants": consts, "autonomous_setup": code.auton_setup},
        "actions": code.actions,
        "draft": draft,
    }
    text = ("Here is everything about the robot. The image is four views: top (front up), iso, front, side.\n\n"
            + json.dumps(facts, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    if len(text) > 180_000:
        facts["code"]["helpers"] = {k: {**v, "body": v["body"][:1500]} for k, v in helpers.items()}
        text = json.dumps(facts, default=str)[:180_000]
    content.append({"type": "text", "text": text})
    return [{"role": "user", "content": content}]


def claude(an: Analysis, draft: dict, api_key: str | None = None, model: str | None = None,
           timeout: float = 600.0, post=None) -> tuple[dict, dict]:
    """Ask the model; returns (spec, raw response). `post` replaces the HTTP
    call (tests)."""
    api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
    if not api_key and post is None:
        raise RuntimeError("set ANTHROPIC_API_KEY to analyse with Claude")
    body = {
        "model": model or DEFAULT_MODEL,
        "max_tokens": 16000,
        "system": SYSTEM,
        "messages": _prompt(an, draft),
        "tools": [{"name": "submit_robot", "description": "Submit the robot.json for this robot.", "input_schema": spec.SCHEMA}],
        "tool_choice": {"type": "tool", "name": "submit_robot"},
    }
    if post is None:
        req = urllib.request.Request(API_URL, data=json.dumps(body).encode(), method="POST", headers={
            "content-type": "application/json", "x-api-key": api_key, "anthropic-version": "2023-06-01"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                raw = json.loads(r.read())
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:500]
            raise RuntimeError(f"Claude API error {e.code}: {detail}") from None
    else:
        raw = post(body)
    out = next((b.get("input") for b in raw.get("content", []) if b.get("type") == "tool_use" and b.get("name") == "submit_robot"), None)
    if not isinstance(out, dict):
        raise RuntimeError("the model didn't return a robot spec")
    merged = merge(draft, out)
    merged["source"] = {**(draft.get("source") or {}), "analyzer": raw.get("model", body["model"])}
    return spec.normalize(merged), raw


def merge(draft: dict, ai: dict) -> dict:
    """The model's answer, with the draft filling anything it left out."""
    out = dict(draft)
    for k, v in ai.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = {**out[k], **v}
        elif v not in (None, [], {}):
            out[k] = v
    return out
