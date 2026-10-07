"""robot.json: the schema, defaults, and checks.

One schema serves three readers: it's the tool definition the model fills in,
what validate() checks a file against, and (via docs/robot_spec.md) what a
person editing robot.json by hand reads. core/include/visbot/robot_spec.hpp
reads the drivetrain; web/game.js reads the mechanisms and bindings.

Robot frame: inches, origin on the floor under the middle of the drivetrain,
+x right, +y forward, +z up. Zones are axis-aligned boxes in that frame,
[[x0, y0], [x1, y1]] seen from above.
"""
from __future__ import annotations

import copy
import re

SCHEMA_ID = "sysvrc-robot/1"
MECH_KINDS = ["intake", "goal_clamp", "wall_stake_arm", "lift", "doinker", "color_sort", "flywheel", "catapult",
              "hang", "other"]
OPS = ["speed", "toggle", "set", "state", "angle", "until", "release"]

ZONE = {"type": "array", "minItems": 2, "maxItems": 2,
        "items": {"type": "array", "minItems": 2, "maxItems": 2, "items": {"type": "number"}},
        "description": "[[x0, y0], [x1, y1]] box in the robot frame (in), seen from above"}

SCHEMA = {
    "type": "object",
    "required": ["name", "footprint", "drive", "mechanisms", "bindings", "evidence", "questions"],
    "properties": {
        "name": {"type": "string", "description": "short name for this robot"},
        "footprint": {"type": "object", "required": ["width", "length"], "properties": {
            "width": {"type": "number", "description": "left-right size at the start (in)"},
            "length": {"type": "number", "description": "front-back size at the start (in)"},
            "height": {"type": "number"}}},
        "outline": {"type": "array", "items": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                    "description": "top-view outline polygon in the robot frame (in)"},
        "mass_lb": {"type": "number", "description": "estimated mass (lb); a typical 6-motor VRC robot is 14-18"},
        "drive": {"type": "object", "required": ["type", "wheel_diameter", "track_width"], "properties": {
            "type": {"type": "string", "enum": ["tank", "x", "mecanum", "h", "asterisk", "other"]},
            "wheel_diameter": {"type": "number", "description": "in; 2.75, 3.25, 4 (4.125 for old 4 in omnis)"},
            "wheels_per_side": {"type": "integer"},
            "motors_per_side": {"type": "integer"},
            "motor": {"type": "string", "enum": ["11W", "5.5W"]},
            "cartridge_rpm": {"type": "integer", "enum": [100, 200, 600]},
            "ratio": {"type": "number", "description": "wheel turns per motor turn (36T driving 48T = 0.75)"},
            "wheel_rpm": {"type": "number", "description": "cartridge_rpm * ratio"},
            "track_width": {"type": "number", "description": "centre-to-centre of the left and right wheels (in)"},
            "wheelbase": {"type": "number"},
            "tau_s": {"type": "number", "description": "only if known; otherwise left out and estimated"}}},
        "code": {"type": "object", "description": "what the code tells the drive library (ez::Drive / lemlib::Drivetrain)",
                 "properties": {"wheel_diameter": {"type": "number"}, "wheel_rpm": {"type": "number"},
                                "cartridge_rpm": {"type": "integer"}, "track_width": {"type": "number"},
                                "where": {"type": "string"}}},
        "mechanisms": {"type": "array", "items": {"type": "object", "required": ["id", "kind"], "properties": {
            "id": {"type": "string", "description": "short id, e.g. intake, clamp, ladybrown"},
            "kind": {"type": "string", "enum": MECH_KINDS},
            "name": {"type": "string"},
            "code_names": {"type": "array", "items": {"type": "string"},
                           "description": "motor/piston objects in the code that drive it, e.g. [\"intake\"] or [\"mogoClamp\"]"},
            "actuator": {"type": "string", "enum": ["motor", "pneumatic", "none"]},
            "motors": {"type": "integer"}, "motor": {"type": "string"}, "cartridge_rpm": {"type": "integer"},
            "ratio": {"type": "number", "description": "output turns per motor turn"},
            "zone": ZONE,
            "capacity": {"type": "integer", "description": "intake: rings it can hold (game limit is 2)"},
            "transfer_s": {"type": "number", "description": "intake: seconds for a ring to go from the floor to the top"},
            "closed_when": {"type": "boolean", "description": "goal_clamp: piston value (extended=true) that holds a goal"},
            "states": {"type": "object", "additionalProperties": {"type": "number"},
                       "description": "wall_stake_arm: named positions in arm degrees, e.g. {\"REST\": 0, \"LOAD\": 25}"},
            "load_state": {"type": "string"}, "score_deg": {"type": "number"}, "deg_per_s": {"type": "number"},
            "reach": ZONE,
            "start": {"type": ["boolean", "string", "number"],
                      "description": "state when the autonomous period starts, from what autonomous() sets before the routine "
                                     "(color_sort on: true; an arm's state name or angle)"},
            "parts": {"type": "array", "items": {"type": "integer"}, "description": "tag numbers from the renders"},
            "notes": {"type": "string"}}}},
        "bindings": {"type": "array", "items": {"type": "object", "required": ["match", "mech", "do"], "properties": {
            "match": {"type": "string", "description": "regex (JS syntax) over the text inside action(\"...\")"},
            "mech": {"type": "string", "description": "a mechanism id"},
            "do": {"type": "string", "enum": OPS},
            "value": {"type": "string", "description": "literal or $1-style capture"},
            "scale": {"type": "number", "description": "for speed: what counts as full speed (127, 12000, 600)"}}}},
        "preload": {"type": "boolean"},
        "evidence": {"type": "array", "items": {"type": "object", "properties": {
            "about": {"type": "string"}, "source": {"type": "string", "enum": ["cad", "code", "both", "guess"]},
            "detail": {"type": "string"}}}},
        "questions": {"type": "array", "items": {"type": "string"},
                      "description": "what a person should check, most important first"},
        "confidence": {"type": "object", "additionalProperties": {"type": "string", "enum": ["high", "medium", "low"]}},
    },
}


def normalize(spec: dict) -> dict:
    """Fill what can be derived, drop what can't be right. Never invents a
    mechanism; does fix arithmetic (wheel_rpm from cartridge x ratio)."""
    s = copy.deepcopy(spec)
    s["schema"] = SCHEMA_ID
    d = s.setdefault("drive", {})
    if "wheel_rpm" not in d and d.get("cartridge_rpm") and d.get("ratio"):
        d["wheel_rpm"] = round(d["cartridge_rpm"] * d["ratio"], 1)
    if "ratio" not in d and d.get("cartridge_rpm") and d.get("wheel_rpm"):
        d["ratio"] = round(d["wheel_rpm"] / d["cartridge_rpm"], 4)
    for m in s.get("mechanisms", []):
        for key in ("zone", "reach"):
            z = m.get(key)
            if isinstance(z, list) and len(z) == 2:
                (x0, y0), (x1, y1) = z
                m[key] = [[round(min(x0, x1), 2), round(min(y0, y1), 2)], [round(max(x0, x1), 2), round(max(y0, y1), 2)]]
    s.setdefault("bindings", [])
    s.setdefault("evidence", [])
    s.setdefault("questions", [])
    return s


def validate(spec: dict) -> tuple[list[str], list[str]]:
    """(errors, warnings). Errors stop the sim from using the file."""
    err, warn = [], []
    d = spec.get("drive")
    if not isinstance(d, dict):
        return ["no drive"], warn
    for k in ("wheel_diameter", "track_width"):
        v = d.get(k)
        if not isinstance(v, (int, float)) or v <= 0:
            err.append(f"drive.{k} must be a positive number")
    wd = d.get("wheel_diameter") or 0
    if wd and not 1.5 <= wd <= 6.5:
        warn.append(f"a {wd} in drive wheel is unusual for VRC")
    rpm = d.get("wheel_rpm")
    if rpm and not 50 <= rpm <= 900:
        warn.append(f"wheel_rpm {rpm} is outside anything a V5 drive does")
    fp = spec.get("footprint") or {}
    for k in ("width", "length"):
        v = fp.get(k)
        if not isinstance(v, (int, float)) or v <= 0:
            err.append(f"footprint.{k} must be a positive number")
        elif v > 24.5:
            warn.append(f"footprint.{k} is {v} in: over the 18 in (24 in for the big robot) starting size")
    if d.get("track_width") and fp.get("width") and d["track_width"] > fp["width"] + 1:
        err.append("drive.track_width is wider than the robot")
    ids = set()
    for m in spec.get("mechanisms", []):
        if m.get("id") in ids:
            err.append(f"two mechanisms called {m.get('id')}")
        ids.add(m.get("id"))
        if m.get("kind") not in MECH_KINDS:
            err.append(f"mechanism {m.get('id')}: unknown kind {m.get('kind')}")
    for b in spec.get("bindings", []):
        if b.get("mech") not in ids:
            err.append(f"binding {b.get('match')!r} points at unknown mechanism {b.get('mech')!r}")
        if b.get("do") not in OPS:
            err.append(f"binding {b.get('match')!r}: unknown op {b.get('do')!r}")
        try:
            re.compile(b.get("match", ""))
        except re.error as e:
            err.append(f"binding {b.get('match')!r} is not a valid regex: {e}")
    return err, warn


def unbound(spec: dict, actions: list[str]) -> list[str]:
    """Action texts nothing in the spec will act on (bindings or code_names)."""
    pats = []
    for b in spec.get("bindings", []):
        try:
            pats.append(re.compile(b["match"]))
        except (re.error, KeyError):
            pass
    names = [n for m in spec.get("mechanisms", []) for n in m.get("code_names", [])]
    out = []
    for a in actions:
        if any(p.search(a) for p in pats):
            continue
        if any(re.match(rf"^{re.escape(n)}\.(move|move_voltage|move_velocity|brake|toggle|extend|retract|set_value)\(", a) for n in names):
            continue
        out.append(a)
    return out
