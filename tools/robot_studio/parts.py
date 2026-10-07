"""Recognise VEX parts in a CAD tree: motors, wheels, gears, pneumatics, sensors.

Names first: the VEX libraries for Onshape, Fusion and Inventor all name parts
("V5 Smart Motor (11W)", "3.25in Omni Wheel", "36T High Strength Gear"), and a
match on a sub-assembly claims everything under it, so a motor's housing,
cartridge and screws count as one motor. Geometry second: an unnamed cylinder
the size of a VEX wheel, lying on its side near the floor, is probably a wheel.

Nothing here decides what a part is *for* (drive motor or intake motor);
that's frame.py's geometry and the analyzer's job.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

from .cad import Model

WHEEL_SIZES = (2.0, 2.75, 3.25, 4.0, 4.125, 5.0)        # in, VEX omni/traction/flex
GEAR_TEETH_PITCH = 24.0                                  # VEX gears: teeth / 24 = pitch diameter (in)


@dataclass
class Component:
    id: str                       # node id (the matched node)
    name: str
    kind: str                     # see KINDS
    attrs: dict = field(default_factory=dict)
    leaves: list[str] = field(default_factory=list)      # leaf node ids it owns
    lo: np.ndarray | None = None  # world bounds, inches
    hi: np.ndarray | None = None
    by: str = "name"              # "name" or "shape"

    @property
    def center(self) -> np.ndarray:
        return (self.lo + self.hi) / 2

    @property
    def size(self) -> np.ndarray:
        return self.hi - self.lo


KINDS = ("motor", "wheel", "tracking_wheel", "flex_wheel", "gear", "sprocket", "chain", "pneumatic", "air_tank",
         "solenoid", "brain", "battery", "radio", "sensor", "structure", "other")

# (kind, pattern). Order matters: first match wins.
_RULES = [
    ("tracking_wheel", r"tracking|odom(etry)?\s*(pod|wheel)"),
    ("flex_wheel", r"flex\s*-?\s*wheel"),
    ("motor", r"smart\s*motor|\bv5\s*motor|\bexp\s*motor|\b11\s*w\b|\b5\.5\s*w\b|276-4840|276-4842|\bmotor\b(?!.*(mount|plate|cap|screw|spacer|insert))"),
    ("wheel", r"omni|traction|mecanum|anti-?static\s*wheel|\bwheel\b(?!.*(hub|insert|spacer|screw|shaft))"),
    ("gear", r"\bgear\b|\d+\s*t\b.*\bgear|\bpinion\b"),
    ("sprocket", r"sprocket"),
    ("chain", r"\bchain\b|tank\s*tread|\bflap\b|conveyor\s*(belt|link)"),
    ("air_tank", r"air\s*tank|reservoir|air\s*reserv"),
    ("solenoid", r"solenoid|\bvalve\b"),
    ("pneumatic", r"pneumatic|piston|\bstroke\b|\bair\s*cylinder\b|\bcylinder\b.*\d+\s*mm"),
    ("brain", r"\bbrain\b"),
    ("battery", r"battery"),
    ("radio", r"\bradio\b|vexnet"),
    ("sensor", r"inertial|\bimu\b|rotation\s*sensor|optical|distance\s*sensor|\bgps\b|vision|limit\s*switch|bumper|potentiometer|encoder|\bsensor\b"),
    ("structure", r"c-?channel|u-?channel|angle|\bplate\b|standoff|bearing|\bshaft\b|axle|spacer|screw|\bnut\b|collar|"
                  r"\brail\b|bracket|gusset|polycarb|lexan|\bstrip\b|hex\s*bar|rubber\s*band|zip\s*tie|washer|insert"),
]
_RULES = [(k, re.compile(p, re.I)) for k, p in _RULES]


def classify_name(name: str) -> tuple[str | None, dict]:
    n = name or ""
    for kind, rx in _RULES:
        if rx.search(n):
            return kind, attrs_from_name(kind, n)
    return None, {}


def attrs_from_name(kind: str, name: str) -> dict:
    a: dict = {}
    low = name.lower()
    if kind == "motor":
        a["watts"] = 5.5 if re.search(r"5\.5\s*w|\bexp\b|276-4842", low) else 11
        rpm = cartridge_rpm(low)
        if rpm:
            a["cartridge_rpm"] = rpm
    if kind in ("wheel", "flex_wheel", "tracking_wheel"):
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:in\b|\"|inch|”)", low) or re.search(r"\b(2\.75|3\.25|4\.125|4|5|2)\b", low)
        if m:
            a["diameter"] = float(m.group(1))
        a["type"] = "omni" if "omni" in low else "traction" if "traction" in low else \
            "mecanum" if "mecanum" in low else "flex" if "flex" in low else "wheel"
    if kind in ("gear", "sprocket"):
        m = re.search(r"(\d+)\s*-?\s*t(?:ooth)?\b", low) or re.search(r"\b(\d{2})\s*t", low)
        if m:
            a["teeth"] = int(m.group(1))
        a["high_strength"] = "high strength" in low or " hs" in f" {low}"
    if kind == "pneumatic":
        m = re.search(r"(\d+)\s*mm", low)
        if m:
            a["stroke_mm"] = int(m.group(1))
    if kind == "sensor":
        for key in ("inertial", "imu", "rotation", "optical", "distance", "gps", "vision"):
            if key in low:
                a["type"] = "inertial" if key == "imu" else key
                break
    return a


def cartridge_rpm(text: str) -> int | None:
    t = text.lower()
    if re.search(r"\b600\s*rpm|\bblue\b|6\s*:\s*1|\brpm_?600\b|gearset_?06", t):
        return 600
    if re.search(r"\b200\s*rpm|\bgreen\b|18\s*:\s*1|\brpm_?200\b|gearset_?18", t):
        return 200
    if re.search(r"\b100\s*rpm|\bred\b|36\s*:\s*1|\brpm_?100\b|gearset_?36", t):
        return 100
    return None


def classify(model: Model) -> list[Component]:
    """Every recognised component, plus the leftover leaves as 'other'."""
    comps: list[Component] = []
    claimed: set[str] = set()

    def descendants(nid):
        stack = [nid]
        while stack:
            n = model.nodes[stack.pop()]
            yield n
            stack.extend(n.children)

    # top-down: the highest named match claims its subtree
    def walk(nid):
        node = model.nodes[nid]
        if nid != model.root:
            kind, attrs = classify_name(node.name)
            if kind and kind != "structure":
                leaves = [d.id for d in descendants(nid) if d.is_leaf]
                if leaves:
                    # details often live on children ("Blue Cartridge 600 RPM")
                    for d in descendants(nid):
                        if d.id == nid:
                            continue
                        if kind == "motor" and "cartridge_rpm" not in attrs:
                            rpm = cartridge_rpm(d.name)
                            if rpm:
                                attrs["cartridge_rpm"] = rpm
                    comps.append(_make(model, nid, node.name, kind, attrs, leaves, "name"))
                    claimed.update(leaves)
                    return
        for c in node.children:
            walk(c)

    walk(model.root)

    # leaves nobody claimed: name them as structure/other, or recognise by shape
    for leaf in model.leaves():
        if leaf.id in claimed:
            continue
        kind, attrs = classify_name(leaf.name)
        if kind:
            comps.append(_make(model, leaf.id, leaf.name, kind, attrs, [leaf.id], "name"))
            continue
        guess = _by_shape(leaf)
        if guess:
            kind, attrs = guess
            comps.append(_make(model, leaf.id, leaf.name, kind, attrs, [leaf.id], "shape"))
        else:
            comps.append(_make(model, leaf.id, leaf.name, "other", {}, [leaf.id], "name"))

    # fill in sizes from geometry where names didn't say
    for c in comps:
        if c.kind in ("wheel", "flex_wheel", "tracking_wheel"):
            d, axis = _round_feature(model, c)
            if d and "diameter" not in c.attrs:
                c.attrs["diameter"] = snap(d, WHEEL_SIZES, 0.2) or round(d, 2)
            if axis is not None:
                c.attrs["axis"] = axis
        if c.kind in ("gear", "sprocket"):
            d, axis = _round_feature(model, c)
            if axis is not None:
                c.attrs["axis"] = axis
            if d and "teeth" not in c.attrs and c.kind == "gear":
                c.attrs["teeth"] = int(round(d * GEAR_TEETH_PITCH / 12.0) * 12) or None
    return comps


def snap(v: float, choices, tol: float):
    best = min(choices, key=lambda c: abs(c - v))
    return best if abs(best - v) <= tol else None


def _make(model, nid, name, kind, attrs, leaves, by) -> Component:
    pts = [model.nodes[l].vertices for l in leaves if model.nodes[l].vertices is not None and len(model.nodes[l].vertices)]
    allv = np.vstack(pts) if pts else np.zeros((1, 3))
    return Component(id=nid, name=name, kind=kind, attrs=dict(attrs), leaves=list(leaves), lo=allv.min(0), hi=allv.max(0), by=by)


def _round_feature(model, comp: Component):
    """Largest cylinder radius on the component and its axis (from B-rep data
    when the file had it, else from the bounding box: a wheel is a flat disc)."""
    best_r, axis = 0.0, None
    for l in comp.leaves:
        for c in model.nodes[l].cylinders:
            if c.radius > best_r:
                best_r, axis = c.radius, np.round(np.abs(c.axis), 3)
    if best_r > 0:
        return 2 * best_r, axis
    s = comp.size
    k = int(np.argmin(s))
    others = [s[i] for i in range(3) if i != k]
    if max(others) > 0 and min(others) / max(others) > 0.85 and s[k] < 0.6 * min(others):
        ax = np.zeros(3)
        ax[k] = 1.0
        return float(np.mean(others)), ax
    return None, None


def _by_shape(leaf):
    """An unnamed disc 2.75-4.125 in across and under 1.5 in thick: a wheel."""
    v = leaf.vertices
    if v is None or not len(v):
        return None
    if leaf.cylinders:
        r = max(c.radius for c in leaf.cylinders)
        d = 2 * r
        s = v.max(0) - v.min(0)
        if snap(d, WHEEL_SIZES[1:], 0.12) and min(s) < 1.6:
            return "wheel", {"diameter": snap(d, WHEEL_SIZES[1:], 0.12), "type": "wheel"}
        return None
    s = v.max(0) - v.min(0)
    k = int(np.argmin(s))
    o = [s[i] for i in range(3) if i != k]
    if min(o) / max(o) > 0.9 and s[k] < 1.6 and snap(float(np.mean(o)), WHEEL_SIZES[1:], 0.15):
        return "wheel", {"diameter": snap(float(np.mean(o)), WHEEL_SIZES[1:], 0.15), "type": "wheel"}
    return None


def summary(comps: list[Component]) -> dict:
    out: dict[str, int] = {}
    for c in comps:
        out[c.kind] = out.get(c.kind, 0) + 1
    return out
