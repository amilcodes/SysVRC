"""Which way is up, which way is forward, and what that makes the drivetrain.

CAD exports come in whatever axes the team modelled in (Onshape is Z-up,
Fusion often Y-up, and nobody puts the robot at the origin). The wheels give
it away: their axles run left-right, and their bottoms are the lowest thing on
the robot. The intake marks the front.

Robot frame (the sim's): inches, origin on the floor under the middle of the
drivetrain, +x right, +y forward, +z up.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .cad import Model
from .parts import Component

AXES = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}


def axis_vec(label: str) -> np.ndarray:
    i, s = AXES[label]
    v = np.zeros(3)
    v[i] = s
    return v


def axis_label(v: np.ndarray) -> str:
    i = int(np.argmax(np.abs(v)))
    return ("+" if v[i] > 0 else "-") + "xyz"[i]


@dataclass
class Frame:
    right: np.ndarray
    forward: np.ndarray
    up: np.ndarray
    origin: np.ndarray                     # world point (in)
    why: list[str] = field(default_factory=list)

    @property
    def matrix(self) -> np.ndarray:
        """4x4 world -> robot."""
        R = np.vstack([self.right, self.forward, self.up])
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = -R @ self.origin
        return M

    def to_robot(self, pts: np.ndarray) -> np.ndarray:
        pts = np.atleast_2d(pts)
        return (pts - self.origin) @ np.vstack([self.right, self.forward, self.up]).T

    def describe(self) -> dict:
        return {"up": axis_label(self.up), "forward": axis_label(self.forward), "right": axis_label(self.right),
                "origin": [round(float(v), 2) for v in self.origin]}


def _wheels(comps):
    return [c for c in comps if c.kind == "wheel"]


def infer(model: Model, comps: list[Component], up: str | None = None, forward: str | None = None) -> Frame:
    """Work the frame out from the parts; `up`/`forward` ("+z", "-x", ...)
    override the guesses when the user knows better."""
    why = []
    allv = model.all_vertices()
    wheels = _wheels(comps)

    # ---- up
    if up:
        up_v = axis_vec(up)
        why.append(f"up is {up} (given)")
    elif len(wheels) >= 2:
        best = None
        for lab in AXES:
            u = axis_vec(lab)
            ground = float(np.min(allv @ u))
            bottoms = [min(w.lo @ u, w.hi @ u) for w in wheels]
            centers = [float(w.center @ u) for w in wheels]
            ds = [w.attrs.get("diameter", 3.25) for w in wheels]
            # wheels sit on the floor: their bottoms are the robot's lowest points
            err = np.mean([abs(b - ground) for b in bottoms]) + np.mean([abs((c - ground) - d / 2) for c, d in zip(centers, ds)])
            # the wheel axles can't point up
            ax = [w.attrs.get("axis") for w in wheels if w.attrs.get("axis") is not None]
            if ax and np.mean([abs(float(np.dot(np.asarray(a), u))) for a in ax]) > 0.7:
                err += 100
            if best is None or err < best[0]:
                best = (err, lab, u)
        up_v = best[2]
        why.append(f"up is {best[1]}: the wheels' bottoms are the lowest points of the robot")
    else:
        up_v = axis_vec("+z")
        why.append("up is +z: no wheels found to tell (Onshape's default); flip it if the robot is on its side")

    # ---- left-right: along the wheel axles
    lateral = None
    ax = [np.asarray(w.attrs["axis"], dtype=float) for w in wheels if w.attrs.get("axis") is not None]
    if ax:
        a = np.mean([v if v @ ax[0] >= 0 else -v for v in ax], axis=0)
        a = a - (a @ up_v) * up_v
        if np.linalg.norm(a) > 1e-6:
            lateral = a / np.linalg.norm(a)
            why.append("left-right is along the wheel axles")
    if lateral is None:
        # the robot's shorter horizontal extent
        ext = allv.max(0) - allv.min(0)
        cands = [i for i in range(3) if abs(up_v[i]) < 0.5]
        i = min(cands, key=lambda k: ext[k])
        lateral = np.zeros(3)
        lateral[i] = 1
        why.append("left-right is the narrower side (no wheel axles to go by)")
    lateral = _snap(lateral)
    fwd_axis = _snap(np.cross(up_v, lateral))   # some horizontal direction, sign decided below

    # ---- origin: middle of the wheels, on the floor
    ground = float(np.min(allv @ up_v)) if len(allv) else 0.0
    if wheels:
        c = np.mean([w.center for w in wheels], axis=0)
        bottoms = [min(w.lo @ up_v, w.hi @ up_v) for w in wheels]
        ground = float(np.min(bottoms))
    else:
        c = (allv.min(0) + allv.max(0)) / 2
    origin = c - (c @ up_v - ground) * up_v

    # ---- front
    if forward:
        fwd = axis_vec(forward)
        why.append(f"front is {forward} (given)")
    else:
        fwd = fwd_axis
        intake = [x for x in comps if x.kind in ("flex_wheel",) or (x.kind in ("motor", "chain", "sprocket")
                  and any(k in x.name.lower() for k in ("intake", "roller", "conveyor", "hook")))]
        if not intake:
            intake = [x for x in comps if x.kind == "flex_wheel"]
        clampish = [x for x in comps if x.kind == "pneumatic" and any(k in x.name.lower() for k in ("clamp", "mogo", "goal"))]
        if intake:
            d = np.mean([(x.center - origin) @ fwd_axis for x in intake])
            fwd = fwd_axis if d >= 0 else -fwd_axis
            why.append(f"front is {axis_label(fwd)}: that's where the intake is")
        elif clampish:
            d = np.mean([(x.center - origin) @ fwd_axis for x in clampish])
            fwd = -fwd_axis if d >= 0 else fwd_axis
            why.append(f"front is {axis_label(fwd)}: the goal clamp is at the other end")
        else:
            why.append(f"front is {axis_label(fwd)}: a guess, nothing marks the front; flip it if it's backwards")
    right = _snap(np.cross(fwd, up_v))
    return Frame(right=right, forward=_snap(fwd), up=_snap(up_v), origin=origin, why=why)


def _snap(v: np.ndarray) -> np.ndarray:
    """Round to the nearest axis when it's within a few degrees of one."""
    v = np.asarray(v, dtype=float)
    v = v / (np.linalg.norm(v) or 1)
    i = int(np.argmax(np.abs(v)))
    if abs(v[i]) > 0.995:
        out = np.zeros(3)
        out[i] = np.sign(v[i])
        return out
    return v


# ---------------------------------------------------------------- measurements
def convex_hull(pts: np.ndarray) -> np.ndarray:
    pts = np.unique(np.round(pts, 3), axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def simplify(poly: np.ndarray, keep: int = 16) -> np.ndarray:
    """Drop the hull vertices that matter least until `keep` are left."""
    poly = list(map(tuple, poly))
    while len(poly) > keep:
        areas = []
        for i in range(len(poly)):
            a, b, c = poly[i - 1], poly[i], poly[(i + 1) % len(poly)]
            areas.append(abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])))
        poly.pop(int(np.argmin(areas)))
    return np.array(poly)


@dataclass
class Measures:
    footprint: dict
    outline: list
    wheels: list                 # {x, y, z, diameter, type}
    drive: dict                  # type, wheels_per_side, motors_per_side, track_width, wheelbase, ratio, cartridge, wheel_rpm
    drive_motors: list[str]
    other_motors: list[str]
    notes: list[str]


def measure(model: Model, comps: list[Component], fr: Frame) -> Measures:
    notes = []
    allv = fr.to_robot(model.all_vertices())
    lo, hi = allv.min(0), allv.max(0)
    footprint = {"width": round(float(hi[0] - lo[0]), 2), "length": round(float(hi[1] - lo[1]), 2),
                 "height": round(float(hi[2] - lo[2]), 2)}
    sub = allv[:: max(1, len(allv) // 40000)]
    outline = simplify(convex_hull(sub[:, :2]), 16)
    outline = [[round(float(x), 2), round(float(y), 2)] for x, y in outline]

    def rc(c):
        return fr.to_robot(c.center)[0]

    wheels = []
    for w in _wheels(comps):
        p = rc(w)
        wheels.append({"id": w.id, "x": round(float(p[0]), 2), "y": round(float(p[1]), 2), "z": round(float(p[2]), 2),
                       "diameter": w.attrs.get("diameter"), "type": w.attrs.get("type", "wheel")})
    left = [w for w in wheels if w["x"] < 0]
    right = [w for w in wheels if w["x"] >= 0]

    drive: dict = {"type": "tank"}
    if wheels:
        ds = [w["diameter"] for w in wheels if w["diameter"]]
        if ds:
            drive["wheel_diameter"] = max(set(ds), key=ds.count)
        drive["wheels_per_side"] = max(len(left), len(right))
        if left and right:
            drive["track_width"] = round(float(np.mean([w["x"] for w in right]) - np.mean([w["x"] for w in left])), 2)
        drive["wheelbase"] = round(float(max(w["y"] for w in wheels) - min(w["y"] for w in wheels)), 2)
        # wheel axles not all parallel to the robot's x axis: holonomic of some kind
        axes = [np.asarray(c.attrs["axis"]) for c in _wheels(comps) if c.attrs.get("axis") is not None]
        if axes:
            off = [abs(float(fr.to_robot(fr.origin + a)[0][0])) for a in axes]
            if any(o < 0.9 for o in off):
                drive["type"] = "x" if all(0.5 < o < 0.9 for o in off) else "h" if any(o < 0.2 for o in off) else "other"
                notes.append(f"not all wheel axles point sideways: looks like a {drive['type']}-drive")
        if any(w["type"] == "mecanum" for w in wheels):
            drive["type"] = "mecanum"
        if len(left) != len(right):
            notes.append(f"{len(left)} wheels on the left, {len(right)} on the right")

    # drive motors: low, out toward the wheels
    motors = [c for c in comps if c.kind == "motor"]
    drive_m, other_m = [], []
    half_track = drive.get("track_width", footprint["width"]) / 2
    ys = [w["y"] for w in wheels] or [lo[1], hi[1]]
    cand = []
    for m in motors:
        p = rc(m)
        low = p[2] < max(6.0, 2.2 * (drive.get("wheel_diameter") or 3.25))
        out = abs(p[0]) > 0.35 * half_track
        near = min(ys) - 3.0 <= p[1] <= max(ys) + 3.0
        (cand if low and out and near else other_m).append(m)
    # A drivetrain is symmetric: every drive motor has a twin on the other
    # side. A low motor out near the intake has none.
    for m in cand:
        p = rc(m)
        twin = any(abs(rc(o)[0] + p[0]) < 1.5 and abs(rc(o)[1] - p[1]) < 1.0 and abs(rc(o)[2] - p[2]) < 1.0
                   for o in cand if o is not m)
        (drive_m if twin else other_m).append(m)
    if drive_m:
        ls = sum(1 for m in drive_m if rc(m)[0] < 0)
        drive["motors_per_side"] = max(ls, len(drive_m) - ls)
        carts = [m.attrs.get("cartridge_rpm") for m in drive_m if m.attrs.get("cartridge_rpm")]
        if carts:
            drive["cartridge_rpm"] = max(set(carts), key=carts.count)
        w = [m.attrs.get("watts", 11) for m in drive_m]
        drive["motor"] = "5.5W" if w.count(5.5) > w.count(11) else "11W"

    # gearing: gears on a wheel's axle are driven, other drive-level gears drive them
    gears = [c for c in comps if c.kind == "gear" and c.attrs.get("teeth")]
    on_wheel, elsewhere = [], []
    for g in gears:
        p = rc(g)
        if p[2] > 8:
            continue
        near = [w for w in wheels if abs(w["y"] - p[1]) < 0.3 and abs(w["z"] - p[2]) < 0.3 and abs(w["x"] - p[0]) < 3]
        (on_wheel if near else elsewhere).append(g.attrs["teeth"])
    if on_wheel and elsewhere:
        driven = max(set(on_wheel), key=on_wheel.count)
        driving = max(set(elsewhere), key=elsewhere.count)
        drive["ratio"] = round(driving / driven, 4)
        notes.append(f"drive gearing {driving}T driving {driven}T on the wheels")
    elif wheels and not gears:
        drive["ratio"] = 1.0
        notes.append("no gears near the wheels: assuming direct drive")
    if "cartridge_rpm" in drive and "ratio" in drive:
        drive["wheel_rpm"] = round(drive["cartridge_rpm"] * drive["ratio"], 1)

    return Measures(footprint=footprint, outline=outline, wheels=wheels, drive=drive,
                    drive_motors=[m.id for m in drive_m], other_motors=[m.id for m in other_m], notes=notes)
