#!/usr/bin/env python3
"""path.jerryio export -> .auton, so a drawn path can go through auton_check.

    tools/path_import.py my_path.txt -o autons/my_path.auton [--reverse] [--spacing 8]

path.jerryio's LemLib export is the points it samples along the path
("x, y, speed" lines up to `endData`) followed by `#PATH.JERRYIO-DATA {json}`
describing the format (units, coordinate system). The importer takes the
sampled points, keeps one every --spacing inches, and writes them as
pid_odom_set moves chained with pid_wait_quick_chain, which is how an EZ
routine would drive the path point to point. EZ-Template's own pure pursuit
isn't ported, so treat cornering as approximate.

The routine starts at the path's first point facing along it, written as a
field start (`# @field_start`), so the sim knows where the walls are. Add
action("...") lines by hand where the mechanisms should run.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys


def parse(text: str) -> dict:
    """Points (inches, field frame), speeds (0-127) and the path's metadata."""
    meta = {}
    m = re.search(r"#PATH\.JERRYIO-DATA\s+(\{.*\})\s*$", text, re.S)
    if m:
        try:
            meta = json.loads(m.group(1))
        except json.JSONDecodeError:
            meta = {}
    body = text[:m.start()] if m else text
    first = body.split("endData")[0]
    pts = []
    for line in first.splitlines():
        nums = re.findall(r"-?\d+(?:\.\d+)?(?:e-?\d+)?", line)
        if len(nums) >= 2:
            x, y = float(nums[0]), float(nums[1])
            speed = float(nums[2]) if len(nums) >= 3 else 127.0
            pts.append((x, y, speed))
    if not pts:
        raise ValueError("no path points found (expected path.jerryio's LemLib export: 'x, y, speed' lines)")

    fmt = (meta.get("format") or "").lower()
    if "(cm" in fmt or " cm" in fmt or "centimet" in fmt:
        scale = 1 / 2.54
    elif "mm" in fmt and "inch" not in fmt:
        scale = 1 / 25.4
    elif "inch" in fmt or "in," in fmt:
        scale = 1.0
    else:
        extent = max(max(abs(x), abs(y)) for x, y, _ in pts)
        scale = 1 / 2.54 if extent > 80 else 1.0           # half a field is 72 in or 183 cm
    gc = meta.get("gc") or {}
    system = (gc.get("coordinateSystem") or meta.get("coordinateSystem") or "").lower()
    off = (-72.0, -72.0) if ("cartesian" in system or "bottom" in system) else (0.0, 0.0)
    pts = [(x * scale + off[0], y * scale + off[1], s) for x, y, s in pts]
    # speeds: LemLib's byte-voltage format is 0-127; some formats use m/s or in/s
    top = max(s for _, _, s in pts)
    if top > 127:
        pts = [(x, y, 127.0 * s / top) for x, y, s in pts]
    name = None
    for p in meta.get("paths", []) or []:
        name = p.get("name") or name
    return {"points": pts, "format": meta.get("format"), "name": name, "scale": scale}


def waypoints(pts, spacing: float):
    """Every `spacing` inches of arc length, plus the last point; each with the
    fastest speed the path asks for since the previous waypoint."""
    out = [pts[0]]
    run, peak = 0.0, 0.0
    for a, b in zip(pts, pts[1:]):
        run += math.hypot(b[0] - a[0], b[1] - a[1])
        peak = max(peak, b[2])
        if run >= spacing:
            out.append((b[0], b[1], peak))
            run, peak = 0.0, 0.0
    if out[-1][:2] != pts[-1][:2]:
        out.append((pts[-1][0], pts[-1][1], max(peak, pts[-1][2])))
    return out


def heading(a, b, reverse=False) -> float:
    h = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1]))     # compass: clockwise from +y
    if reverse:
        h += 180
    return (h + 180) % 360 - 180


def to_auton(parsed: dict, name: str, source: str, reverse: bool = False, spacing: float = 8.0) -> str:
    pts = parsed["points"]
    wps = waypoints(pts, spacing)
    look = next((p for p in pts[1:] if math.hypot(p[0] - pts[0][0], p[1] - pts[0][1]) > 1.0), pts[-1])
    h0 = heading(pts[0], look, reverse)
    f = lambda v: f"{v:.2f}".rstrip("0").rstrip(".")   # noqa: E731
    lines = [f"# @name {name}",
             f"# imported from {source} by tools/path_import.py ({len(pts)} path points -> {len(wps) - 1} moves)",
             "# path.jerryio has no mechanisms: add action(\"...\") lines where they should run",
             f"# @field_start {f(pts[0][0])} {f(pts[0][1])} {f(h0)}",
             "",
             f"odom_xyt_set({f(pts[0][0])}, {f(pts[0][1])}, {f(h0)})"]
    d = "rev" if reverse else "fwd"
    for i, (x, y, s) in enumerate(wps[1:], start=1):
        speed = max(20, min(127, round(s)))
        lines.append(f"pid_odom_set({f(x)}, {f(y)}, {speed}, {d})")
        lines.append("pid_wait()" if i == len(wps) - 1 else "pid_wait_quick_chain()")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", help="path.jerryio export (LemLib format .txt)")
    ap.add_argument("-o", "--out", help="output .auton (default: autons/<name>.auton)")
    ap.add_argument("--name", help="routine name (default: the file name)")
    ap.add_argument("--reverse", action="store_true", help="the robot drives the path backwards")
    ap.add_argument("--spacing", type=float, default=8.0, help="inches between waypoints (default 8)")
    args = ap.parse_args(argv)
    src = pathlib.Path(args.path)
    try:
        parsed = parse(src.read_text(errors="replace"))
    except (OSError, ValueError) as e:
        print(f"{src}: {e}", file=sys.stderr)
        return 1
    name = args.name or re.sub(r"\W+", "_", parsed.get("name") or src.stem).strip("_") or "path"
    out = pathlib.Path(args.out) if args.out else pathlib.Path(__file__).resolve().parent.parent / "autons" / f"{name}.auton"
    text = to_auton(parsed, name, src.name, args.reverse, args.spacing)
    out.write_text(text)
    n = text.count("pid_odom_set")
    print(f"wrote {out}: {n} moves from {len(parsed['points'])} points"
          f"{'' if parsed['scale'] == 1 else ' (converted from cm)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
