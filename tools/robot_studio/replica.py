"""Write a robot out: robots/<name>/robot.json plus what the viewer and docs use.

  robot.json   the spec (auton_check --robot, the report's game sim)
  model.glb    the CAD in the robot frame, metres, one mesh per part named by
               part id (the studio viewer colours and labels parts from that)
  views.png    the four renders the analysis looked at
"""
from __future__ import annotations

import json
import pathlib
import re

from . import cad, render, spec as specmod


def slug(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", (name or "robot").lower()).strip("-")
    return s or "robot"


def save(an, robot: dict, out_dir: str | pathlib.Path) -> list[pathlib.Path]:
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    robot = specmod.normalize(robot)
    errors, _ = specmod.validate(robot)
    if errors:
        raise ValueError("; ".join(errors))
    src = dict(robot.get("source") or {})
    if an.model:
        src["cad"] = pathlib.Path(an.model.source).name
        src["frame"] = an.frame.describe()
    if an.code.files:
        src["code"] = sorted({pathlib.Path(f).name for f in an.code.files})
    robot["source"] = src
    written = []
    p = out / "robot.json"
    p.write_text(json.dumps(robot, indent=2) + "\n")
    written.append(p)
    if an.model:
        p = out / "model.glb"
        cad.export_glb(an.model, p, transform=an.frame.matrix)
        written.append(p)
        if an.views:
            p = out / "views.png"
            render.contact_sheet(an.views, size=next(iter(an.views.values())).size[0]).save(p)
            written.append(p)
    return written


def part_map(an) -> dict:
    """leaf id -> {kind, name, tag} for the viewer."""
    out = {}
    for c in an.comps:
        for leaf in c.leaves:
            out[leaf] = {"kind": c.kind, "name": c.name, "tag": an.index.get(c.id)}
    return out
