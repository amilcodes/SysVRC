"""Flat-shaded orthographic views of the robot, for people and for the model.

Pillow only (no GPU, no display), painter's algorithm, so it runs anywhere the
rest of the studio does. Parts are coloured by what they are, and the parts the
analysis cares about (motors, pistons, flex wheels, sensors) get numbered tags
that match the parts table, so a model looking at the picture can say "#12 is
the intake motor".
"""
from __future__ import annotations

import io

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .cad import Model
from .frame import Frame
from .parts import Component

BG = (15, 17, 20)
INK = (231, 234, 238)
MUTED = (108, 116, 128)
COLORS = {
    "motor": (216, 66, 59), "wheel": (70, 74, 80), "tracking_wheel": (110, 110, 120), "flex_wheel": (31, 200, 240),
    "gear": (245, 184, 0), "sprocket": (245, 184, 0), "chain": (31, 160, 200), "pneumatic": (47, 127, 216),
    "air_tank": (47, 127, 216), "solenoid": (47, 127, 216), "brain": (30, 32, 36), "battery": (30, 32, 36),
    "radio": (30, 32, 36), "sensor": (170, 110, 230), "structure": (160, 166, 175), "other": (128, 134, 143),
}
TAGGED = ("motor", "pneumatic", "flex_wheel", "sensor", "tracking_wheel")

# view: (screen-x axis, screen-y axis (down), depth axis toward the camera), in robot coords
VIEWS = {
    "top": (np.array([1, 0, 0]), np.array([0, -1, 0]), np.array([0, 0, 1])),
    "front": (np.array([-1, 0, 0]), np.array([0, 0, -1]), np.array([0, 1, 0])),
    "side": (np.array([0, 1, 0]), np.array([0, 0, -1]), np.array([1, 0, 0])),
}


def _iso():
    az, el = np.radians(35), np.radians(28)
    d = np.array([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])   # toward camera (front-right, up)
    right = np.cross([0, 0, 1], d)
    right /= np.linalg.norm(right)
    up = np.cross(d, right)
    return right, -up, d


VIEWS["iso"] = _iso()


def _font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def gather(model: Model, comps: list[Component], fr: Frame, cap: int = 350_000):
    """All triangles in the robot frame with a colour each, biggest first if
    there are too many to draw (a fully detailed robot can be millions)."""
    kind_of = {}
    for c in comps:
        for l in c.leaves:
            kind_of[l] = c.kind
    tris, cols = [], []
    for leaf in model.leaves():
        if leaf.vertices is None or not len(leaf.faces):
            continue
        v = fr.to_robot(leaf.vertices)
        t = v[leaf.faces]
        tris.append(t)
        cols.append(np.repeat([COLORS.get(kind_of.get(leaf.id, "other"), COLORS["other"])], len(t), axis=0))
    if not tris:
        return np.zeros((0, 3, 3)), np.zeros((0, 3))
    T = np.concatenate(tris)
    C = np.concatenate(cols)
    if len(T) > cap:
        area = np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
        keep = np.argsort(-area)[:cap]
        T, C = T[keep], C[keep]
    return T, C


def render_view(T, C, view: str, size: int = 900, tags=None, title=None) -> Image.Image:
    sx, sy, sd = VIEWS[view]
    img = Image.new("RGB", (size, size), BG)
    if not len(T):
        return img
    P = np.stack([T @ sx, T @ sy, T @ sd], axis=-1)          # (n, 3 verts, 3)
    lo = P[..., :2].reshape(-1, 2).min(0)
    hi = P[..., :2].reshape(-1, 2).max(0)
    margin = 70
    scale = (size - 2 * margin) / max(hi - lo)
    off = (size - (hi - lo) * scale) / 2 - lo * scale
    xy = P[..., :2] * scale + off
    depth = P[..., 2].mean(axis=1)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nn = np.linalg.norm(n, axis=1)
    nn[nn == 0] = 1
    light = np.abs((n / nn[:, None]) @ (sd * 0.8 + np.array([0.25, 0.3, 0.45]) * 0.6))
    shade = np.clip(0.42 + 0.58 * light, 0, 1)
    draw = ImageDraw.Draw(img)
    for i in np.argsort(depth):                                # far first
        c = tuple(int(v) for v in C[i] * shade[i])
        draw.polygon([tuple(p) for p in xy[i]], fill=c)

    f = _font(15)
    # scale bar: 6 in
    bar = 6 * scale
    draw.line([(margin, size - 30), (margin + bar, size - 30)], fill=INK, width=3)
    draw.text((margin + bar + 8, size - 39), "6 in", fill=INK, font=f)
    draw.text((margin, 18), title or view, fill=INK, font=_font(18))
    if view == "top":
        draw.text((size / 2 - 22, 18), "FRONT", fill=(245, 184, 0), font=_font(18))
        draw.polygon([(size / 2, 44), (size / 2 - 9, 58), (size / 2 + 9, 58)], fill=(245, 184, 0))
    for label, p in (tags or []):
        q = np.array([p @ sx, p @ sy]) * scale + off
        r = 13
        draw.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], fill=(15, 17, 20), outline=INK, width=2)
        draw.text((q[0], q[1]), str(label), fill=INK, font=_font(14), anchor="mm")
    return img


def tags_for(comps: list[Component], fr: Frame, index: dict[str, int]):
    out = []
    for c in comps:
        if c.kind in TAGGED and c.id in index:
            out.append((index[c.id], fr.to_robot(c.center)[0]))
    return out


def render_all(model: Model, comps: list[Component], fr: Frame, index: dict[str, int], size: int = 900):
    T, C = gather(model, comps, fr)
    tags = tags_for(comps, fr, index)
    return {v: render_view(T, C, v, size, tags=tags) for v in ("top", "front", "side", "iso")}


def contact_sheet(views: dict, size: int = 900) -> Image.Image:
    sheet = Image.new("RGB", (size * 2, size * 2), BG)
    for i, k in enumerate(("top", "iso", "front", "side")):
        if k in views:
            sheet.paste(views[k], ((i % 2) * size, (i // 2) * size))
    return sheet


def png_bytes(img: Image.Image, max_side: int = 1400) -> bytes:
    if max(img.size) > max_side:
        img = img.copy()
        img.thumbnail((max_side, max_side))
    b = io.BytesIO()
    img.save(b, "PNG", optimize=True)
    return b.getvalue()
