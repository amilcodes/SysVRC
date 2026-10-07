"""The studio's local web server: drop a CAD file in, look, ask Claude, fix
what's wrong, save, and run a routine with the new robot.

Everything stays on this machine except the analysis call to the Claude API,
which sends the renders, the parts table and the relevant code.
"""
from __future__ import annotations

import http.server
import json
import os
import pathlib
import shutil
import subprocess
import tempfile
import threading
import traceback
import urllib.parse
import uuid

from . import analyze, cad, replica, spec as specmod

ROOT = pathlib.Path(__file__).resolve().parents[2]          # the repo
WEB = pathlib.Path(__file__).resolve().parent / "web"
MAX_UPLOAD = 400 * 1024 * 1024


class Studio:
    def __init__(self, code_paths, autons_dir, robots_dir):
        self.code_paths = [str(p) for p in code_paths]
        self.autons_dir = str(autons_dir) if autons_dir else None
        self.robots_dir = pathlib.Path(robots_dir)
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix="robot-studio-"))
        self.sessions: dict[str, dict] = {}
        self.lock = threading.Lock()

    # -------------------------------------------------------------- sessions
    def load(self, filename: str | None, data: bytes | None, up=None, forward=None, sid=None) -> dict:
        sid = sid or uuid.uuid4().hex[:10]
        d = self.tmp / sid
        d.mkdir(exist_ok=True)
        cad_path = None
        if data is not None and filename:
            ext = pathlib.Path(filename).suffix.lower()
            if ext not in cad.SUPPORTED:
                raise ValueError(f"{filename}: use one of {', '.join(cad.SUPPORTED)}")
            cad_path = d / ("upload" + ext)
            cad_path.write_bytes(data)
        elif sid in self.sessions:
            cad_path = self.sessions[sid].get("cad_path")
        an = analyze.gather(str(cad_path) if cad_path else None, self.code_paths, self.autons_dir, up=up, forward=forward)
        name = pathlib.Path(filename).stem if filename else (self.sessions.get(sid, {}).get("name") or "robot")
        draft = analyze.heuristic(an, name)
        if an.model:
            for k, img in an.views.items():
                img.save(d / f"{k}.png")
            cad.export_glb(an.model, d / "model.glb", transform=an.frame.matrix)
        with self.lock:
            self.sessions[sid] = {"an": an, "cad_path": cad_path, "name": name, "draft": draft}
        return self.describe(sid)

    def describe(self, sid: str) -> dict:
        s = self.sessions[sid]
        an = s["an"]
        out = {
            "session": sid, "name": s["name"], "draft": s["draft"], "spec": s.get("spec") or s["draft"],
            "ai": bool(os.environ.get("ANTHROPIC_API_KEY")), "model": analyze.DEFAULT_MODEL,
            "code": {"files": [pathlib.Path(f).name for f in an.code.files], "chassis": an.code.chassis,
                     "decls": [d.__dict__ for d in an.code.decls], "actions": an.code.actions},
        }
        if an.model:
            out["cad"] = {
                "file": s["name"], "format": an.model.format, "named": an.model.named,
                "triangles": an.model.triangle_count(), "frame": an.frame.describe(), "why": an.frame.why,
                "measures": {"footprint": an.measures.footprint, "drive": an.measures.drive, "wheels": an.measures.wheels,
                             "notes": an.measures.notes},
                "summary": {k: v for k, v in sorted(_count(an.comps).items())},
                "parts": an.parts_table(), "map": replica.part_map(an),
                "views": [f"/s/{sid}/{k}.png" for k in ("top", "iso", "front", "side")],
                "glb": f"/s/{sid}/model.glb",
            }
        return out

    def analyze(self, sid: str) -> dict:
        s = self.sessions[sid]
        robot, raw = analyze.claude(s["an"], s["draft"])
        s["spec"] = robot
        errs, warns = specmod.validate(robot)
        return {"spec": robot, "errors": errs, "warnings": warns, "usage": raw.get("usage"), "model": raw.get("model"),
                "unbound": specmod.unbound(robot, s["an"].code.actions)}

    def save(self, sid: str, robot: dict, name: str) -> dict:
        s = self.sessions[sid]
        out = self.robots_dir / replica.slug(name or robot.get("name"))
        robot["name"] = name or robot.get("name")
        files = replica.save(s["an"], robot, out)
        s["spec"] = robot
        s["saved"] = out
        return {"dir": str(out.relative_to(ROOT) if out.is_relative_to(ROOT) else out),
                "files": [f.name for f in files]}

    def check(self, robot_path: str, routine: str) -> dict:
        exe = find_auton_check()
        rp = (ROOT / robot_path) if not pathlib.Path(robot_path).is_absolute() else pathlib.Path(robot_path)
        auton = ROOT / routine
        name = f"{pathlib.Path(routine).stem}.{uuid.uuid4().hex[:6]}.html"
        out = self.tmp / "reports"
        out.mkdir(exist_ok=True)
        r = subprocess.run([str(exe), str(auton), "--robot", str(rp), "--html", str(out / name)],
                           capture_output=True, text=True, timeout=300)
        if r.returncode != 0:
            raise RuntimeError((r.stderr or r.stdout)[-1500:])
        return {"report": f"/r/{name}", "summary": r.stdout[-2500:]}


def _count(comps):
    out = {}
    for c in comps:
        out[c.kind] = out.get(c.kind, 0) + 1
    return out


def find_auton_check() -> pathlib.Path:
    for p in (ROOT / "build/core/auton_check", ROOT / "build/studio/auton_check"):
        if p.exists():
            return p
    cxx = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise RuntimeError("auton_check isn't built and there's no C++ compiler: run "
                           "cmake -S core -B build/core && cmake --build build/core")
    out = ROOT / "build/studio/auton_check"
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([cxx, "-std=c++17", "-O2", f"-I{ROOT/'core/include'}", f"-I{ROOT/'core/tools'}",
                    str(ROOT / "core/tools/auton_check.cpp"), "-o", str(out)], check=True, timeout=600)
    return out


def make_handler(studio: Studio):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)).encode())

        def _file(self, path: pathlib.Path):
            if not path.is_file():
                return self._send(404, b"not found", "text/plain")
            ctype = {".html": "text/html; charset=utf-8", ".js": "text/javascript", ".png": "image/png",
                     ".glb": "model/gltf-binary", ".json": "application/json", ".css": "text/css"}.get(path.suffix, "application/octet-stream")
            self._send(200, path.read_bytes(), ctype)

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            p = u.path
            if p in ("/", "/index.html"):
                return self._file(WEB / "studio.html")
            if p in ("/field.js", "/game.js"):
                return self._file(ROOT / "web" / p.lstrip("/"))
            if p.startswith("/s/"):
                _, _, sid, fname = p.split("/", 3)
                return self._file(studio.tmp / sid / pathlib.Path(fname).name)
            if p.startswith("/r/"):
                return self._file(studio.tmp / "reports" / pathlib.Path(p[3:]).name)
            if p == "/api/routines":
                d = pathlib.Path(studio.autons_dir) if studio.autons_dir else ROOT / "autons"
                return self._json({"routines": [str(f.relative_to(ROOT)) for f in sorted(d.glob("*.auton"))]})
            if p == "/api/robots":
                return self._json({"robots": [str(f.relative_to(ROOT)) for f in sorted(studio.robots_dir.glob("*/robot.json"))]})
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_UPLOAD:
                return self._json({"error": "file is over 400 MB; export at a coarser tolerance"}, 413)
            body = self.rfile.read(n) if n else b""
            try:
                if u.path == "/api/load":
                    fname = (q.get("name") or [None])[0]
                    return self._json(studio.load(fname, body if fname else None))
                req = json.loads(body or b"{}")
                if u.path == "/api/open":
                    # a CAD file already in the repo (only inside it)
                    f = (ROOT / req["path"]).resolve()
                    if not f.is_relative_to(ROOT) or not f.is_file():
                        raise ValueError(f"no such file in the repo: {req['path']}")
                    return self._json(studio.load(f.name, f.read_bytes()))
                if u.path == "/api/frame":
                    return self._json(studio.load(None, None, up=req.get("up"), forward=req.get("forward"), sid=req["session"]))
                if u.path == "/api/analyze":
                    return self._json(studio.analyze(req["session"]))
                if u.path == "/api/save":
                    return self._json(studio.save(req["session"], req["spec"], req.get("name")))
                if u.path == "/api/check":
                    return self._json(studio.check(req["robot"], req["routine"]))
                if u.path == "/api/validate":
                    errs, warns = specmod.validate(specmod.normalize(req["spec"]))
                    return self._json({"errors": errs, "warnings": warns})
            except Exception as e:     # report it to the page instead of dropping the connection
                traceback.print_exc()
                return self._json({"error": str(e)}, 400)
            self._send(404, b"not found", "text/plain")

    return H


def serve(port: int, code_paths, autons_dir, robots_dir):
    studio = Studio(code_paths, autons_dir, robots_dir)
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), make_handler(studio))
    print(f"robot studio: http://localhost:{port}")
    print(f"  code: {', '.join(studio.code_paths) or '(none)'}   routines: {studio.autons_dir or '(none)'}")
    print("  Claude: " + ("ANTHROPIC_API_KEY set, model " + analyze.DEFAULT_MODEL if os.environ.get("ANTHROPIC_API_KEY")
                         else "no ANTHROPIC_API_KEY, so the analysis is the no-AI draft only"))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        shutil.rmtree(studio.tmp, ignore_errors=True)
