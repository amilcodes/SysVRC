"""python3 tools/robot_studio serve | analyze

  serve                     the web studio (default http://localhost:8090)
  analyze CAD [--ai]        CAD + code -> robots/<name>/robot.json, no browser
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from robot_studio import analyze, replica, server, spec  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main(argv=None):
    ap = argparse.ArgumentParser(prog="robot_studio", description="CAD + code -> a robot the sim can run")
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--code", action="append", default=None,
                        help="team code to read (repeatable; default v5/src and v5/include)")
    common.add_argument("--autons", default=str(ROOT / "autons"), help="folder of .auton files (default autons/)")
    s = sub.add_parser("serve", parents=[common], help="run the web studio")
    s.add_argument("--port", type=int, default=8090)
    s.add_argument("--robots", default=str(ROOT / "robots"))
    a = sub.add_parser("analyze", parents=[common], help="analyse a CAD export without the browser")
    a.add_argument("cad", nargs="?", help="STEP/STL/OBJ/GLB export (leave out to work from code only)")
    a.add_argument("--name", help="robot name (default: the file name)")
    a.add_argument("--out", help="where to write (default robots/<name>/)")
    a.add_argument("--ai", action="store_true", help="also ask Claude (needs ANTHROPIC_API_KEY)")
    a.add_argument("--up", help="override the up axis, e.g. +z")
    a.add_argument("--front", help="override the front, e.g. -x")
    args = ap.parse_args(argv)
    code = args.code if args.code is not None else [str(p) for p in (ROOT / "v5/src", ROOT / "v5/include") if p.exists()]

    if args.cmd == "serve":
        server.serve(args.port, code, args.autons, args.robots)
        return 0

    an = analyze.gather(args.cad, code, args.autons, up=args.up, forward=args.front)
    name = args.name or (pathlib.Path(args.cad).stem if args.cad else "robot")
    robot = analyze.heuristic(an, name)
    if an.model:
        print(f"{pathlib.Path(args.cad).name}: {an.model.triangle_count()} triangles, "
              f"{'named parts' if an.model.named else 'no part names (mesh only)'}")
        for w in an.frame.why:
            print("  " + w)
    if args.ai:
        robot, raw = analyze.claude(an, robot)
        u = raw.get("usage") or {}
        print(f"Claude ({raw.get('model')}): {u.get('input_tokens', '?')} in / {u.get('output_tokens', '?')} out tokens")
    errs, warns = spec.validate(robot)
    d = robot["drive"]
    print(f"drive: {d.get('type')} {d.get('wheel_diameter')} in @ {d.get('wheel_rpm')} rpm, track {d.get('track_width')} in, "
          f"{d.get('motors_per_side', '?')} motors/side")
    for m in robot["mechanisms"]:
        print(f"  {m['kind']:15} {m['id']:14} {', '.join(m.get('code_names') or [])}")
    for w in warns:
        print("warning: " + w)
    for q in robot.get("questions", []):
        print("check: " + q)
    if errs:
        print("not saved: " + "; ".join(errs))
        return 1
    out = args.out or str(ROOT / "robots" / replica.slug(name))
    for f in replica.save(an, robot, out):
        print("wrote", f.relative_to(ROOT) if f.is_relative_to(ROOT) else f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
