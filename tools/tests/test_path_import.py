"""tools/path_import.py: a path.jerryio export becomes a routine auton_check runs."""
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import path_import  # noqa: E402


def lemlib_export(points, fmt="LemLib v0.5.0 (inch, byte-voltage)", system="VEX Gaming Positioning System"):
    body = "\n".join(f"{x:.3f}, {y:.3f}, {s}" for x, y, s in points)
    meta = ('{"appVersion":"0.10.0","format":"%s","gc":{"robotWidth":30,"robotHeight":30,"uol":2.54,'
            '"coordinateSystem":"%s"},"paths":[{"name":"Ring Run","segments":[]}]}' % (fmt, system))
    return f"{body}\nendData\n200\n0\n#PATH.JERRYIO-DATA {meta}\n"


# a quarter circle from (-60, -24) sweeping to (-24, 12), plus a straight run, at 1 in steps
ARC = [(-24 - 36 * math.cos(a / 100 * math.pi / 2), -24 + 36 * math.sin(a / 100 * math.pi / 2), 110) for a in range(101)]
ARC = [(x, y, s) for x, y, s in ARC if math.hypot(x + 24, y + 24) > 0] + [(-24 + i, 12, 90) for i in range(1, 25)]


class PathImport(unittest.TestCase):
    def test_reads_points_and_metadata(self):
        p = path_import.parse(lemlib_export(ARC))
        self.assertEqual(len(p["points"]), len(ARC))
        self.assertEqual(p["name"], "Ring Run")
        self.assertEqual(p["scale"], 1.0)
        self.assertAlmostEqual(p["points"][0][0], -60.0, places=2)

    def test_centimetres_are_converted(self):
        cm = [(x * 2.54, y * 2.54, s) for x, y, s in ARC]
        p = path_import.parse(lemlib_export(cm, fmt="LemLib v0.4.x (cm, byte-voltage)"))
        self.assertAlmostEqual(p["points"][0][0], -60.0, places=2)

    def test_corner_origin_coordinates_are_moved_to_the_centre(self):
        shifted = [(x + 72, y + 72, s) for x, y, s in ARC]
        p = path_import.parse(lemlib_export(shifted, system="Cartesian Bottom-Left"))
        self.assertAlmostEqual(p["points"][0][1], -24.0, places=2)

    def test_waypoints_are_spaced_and_end_on_the_end(self):
        p = path_import.parse(lemlib_export(ARC))
        wps = path_import.waypoints(p["points"], 8.0)
        gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(wps, wps[1:])]
        self.assertTrue(all(g <= 9.0 for g in gaps[:-1]))   # spacing plus one sample step
        self.assertEqual(wps[-1][:2], p["points"][-1][:2])
        self.assertEqual(wps[1][2], 110)

    def test_heading_faces_along_the_path(self):
        self.assertAlmostEqual(path_import.heading((0, 0), (0, 10)), 0.0)
        self.assertAlmostEqual(path_import.heading((0, 0), (10, 0)), 90.0)
        self.assertAlmostEqual(path_import.heading((0, 0), (10, 0), reverse=True), -90.0)

    def test_no_points_is_an_error(self):
        with self.assertRaises(ValueError):
            path_import.parse("endData\n#PATH.JERRYIO-DATA {}\n")

    def test_auton_check_drives_it(self):
        """End to end: the .auton parses and the robot ends near the path's end."""
        exe = None
        for p in (ROOT / "build/core/auton_check", ROOT / "build/studio/auton_check"):
            if p.exists():
                exe = p
        if exe is None:
            self.skipTest("auton_check isn't built")
        with tempfile.TemporaryDirectory() as d:
            src = pathlib.Path(d) / "ring_run.txt"
            src.write_text(lemlib_export(ARC))
            out = pathlib.Path(d) / "ring_run.auton"
            self.assertEqual(path_import.main([str(src), "-o", str(out)]), 0)
            text = out.read_text()
            self.assertIn("# @field_start -60 -24", text)
            self.assertTrue(text.rstrip().endswith("pid_wait()"))
            js = pathlib.Path(d) / "r.json"
            r = subprocess.run([str(exe), str(out), "--runs", "20", "--json", str(js)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            import json
            data = json.loads(js.read_text())
            end = data["path"][-1]
            self.assertLess(math.hypot(end[0] - 0, end[1] - 12), 3.0, end)


if __name__ == "__main__":
    unittest.main()
