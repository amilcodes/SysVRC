"""Robot studio, end to end on a fixture robot whose parts are known.

    pip install -r tools/robot_studio/requirements.txt
    python3 -m unittest discover -s tools/robot_studio/tests

The fixture (tests/fixtures/hs_bot.step, made by make_fixture.py) is a 6-motor
3.25 in drive geared 36:48, track 12 in, wheelbase 10.2 in, intake at +x,
clamp at -x, lady brown on two green motors at 1:3. hs_bot.stl is the same
robot with no names at all.
"""
import json
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = HERE.parent.parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

from robot_studio import analyze, cad, codescan, frame, parts, render, replica, spec  # noqa: E402

STEP = HERE / "fixtures" / "hs_bot.step"
STL = HERE / "fixtures" / "hs_bot.stl"


def _rotated(model: cad.Model, R: np.ndarray) -> cad.Model:
    for n in model.nodes.values():
        if n.vertices is not None:
            n.vertices = n.vertices @ R.T
        for c in n.cylinders:
            c.axis = R @ c.axis
            c.center = R @ c.center
    return model


class Load(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = cad.load(STEP)

    def test_names_survive_including_subassemblies(self):
        names = {n.name for n in self.m.nodes.values()}
        self.assertIn("3.25in Omni Wheel <1>", names)
        self.assertIn("Drive Left", names)                    # an unnamed instance, named from the STEP product
        self.assertIn("V5 Smart Motor (11W) <1>", names)
        self.assertTrue(self.m.named)

    def test_units_are_inches(self):
        size = np.ptp(self.m.all_vertices(), axis=0)
        self.assertAlmostEqual(float(size.max()), 17.54, delta=0.1)

    def test_cylinders_give_wheel_and_gear_radii(self):
        wheel = next(n for n in self.m.nodes.values() if n.name == "3.25in Omni Wheel <1>")
        self.assertAlmostEqual(max(c.radius for c in wheel.cylinders), 1.625, places=3)
        gear = next(n for n in self.m.nodes.values() if n.name.startswith("48T"))
        self.assertAlmostEqual(max(c.radius for c in gear.cylinders), 1.0, places=3)

    def test_an_stl_is_split_back_into_bodies(self):
        s = cad.load(STL)
        self.assertFalse(s.named)
        self.assertGreater(len(list(s.leaves())), 40)

    def test_unsupported_format_says_so(self):
        with tempfile.NamedTemporaryFile(suffix=".sldprt") as f:
            with self.assertRaises(ValueError):
                cad.load(f.name)


class Parts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = cad.load(STEP)
        cls.comps = parts.classify(cls.m)

    def test_counts(self):
        s = parts.summary(self.comps)
        self.assertEqual(s["motor"], 9)
        self.assertEqual(s["wheel"], 6)
        self.assertEqual(s["flex_wheel"], 3)
        self.assertEqual(s["pneumatic"], 3)
        self.assertEqual(s["gear"], 16)

    def test_motor_cartridge_from_its_child_part(self):
        carts = sorted(c.attrs.get("cartridge_rpm") for c in self.comps if c.kind == "motor")
        self.assertEqual(carts, [200, 200] + [600] * 7)

    def test_wheel_and_gear_details(self):
        wheels = [c for c in self.comps if c.kind == "wheel"]
        self.assertTrue(all(w.attrs["diameter"] == 3.25 for w in wheels))
        self.assertEqual(sorted({w.attrs["type"] for w in wheels}), ["omni", "traction"])
        teeth = sorted({c.attrs["teeth"] for c in self.comps if c.kind == "gear"})
        self.assertEqual(teeth, [12, 36, 48])

    def test_unnamed_wheels_are_found_by_shape(self):
        s = cad.load(STL)
        wheels = [c for c in parts.classify(s) if c.kind == "wheel"]
        self.assertEqual(len(wheels), 6)
        self.assertTrue(all(w.attrs["diameter"] == 3.25 and w.by == "shape" for w in wheels))

    def test_names_from_other_libraries(self):
        for name, kind, attrs in [("276-4840 V5 Smart Motor", "motor", {"watts": 11}),
                                  ("EXP Smart Motor 5.5W", "motor", {"watts": 5.5}),
                                  ("4in Omni-Directional Wheel", "wheel", {"diameter": 4.0}),
                                  ("60 Tooth Gear", "gear", {"teeth": 60}),
                                  ("Air Tank 200 mL", "air_tank", {}),
                                  ("Motor Cartridge (Red) 100 RPM", "motor", {"cartridge_rpm": 100})]:
            k, a = parts.classify_name(name)
            self.assertEqual(k, kind, name)
            for key, v in attrs.items():
                self.assertEqual(a.get(key), v, name)


class Frame(unittest.TestCase):
    def check(self, m, up, front):
        comps = parts.classify(m)
        fr = frame.infer(m, comps)
        ms = frame.measure(m, comps, fr)
        d = fr.describe()
        self.assertEqual((d["up"], d["forward"]), (up, front))
        self.assertAlmostEqual(ms.drive["track_width"], 11.96, delta=0.1)
        self.assertAlmostEqual(ms.drive["wheelbase"], 10.24, delta=0.1)
        self.assertAlmostEqual(ms.footprint["length"], 17.54, delta=0.1)
        self.assertAlmostEqual(ms.footprint["width"], 13.27, delta=0.1)
        return ms

    def test_z_up_x_forward(self):
        ms = self.check(cad.load(STEP), "+z", "+x")
        self.assertEqual(ms.drive["motors_per_side"], 3)
        self.assertEqual(ms.drive["ratio"], 0.75)
        self.assertEqual(ms.drive["wheel_rpm"], 450)
        self.assertEqual(ms.drive["type"], "tank")

    def test_same_robot_exported_y_up(self):
        R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        self.check(_rotated(cad.load(STEP), R), "-y", "+z")

    def test_origin_is_on_the_floor_under_the_drive(self):
        m = cad.load(STEP)
        comps = parts.classify(m)
        fr = frame.infer(m, comps)
        pts = fr.to_robot(m.all_vertices())
        self.assertAlmostEqual(float(pts[:, 2].min()), 0.0, delta=0.05)
        wheels = [fr.to_robot(c.center)[0] for c in comps if c.kind == "wheel"]
        self.assertAlmostEqual(float(np.mean([w[0] for w in wheels])), 0.0, delta=0.05)
        self.assertAlmostEqual(float(np.mean([w[1] for w in wheels])), 0.0, delta=0.05)

    def test_user_can_overrule_the_front(self):
        m = cad.load(STEP)
        fr = frame.infer(m, parts.classify(m), forward="-x")
        self.assertEqual(fr.describe()["forward"], "-x")
        self.assertEqual(fr.describe()["right"], "+y")


class Render(unittest.TestCase):
    def test_views_have_the_robot_in_them(self):
        an = analyze.gather(str(STEP), [], None, render_size=300)
        self.assertEqual(sorted(an.views), ["front", "iso", "side", "top"])
        for img in an.views.values():
            px = np.asarray(img).reshape(-1, 3)
            lit = np.mean(np.any(px != render.BG, axis=1))
            self.assertGreater(lit, 0.08)
        self.assertGreater(len(render.png_bytes(render.contact_sheet(an.views, 300))), 10_000)


CPP = r"""
// EZ-Template chassis and some mechanisms
ez::Drive chassis({-11, -12, 13}, {1, 2, -3}, 21, 2.75, 600);
pros::MotorGroup left_mg({-11, -12, 13}, pros::v5::MotorGears::blue);
pros::Motor intake(-5, pros::v5::MotorGears::blue);
pros::Motor arm(8, pros::v5::MotorGears::green);
pros::adi::Pneumatics clamp('A', false);
pros::Optical colour(9);
const double LOAD = 12 * 2;
double SCORE = LOAD + 150;
void spin(int v) { intake.move(v); }
void grab() { clamp.toggle(); }
"""
LEMLIB = r"""
lemlib::Drivetrain drivetrain(&left, &right, 11.5, lemlib::Omniwheel::NEW_325, 450, 2);
"""


class Code(unittest.TestCase):
    def facts(self, text, name="robot.cpp"):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / name
            p.write_text(text)
            a = pathlib.Path(d) / "autons"
            a.mkdir()
            (a / "x.auton").write_text('action("spin(127)")  @1\naction("grab()")  @2\naction("arm.move(50)")  @3\n')
            return codescan.scan([p], a)

    def test_ez_chassis_and_declarations(self):
        f = self.facts(CPP)
        self.assertEqual(f.chassis["lib"], "EZ-Template")
        self.assertEqual((f.chassis["wheel_diameter"], f.chassis["wheel_rpm"]), (2.75, 600))
        kinds = {d.name: (d.kind, d.cartridge_rpm) for d in f.decls}
        self.assertEqual(kinds["intake"], ("motor", 600))
        self.assertEqual(kinds["arm"], ("motor", 200))
        self.assertEqual(kinds["clamp"][0], "piston")
        self.assertEqual(kinds["colour"][0], "optical")
        self.assertEqual(f.constants["SCORE"], 174)
        self.assertEqual(f.actions, ["spin(127)", "grab()", "arm.move(50)"])
        self.assertIn("intake.move", f.helpers["spin"]["calls"])

    def test_lemlib_gives_track_width(self):
        f = self.facts(LEMLIB)
        self.assertEqual(f.chassis["lib"], "LemLib")
        self.assertEqual((f.chassis["track_width"], f.chassis["wheel_diameter"], f.chassis["wheel_rpm"]), (11.5, 3.25, 450))

    def test_reads_our_robot(self):
        f = codescan.scan([ROOT / "v5/src"], ROOT / "autons")
        self.assertEqual(f.chassis["wheel_rpm"], 450)
        self.assertIn("mogoClamp", [d.name for d in f.decls])
        self.assertEqual(f.constants.get("STOP1"), 27)


class Draft(unittest.TestCase):
    def test_from_code_alone(self):
        an = analyze.gather(None, [ROOT / "v5/src"], ROOT / "autons")
        r = analyze.heuristic(an, "ours")
        self.assertEqual(spec.validate(r)[0], [])
        kinds = {m["id"]: m["kind"] for m in r["mechanisms"]}
        self.assertEqual(kinds["intake"], "intake")
        self.assertEqual(kinds["clamp"], "goal_clamp")
        self.assertEqual(kinds["arm"], "wall_stake_arm")
        self.assertEqual(kinds["intakelift"], "lift")
        self.assertIn({"match": r"^setIntake\((.+)\)$", "mech": "intake", "do": "speed", "value": "$1", "scale": 127},
                      r["bindings"])
        self.assertEqual(r["drive"]["wheel_rpm"], 450)

    def test_with_cad_the_zones_come_from_the_robot(self):
        an = analyze.gather(str(STEP), [ROOT / "v5/src"], ROOT / "autons", render_size=300)
        r = analyze.heuristic(an, "fixture")
        intake = next(m for m in r["mechanisms"] if m["kind"] == "intake")
        clamp = next(m for m in r["mechanisms"] if m["kind"] == "goal_clamp")
        self.assertGreater(intake["zone"][0][1], 4)          # front
        self.assertLess(clamp["zone"][1][1], 0)              # back
        self.assertEqual(r["drive"]["track_width"], 11.96)
        self.assertEqual(len(r["outline"]) >= 4, True)


class Claude(unittest.TestCase):
    """The request we'd send, and what we do with an answer, without the network."""

    def test_round_trip(self):
        an = analyze.gather(str(STEP), [ROOT / "v5/src"], ROOT / "autons", render_size=300)
        draft = analyze.heuristic(an, "fixture")
        seen = {}

        def fake_post(body):
            seen["body"] = body
            ai = {**draft, "mechanisms": draft["mechanisms"] + [], "bindings": draft["bindings"] + [
                {"match": r"^ChangeLBState\((\w+)\)$", "mech": "arm", "do": "state", "value": "$1"}],
                "questions": ["Is the clamp at the back?"]}
            arm = next(m for m in ai["mechanisms"] if m["id"] == "arm")
            arm.update({"states": {"REST": 0, "PROPPED": 27, "EXTENDED": 177}, "load_state": "PROPPED"})
            return {"model": "claude-test", "content": [{"type": "tool_use", "name": "submit_robot", "input": ai}],
                    "usage": {"input_tokens": 1, "output_tokens": 1}}

        robot, raw = analyze.claude(an, draft, post=fake_post)
        body = seen["body"]
        self.assertEqual(body["tool_choice"], {"type": "tool", "name": "submit_robot"})
        self.assertEqual(body["tools"][0]["input_schema"], spec.SCHEMA)
        content = body["messages"][0]["content"]
        self.assertEqual(content[0]["type"], "image")
        facts = json.loads(content[1]["text"].split("\n\n", 1)[1])
        self.assertEqual(facts["code"]["chassis"]["wheel_rpm"], 450)
        self.assertIn("setIntake", facts["code"]["helpers"])
        self.assertTrue(any(p["kind"] == "motor" for p in facts["cad"]["parts"]))
        self.assertEqual(robot["source"]["analyzer"], "claude-test")
        self.assertEqual(spec.validate(robot)[0], [])
        self.assertIn("ChangeLBState(EXTENDED)", [a for a in an.code.actions if a.startswith("ChangeLBState")])
        self.assertNotIn("ChangeLBState(EXTENDED)", spec.unbound(robot, an.code.actions))

    def test_no_answer_is_an_error(self):
        an = analyze.gather(None, [], None)
        with self.assertRaises(RuntimeError):
            analyze.claude(an, analyze.heuristic(an), post=lambda body: {"content": [{"type": "text", "text": "no"}]})


class Spec(unittest.TestCase):
    def test_catches_what_would_break_the_sim(self):
        bad = {"footprint": {"width": 12, "length": 15}, "drive": {"wheel_diameter": 3.25, "track_width": 14},
               "mechanisms": [{"id": "a", "kind": "intake"}, {"id": "a", "kind": "laser"}],
               "bindings": [{"match": "([", "mech": "a", "do": "speed"}, {"match": "x", "mech": "nope", "do": "fly"}]}
        errs, _ = spec.validate(spec.normalize(bad))
        text = " | ".join(errs)
        for want in ("wider than the robot", "two mechanisms called a", "unknown kind laser", "not a valid regex",
                     "unknown mechanism 'nope'", "unknown op 'fly'"):
            self.assertIn(want, text)

    def test_ratio_and_rpm_fill_each_other(self):
        s = spec.normalize({"drive": {"cartridge_rpm": 600, "ratio": 0.8}})
        self.assertEqual(s["drive"]["wheel_rpm"], 480)
        s = spec.normalize({"drive": {"cartridge_rpm": 600, "wheel_rpm": 450}})
        self.assertEqual(s["drive"]["ratio"], 0.75)

    def test_the_checked_in_robot_is_valid(self):
        r = json.loads((ROOT / "robots/ours/robot.json").read_text())
        self.assertEqual(spec.validate(r)[0], [])


class Save(unittest.TestCase):
    def test_writes_a_replica(self):
        an = analyze.gather(str(STEP), [], None, render_size=200)
        r = analyze.heuristic(an, "Fixture Bot!")
        with tempfile.TemporaryDirectory() as d:
            files = replica.save(an, r, pathlib.Path(d) / replica.slug("Fixture Bot!"))
            self.assertEqual(sorted(f.name for f in files), ["model.glb", "robot.json", "views.png"])
            saved = json.loads(files[0].read_text())
            self.assertEqual(saved["schema"], spec.SCHEMA_ID)
            self.assertEqual(saved["source"]["frame"]["forward"], "+x")
            back = cad.load(files[1])                       # the GLB round-trips, in the robot frame
            lo = back.all_vertices().min(0)
            self.assertAlmostEqual(float(lo[2]), 0.0, delta=0.05)
        self.assertEqual(replica.slug("Fixture Bot!"), "fixture-bot")


if __name__ == "__main__":
    unittest.main()
