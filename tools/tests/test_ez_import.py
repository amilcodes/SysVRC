"""Tests for tools/ez_import.py.

    python3 -m unittest discover -s tools/tests
"""
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import ez_import  # noqa: E402


def read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def run_import(cpp: str, func: str, **bind) -> ez_import.Result:
    """Import one function from a snippet of C++."""
    toks = ez_import.tokenize(cpp)
    consts = dict(ez_import.EZ_ENUMS)
    ez_import.file_constants(cpp, toks, consts)
    funcs = ez_import.find_functions(toks, "autons.cpp")
    return ez_import.Importer(funcs, consts).run(func, bind)


def lines(res: ez_import.Result) -> list[str]:
    return [l.text for l in res.lines if l.text]


class Basics(unittest.TestCase):
    def test_ez_calls_with_units_and_constants(self):
        res = run_import("""
            const int DRIVE_SPEED = 110;
            #define TURN_SPEED 90
            void a() {
              chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
              chassis.pid_wait();
              chassis.pid_turn_set(90_deg, TURN_SPEED);
              chassis.pid_wait_until(45_deg);
              chassis.pid_wait_quick_chain();
            }""", "a")
        self.assertEqual(lines(res), ["pid_drive_set(24, 110)", "pid_wait()", "pid_turn_set(90, 90)",
                                      "pid_wait_until(45)", "pid_wait_quick_chain()"])
        self.assertEqual(res.warnings, [])

    def test_slew_off_is_kept(self):
        res = run_import("void a() { chassis.pid_drive_set(24, 110, false); }", "a")
        self.assertEqual(lines(res), ["pid_drive_set(24, 110, false)"])

    def test_arithmetic_is_evaluated(self):
        res = run_import("void a() { chassis.pid_drive_set(32 + 4, 127); chassis.pid_wait_until(12 - 6); }", "a")
        self.assertEqual(lines(res), ["pid_drive_set(36, 127)", "pid_wait_until(6)"])

    def test_source_lines_are_recorded(self):
        res = run_import("void a() {\n\n  chassis.pid_wait();\n}", "a")
        self.assertEqual([l.src for l in res.lines if l.text], [3])

    def test_mechanism_calls_become_actions(self):
        res = run_import("void a() { intake.move(-127); mogoClamp.toggle(); ChangeLBState(EXTENDED); }", "a")
        self.assertEqual(lines(res), ['action("intake.move(-127)")', 'action("mogoClamp.toggle()")',
                                      'action("ChangeLBState(EXTENDED)")'])

    def test_first_odom_xyt_set_is_the_start(self):
        res = run_import("void a() { chassis.odom_xyt_set(0, 0, -89); chassis.pid_drive_set(5, 90); "
                         "chassis.odom_xyt_set(1, 2, 3); }", "a")
        self.assertEqual(res.start, (0.0, 0.0, -89.0))


class ColourMirroring(unittest.TestCase):
    CPP = """
        void a(bool isBlue) {
          int sgn = isBlue ? 1 : -1;
          chassis.pid_turn_set(-57 * sgn, 90);
          if (isBlue) { rightDoinker.toggle(); } else { leftDoinker.toggle(); }
          if (!isBlue) chassis.pid_wait();
          chassis.pid_swing_set((!isBlue ? ez::RIGHT_SWING : ez::LEFT_SWING), 45 * sgn, 90, 30);
        }"""

    def test_blue(self):
        res = run_import(self.CPP, "a", isBlue=True)
        self.assertEqual(lines(res), ["pid_turn_set(-57, 90)", 'action("rightDoinker.toggle()")',
                                      "pid_swing_set(left, 45, 90, 30)"])
        self.assertEqual(res.warnings, [])

    def test_red(self):
        res = run_import(self.CPP, "a", isBlue=False)
        self.assertEqual(lines(res), ["pid_turn_set(57, 90)", 'action("leftDoinker.toggle()")', "pid_wait()",
                                      "pid_swing_set(right, -45, 90, 30)"])


class Wrappers(unittest.TestCase):
    def test_set_drive_alias_uses_max_speed(self):
        # set_drive(inches, time, minSpeed, maxSpeed): the THIRD argument is
        # minSpeed, not the speed. Hand transcription got this wrong once.
        res = run_import("void a() { set_drive(-12, 1500, 120); set_drive(10, 2000, 0, 80); set_drive(3); }", "a")
        self.assertEqual(lines(res), ["pid_drive_set(-12, 127)", "pid_drive_set(10, 80)", "pid_drive_set(3, 127)"])

    def test_helpers_are_inlined(self):
        res = run_import("""
            void grab(int n) { chassis.pid_drive_set(n, 90); chassis.pid_wait(); }
            void a() { grab(12); grab(6 * 2); }""", "a")
        self.assertEqual(lines(res), ["pid_drive_set(12, 90)", "pid_wait()"] * 2)


class Loops(unittest.TestCase):
    def test_timed_push(self):
        res = run_import("""
            void a() {
              long start = pros::millis();
              while (pros::millis() - start < 1500 - 500) { chassis.drive_set(100, 100); pros::delay(10); }
            }""", "a")
        self.assertEqual(lines(res), ["drive_set(100, 100)", "delay(1000)"])

    def test_timer_reassigned(self):
        res = run_import("""
            void a() {
              long t = 0;
              t = pros::millis();
              while (pros::millis() - t < 700) { chassis.drive_set(-60, -60); pros::delay(10); }
            }""", "a")
        self.assertEqual(lines(res), ["drive_set(-60, -60)", "delay(700)"])
        self.assertEqual(res.warnings, [])


class HonestAboutGaps(unittest.TestCase):
    def test_sensor_conditions_are_reported_not_guessed_silently(self):
        res = run_import("void a() { if (dist.get() > 10) { chassis.pid_wait(); } }", "a")
        self.assertEqual(len(res.warnings), 1)
        self.assertIn("can't evaluate if-condition", res.warnings[0])
        self.assertIn("NOT IMPORTED", "\n".join(l.note for l in res.lines))

    def test_runtime_arguments_are_reported(self):
        res = run_import("void a() { chassis.pid_drive_set(chassis.odom_x_get() - 3, 127); }", "a")
        self.assertEqual(lines(res), [])
        self.assertEqual(len(res.warnings), 1)

    def test_for_loops_are_reported(self):
        res = run_import("void a() { for (int i = 0; i < 3; i++) { chassis.pid_wait(); } }", "a")
        self.assertIn("'for' loops", res.warnings[0])


class Cli(unittest.TestCase):
    def test_colour_variants_do_not_collide(self):
        # fooBlue() and foo(isBlue=true) used to both become foo_blue.auton.
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "autons.cpp")
            with open(src, "w") as f:
                f.write("void rush(bool isBlue) { chassis.pid_drive_set(1, 90); }\n"
                        "void rushBlue() { chassis.pid_drive_set(2, 90); }\n")
            out = os.path.join(d, "out")
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                ez_import.main([src, "--all", "-o", out])
            self.assertEqual(sorted(os.listdir(out)), ["rush.blue.auton", "rush_blue.auton"])


class RepoIsUpToDate(unittest.TestCase):
    """autons/ must match what the importer makes from v5/src right now, so
    editing an auton without re-importing it fails CI instead of silently
    testing an old copy."""

    def test_committed_autons_match_the_source(self):
        srcs = [os.path.join(ROOT, "v5", "src", f) for f in ("autons.cpp", "skills.cpp")]
        with tempfile.TemporaryDirectory() as d:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                ez_import.main(srcs + ["--all", "-o", d])
            fresh = {f: read(os.path.join(d, f)) for f in os.listdir(d)}
        committed_dir = os.path.join(ROOT, "autons")
        committed = {f: read(os.path.join(committed_dir, f))
                     for f in os.listdir(committed_dir) if f.endswith(".auton")}
        stale = sorted(f for f in fresh if committed.get(f) != fresh[f])
        self.assertEqual(stale, [], "autons/ is out of date; run: "
                         "tools/ez_import.py v5/src/autons.cpp v5/src/skills.cpp --all -o autons")


if __name__ == "__main__":
    unittest.main()
