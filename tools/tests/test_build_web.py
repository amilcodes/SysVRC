"""The generated report template and the dash's copy of field.js match web/."""
import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class Generated(unittest.TestCase):
    def test_up_to_date(self):
        r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "build_web.py"), "--check"],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == "__main__":
    unittest.main()
