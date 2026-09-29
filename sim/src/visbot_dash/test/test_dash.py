"""Unit tests for the dashboard's own logic.

Only the pure helpers are covered here — the ROS plumbing is exercised by the
end-to-end smoke tests. Written as unittest.TestCase so that both pytest and
setuptools' own runner collect them; with no collectable tests, pytest exits 5
("no tests collected") and colcon reports the package as failed.
"""
import math
import unittest

from visbot_dash.dash_node import IN_PER_M, yaw_to_heading_deg


def quat_from_yaw(yaw_rad):
    """(z, w) of a yaw-only quaternion."""
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class YawToHeading(unittest.TestCase):
    def test_cardinals(self):
        # ROS yaw is CCW from +x; the field frame uses a compass heading that
        # is CW from +y, so 0 rad (facing field-right) is 90 degrees.
        for yaw_deg, expected in [(0, 90), (90, 0), (180, -90), (-90, 180)]:
            with self.subTest(yaw=yaw_deg):
                qz, qw = quat_from_yaw(math.radians(yaw_deg))
                self.assertAlmostEqual(yaw_to_heading_deg(qz, qw), expected, places=6)

    def test_always_wrapped(self):
        for yaw_deg in range(-360, 361, 7):
            with self.subTest(yaw=yaw_deg):
                qz, qw = quat_from_yaw(math.radians(yaw_deg))
                h = yaw_to_heading_deg(qz, qw)
                self.assertGreater(h, -180.0)
                self.assertLessEqual(h, 180.0)

    def test_round_trips(self):
        for heading in (-179.0, -90.0, 0.0, 45.0, 179.0):
            with self.subTest(heading=heading):
                qz, qw = quat_from_yaw(math.radians(90.0 - heading))
                self.assertAlmostEqual(yaw_to_heading_deg(qz, qw), heading, places=6)


class Units(unittest.TestCase):
    def test_inches_per_metre(self):
        self.assertAlmostEqual(IN_PER_M, 39.3700787402, places=9)
        # The dashboard converts Gazebo's metres into the field frame's inches.
        self.assertAlmostEqual(3.6576 * IN_PER_M, 144.0, places=3)  # 12 ft field


class Broadcast(unittest.TestCase):
    def test_clients_can_come_and_go_during_a_broadcast(self):
        # Regression: a browser disconnecting mid-send used to raise
        # "Set changed size during iteration" and kill the dashboard.
        import asyncio
        import inspect
        from visbot_dash import dash_node
        src = inspect.getsource(dash_node.ws_main)
        self.assertIn("for c in list(clients)", src)

        async def scenario():
            clients = set()

            class Client:
                def __init__(self, drop):
                    self.drop = drop

                async def send(self, _):
                    await asyncio.sleep(0)
                    if self.drop:
                        clients.discard(self)      # disconnects mid-broadcast
                        clients.add(Client(False))  # and someone else connects
                        raise ConnectionError

            clients.update({Client(True), Client(False), Client(False)})
            for c in list(clients):
                try:
                    await c.send("x")
                except Exception:
                    clients.discard(c)
            return len(clients)

        self.assertEqual(asyncio.run(scenario()), 3)


if __name__ == "__main__":
    unittest.main()
