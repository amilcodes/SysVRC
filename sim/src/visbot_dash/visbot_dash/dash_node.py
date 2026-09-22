"""visbot_dash — serves web/index.html on :8080 and streams telemetry on ws://:8081.

Subscribes to the controller's state/stats topics and the backend's ground-truth
odometry, merges them into one JSON frame at 30 Hz, and fans it out to every
connected browser. Also writes an optional JSONL recording for offline replay.
"""
import asyncio
import functools
import http.server
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visbot_msgs.msg import ControlState, ControlStats

try:
    import websockets
except ImportError:  # pragma: no cover
    websockets = None

IN_PER_M = 39.3700787402


def yaw_to_heading_deg(qz, qw):
    """ROS yaw (CCW from +x) -> field compass heading in (-180, 180].

    Mirrors visbot::yawToHeading / wrapDeg, including putting exactly 180 at
    +180 rather than -180.
    """
    yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    h = (90.0 - math.degrees(yaw) + 180.0) % 360.0
    if h <= 0.0:
        h += 360.0
    return h - 180.0


class DashNode(Node):
    def __init__(self):
        super().__init__("visbot_dash")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("ws_port", 8081)
        self.declare_parameter("record", "")
        self.declare_parameter("snapshot_dir", "/tmp")
        DashHandler.snapshot_dir = self.get_parameter("snapshot_dir").value
        self.frame = {"state": None, "stats": None, "truth": None, "t0": time.time()}
        self.lock = threading.Lock()
        self.create_subscription(ControlState, "/visbot/control_state", self.on_state, 10)
        self.create_subscription(ControlStats, "/visbot/control_stats", self.on_stats, 10)
        self.create_subscription(Odometry, "/visbot/odom", self.on_odom, qos_profile_sensor_data)
        rec = self.get_parameter("record").value
        self.recorder = open(rec, "w") if rec else None

    def on_state(self, m: ControlState):
        d = {
            "x": m.x, "y": m.y, "theta": m.theta_deg, "mission": m.mission, "labels": list(m.step_labels),
            "step": m.step_index, "steps": m.step_count, "step_name": m.step_name,
            "error": m.step_error, "elapsed_ms": m.step_elapsed_ms, "last_exit": m.last_exit,
            "mode": m.mode, "interfered": m.interfered,
            "action": m.last_action, "action_at_ms": m.last_action_at_ms,
            "done": m.done, "imu": m.imu_heading_deg, "enc_l": m.enc_left_deg, "enc_r": m.enc_right_deg,
            "age_us": m.sensor_age_us, "cmd_l": m.cmd_left, "cmd_r": m.cmd_right,
            "v": m.cmd_linear_mps, "w": m.cmd_angular_rps,
        }
        with self.lock:
            self.frame["state"] = d

    def on_stats(self, m: ControlStats):
        d = {
            "rate": m.rate_hz, "period_us": m.period_us, "policy": m.sched_policy, "prio": m.priority,
            "cpu": m.cpu, "mlock": m.memory_locked, "ticks": m.ticks, "overruns": m.overruns,
            "missed": m.missed_deadlines, "stale": m.stale_sensor_ticks,
            "wake_mean": m.wake_latency_mean_us, "wake_p50": m.wake_latency_p50_us,
            "wake_p99": m.wake_latency_p99_us, "wake_max": m.wake_latency_max_us,
            "jitter_rms": m.jitter_rms_us, "exec_mean": m.exec_mean_us, "exec_p50": m.exec_p50_us,
            "exec_p99": m.exec_p99_us, "exec_max": m.exec_max_us, "age_mean": m.sensor_age_mean_us,
            "window_s": m.window_seconds, "bucket_us": m.hist_bucket_us, "exec_bucket_us": m.exec_hist_bucket_us,
            "wake_hist": list(m.wake_latency_hist), "exec_hist": list(m.exec_hist),
        }
        with self.lock:
            self.frame["stats"] = d

    def on_odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.frame["truth"] = {"x": p.x * IN_PER_M, "y": p.y * IN_PER_M, "theta": yaw_to_heading_deg(q.z, q.w)}

    def snapshot(self):
        with self.lock:
            f = dict(self.frame)
        f["t"] = time.time() - f.pop("t0")
        return f


class DashHandler(http.server.SimpleHTTPRequestHandler):
    """Static files plus POST /snapshot: the page renders itself to a PNG and
    the node writes it to `snapshot_dir` (docs screenshots, CI artifacts)."""
    snapshot_dir = "/tmp"

    def log_message(self, *a, **k):
        pass

    def do_POST(self):
        if self.path != "/snapshot":
            self.send_error(404)
            return
        import base64
        n = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(n)
        if data.startswith(b"data:image/png;base64,"):
            data = base64.b64decode(data.split(b",", 1)[1])
        name = self.headers.get("X-Snapshot-Name", "dashboard.png")
        path = os.path.join(self.snapshot_dir, os.path.basename(name))
        try:
            os.makedirs(self.snapshot_dir, exist_ok=True)
            with open(path, "wb") as f:
                f.write(data)
        except OSError as e:
            self.send_error(500, f"could not write {path}: {e}")
            return
        self.send_response(200)
        self.end_headers()
        self.wfile.write(path.encode())


def serve_http(port, web_dir, log):
    handler = functools.partial(DashHandler, directory=web_dir)
    srv = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    log.info(f"dashboard: http://localhost:{port}")
    srv.serve_forever()


async def ws_main(node: DashNode, port):
    clients = set()

    async def handler(ws, *_):
        clients.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            clients.discard(ws)

    async def broadcaster():
        n = 0
        while rclpy.ok():
            f = node.snapshot()
            if f["state"] or f["stats"]:
                msg = json.dumps(f)
                # Recording: 10 Hz, histograms once a second — keeps a minute under ~1 MB.
                if node.recorder and n % 3 == 0:
                    rec = dict(f)
                    if n % 30 != 0 and rec.get("stats"):
                        rec["stats"] = {k: v for k, v in rec["stats"].items() if not k.endswith("_hist")}
                    node.recorder.write(json.dumps(rec) + "\n")
                n += 1
                dead = []
                for c in clients:
                    try:
                        await c.send(msg)
                    except Exception:
                        dead.append(c)
                for c in dead:
                    clients.discard(c)
            await asyncio.sleep(1 / 30)

    async with websockets.serve(handler, "0.0.0.0", port):
        node.get_logger().info(f"telemetry: ws://localhost:{port}")
        await broadcaster()


def main():
    rclpy.init()
    node = DashNode()
    web_dir = os.path.join(get_package_share_directory("visbot_dash"), "web")
    http_port = node.get_parameter("http_port").value
    ws_port = node.get_parameter("ws_port").value
    threading.Thread(target=serve_http, args=(http_port, web_dir, node.get_logger()), daemon=True).start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if websockets is None:
        node.get_logger().error("python3-websockets not installed; telemetry stream disabled")
        threading.Event().wait()
    try:
        asyncio.run(ws_main(node, ws_port))
    except KeyboardInterrupt:
        pass
    finally:
        if node.recorder:
            node.recorder.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
