#!/usr/bin/env bash
# Headless Gazebo end-to-end: world loads, robot spawns, sensors bridge into
# ROS 2, the RT controller drives the mission on the physics backend.
#   sim/tools/gazebo_smoke.sh [seconds] [mission]
set -eo pipefail
SECS="${1:-25}"; MISSION="${2:-square}"
# Max tolerated odometry-vs-truth drift, inches. This is a property of the
# routine, not of the code: encoder+IMU odometry cannot see wheel slip, so an
# all-out rush drifts far more than a gentle square. See docs/fidelity.md.
DRIFT_MAX="${3:-6.0}"
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
set -u
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"

setsid ros2 launch visbot_gazebo sim.launch.py mission:="$MISSION" dash:=false cpu:=3 poll_idle:=true spin_us:=500 > /tmp/gz_launch.log 2>&1 &
LPID=$!
teardown() {
  kill -INT -- -"$LPID" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "$LPID" 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- -"$LPID" 2>/dev/null || true
}
trap teardown EXIT
sleep "$SECS"

# One-shot reads can take several seconds on a small CI runner: the ros2 CLI
# has to start and discover the graph. Retry with a generous timeout rather
# than mistake a slow start for a dead topic.
echo_once() {  # topic [extra args...]
  local out=""
  for _ in 1 2 3; do
    out=$(timeout 15 ros2 topic echo --once "$@" 2>/dev/null || true)
    [ -n "$out" ] && break
  done
  printf '%s' "$out"
}

echo "--- topic rates (gz -> ros bridge) ---"
imu_ok=0
for t in /visbot/imu /visbot/joint_states /visbot/odom; do
  rate=$({ timeout 10 ros2 topic hz "$t" 2>/dev/null || true; } | grep -m1 "average rate" || true)
  printf "%-22s %s\n" "$t" "${rate:-NO DATA}"
  [ "$t" = /visbot/imu ] && [ -n "$rate" ] && imu_ok=1
done
STATE=$(echo_once /visbot/control_state)
STATS=$(echo_once /visbot/control_stats --no-arr)
ODOM=$(echo_once /visbot/odom)
echo "--- controller ---"
echo "$STATE" | grep -E "^(x|y|theta_deg|step_index|step_count|step_name|last_exit|done):" || echo "(no control_state)"
echo "$STATS" | grep -E "^(sched_policy|ticks|overruns|missed_deadlines|stale_sensor_ticks|wake_latency_p99_us|wake_latency_max_us|sensor_age_mean_us):" || true
echo "--- gazebo ground truth (m) ---"
echo "$ODOM" | sed -n '/position:/,/z:/p' | tr -d ' ' | tr '\n' ' ' || true; echo

done_flag=$(echo "$STATE" | awk '/^done:/{print $2}')
fail=0
[ "$imu_ok" = "1" ] || { echo "FAIL: no IMU data bridged from Gazebo"; fail=1; }
[ "$done_flag" = "true" ] || { echo "FAIL: mission not done"; fail=1; }
python3 - "$STATE" "$ODOM" "$DRIFT_MAX" <<'PY' || fail=1
import re, sys, math
st, od, drift_max = sys.argv[1], sys.argv[2], float(sys.argv[3])
g = lambda k, s: float(re.search(rf"^{k}: (\S+)", s, re.M).group(1))
try:
    ex, ey = g("x", st), g("y", st)
    m = re.search(r"position:\s*x: (\S+)\s*y: (\S+)", od)
    tx, ty = float(m.group(1)) * 39.37, float(m.group(2)) * 39.37
except Exception as e:
    print("FAIL: could not parse", e); sys.exit(1)
err = math.hypot(ex - tx, ey - ty)
print(f"odometry vs gazebo truth: {err:.2f} in (limit {drift_max:.1f})")
sys.exit(0 if err < drift_max else 1)
PY
[ $fail -eq 0 ] && echo "GAZEBO SMOKE OK" || { echo "--- launch log tail ---"; tail -40 /tmp/gz_launch.log; exit 1; }
