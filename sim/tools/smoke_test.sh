#!/usr/bin/env bash
# End-to-end check without Gazebo: plant backend + RT controller must finish
# the skills mission over real ROS 2 topics, with no overruns and bounded
# wake latency. Run inside the container (CI does).
#
#   sim/tools/smoke_test.sh [seconds] [mission] [sched]
set -eo pipefail
SECS="${1:-22}"; MISSION="${2:-skills}"; SCHED="${3:-fifo}"; CPU="${4:-3}"; POLL="${5:-true}"; SPIN="${6:-500}"
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
set -u

setsid ros2 launch visbot_control plant.launch.py mission:="$MISSION" sched:="$SCHED" cpu:="$CPU" poll_idle:="$POLL" spin_us:="$SPIN" dash:=false > /tmp/smoke_launch.log 2>&1 &
LPID=$!
teardown() {
  kill -INT -- -"$LPID" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$LPID" 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- -"$LPID" 2>/dev/null || true
}
trap teardown EXIT
sleep "$SECS"

STATE=$(timeout 5 ros2 topic echo --once /visbot/control_state 2>/dev/null || true)
STATS=$(timeout 5 ros2 topic echo --once --no-arr /visbot/control_stats 2>/dev/null || true)
echo "$STATE" | grep -E "^(x|y|theta_deg|step_index|step_count|step_name|last_exit|done):" || echo "(no control_state)"
echo "$STATS" | grep -E "^(sched_policy|priority|memory_locked|ticks|overruns|missed_deadlines|stale_sensor_ticks|wake_latency_p50_us|wake_latency_p99_us|wake_latency_max_us|jitter_rms_us|exec_p99_us|sensor_age_mean_us):" || true

done_flag=$(echo "$STATE" | awk '/^done:/{print $2}')
overruns=$(echo "$STATS" | awk '/^overruns:/{print $2}')
p99=$(echo "$STATS" | awk '/^wake_latency_p99_us:/{print $2}')
x=$(echo "$STATE" | awk '/^x:/{print $2}'); y=$(echo "$STATE" | awk '/^y:/{print $2}')
fail=0
[ "$done_flag" = "true" ] || { echo "FAIL: mission not done"; fail=1; }
[ "${overruns:-1}" = "0" ] || { echo "FAIL: $overruns overruns"; fail=1; }
python3 - "$p99" "$x" "$y" <<'PY' || fail=1
import sys, math
p99, x, y = map(float, sys.argv[1:])
ok = True
if p99 > 4000: print(f"FAIL: wake p99 {p99} us > 4000 us"); ok = False
if math.hypot(x, y) > 4.0: print(f"FAIL: ended {math.hypot(x,y):.1f} in from origin"); ok = False
sys.exit(0 if ok else 1)
PY
[ $fail -eq 0 ] && echo "SMOKE OK" || { echo "--- launch log tail ---"; tail -30 /tmp/smoke_launch.log; exit 1; }
