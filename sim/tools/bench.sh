#!/usr/bin/env bash
# Run the RT loop benchmark matrix and drop JSON + CSV into docs/bench/.
# Run inside the container:  sim/tools/bench.sh [seconds] [cpu]
set -euo pipefail
SECS="${1:-10}"; CPU="${2:-3}"
OUT=/ws/src/sysvrc/docs/bench
mkdir -p "$OUT"
B=/ws/install/visbot_control/lib/visbot_control/rt_bench
run() { # name, args...
  local name=$1; shift
  echo "== $name: $*"
  "$B" "$@" --seconds "$SECS" --csv "$OUT/$name.csv" > "$OUT/$name.json"
  grep -E "wake_latency|missed" "$OUT/$name.json"
}
run other_idle          --policy other
run other_stress        --policy other --stress 8
run fifo_idle           --policy fifo
run fifo_stress         --policy fifo --stress 8
run fifo_pin_pollidle   --policy fifo --cpu "$CPU" --poll-idle
run fifo_pin_pollidle_stress --policy fifo --cpu "$CPU" --poll-idle --stress 8
run fifo_pin_pollidle_spin   --policy fifo --cpu "$CPU" --poll-idle --spin-us 500
run fifo_spin3000       --policy fifo --spin-us 3000
uname -r > "$OUT/kernel.txt"; nproc >> "$OUT/kernel.txt"
echo "wrote $OUT"
