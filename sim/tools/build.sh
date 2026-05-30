#!/usr/bin/env bash
# Build the ROS 2 workspace inside the container (run from anywhere).
set -euo pipefail
cd "$(dirname "$0")/../.."
docker compose -f sim/docker/compose.yaml run --rm sim bash -lc \
  "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+ $*"
