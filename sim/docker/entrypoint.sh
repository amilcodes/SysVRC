#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
# Headless Gazebo unless a display is wired in.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:/ws/src/sim/src/visbot_description:/ws/src/sim/src/visbot_gazebo"
exec "$@"
