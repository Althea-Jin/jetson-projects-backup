#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${ATOM_RUN_DIR:-/tmp/atom_lower_stack}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/atom01_deploy/install/setup.bash" ]]; then
  source "$HOME/atom01_deploy/install/setup.bash"
fi
set -u

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true

for name in cmd_vel_monitor robot; do
  pid_file="$RUN_DIR/$name.pid"
  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "Stopping $name, pid=$(cat "$pid_file")"
    kill "$(cat "$pid_file")" 2>/dev/null || true
  fi
done

sleep 2

for name in cmd_vel_monitor robot; do
  pid_file="$RUN_DIR/$name.pid"
  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "Force stopping $name, pid=$(cat "$pid_file")"
    kill -9 "$(cat "$pid_file")" 2>/dev/null || true
  fi
  rm -f "$pid_file"
done

echo "Lower stack stopped."
