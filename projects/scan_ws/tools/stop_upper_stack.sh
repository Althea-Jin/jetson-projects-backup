#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${SCAN_RUN_DIR:-/tmp/scan_upper_stack}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

set +u
source /opt/ros/humble/setup.bash
set -u

if command -v ros2 >/dev/null 2>&1; then
  timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
fi

stop_recorded_process() {
  local signal="$1"
  local name="$2"
  local pid_file="$RUN_DIR/$name.pid"
  [[ -f "$pid_file" ]] || return 0

  local pid pgid
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0

  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ "$pgid" == "$pid" ]]; then
    echo "Sending $signal to $name process group, pgid=$pgid"
    kill "-$signal" -- "-$pgid" 2>/dev/null || true
  elif kill -0 "$pid" 2>/dev/null; then
    echo "Sending $signal to legacy $name process, pid=$pid"
    kill "-$signal" "$pid" 2>/dev/null || true
  fi
}

stop_residual_processes() {
  local signal="$1"
  local label pattern pid
  while IFS='|' read -r label pattern; do
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      echo "Sending $signal to residual $label process, pid=$pid"
      kill "-$signal" "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done <<'EOF'
follower|/scan_cmd_vel_follower/[s]can_cmd_vel_follower
scan_planner|/scan_planner/[m]otion_plan
frame_corrector|[f]astlio_frame_corrector
global_planner|[g]lobal_2d_planner
fast_lio|[f]astlio_mapping
livox|[l]ivox_ros_driver2_node
rviz|[r]viz2
EOF
}

for name in rviz follower scan_planner global_planner frame_corrector fast_lio livox; do
  stop_recorded_process TERM "$name"
done

sleep 2

for name in rviz follower scan_planner global_planner frame_corrector fast_lio livox; do
  stop_recorded_process KILL "$name"
done

stop_residual_processes TERM
sleep 1
stop_residual_processes KILL

for name in rviz follower scan_planner global_planner frame_corrector fast_lio livox; do
  pid_file="$RUN_DIR/$name.pid"
  rm -f "$pid_file"
done

echo "Upper stack stopped."
