#!/usr/bin/env bash
set -euo pipefail

# Run this script on the lower board in ~/atom01_deploy.
# It starts CAN setup and the robot control stack, then monitors /cmd_vel.
#
# Copy from upper board if needed:
#   scp ~/scan_ws/tools/start_lower_stack.sh sunrise@192.168.4.109:~/atom01_deploy/tools/

RUN_DIR="${ATOM_RUN_DIR:-/tmp/atom_lower_stack}"
DEPLOY_WS="${ATOM_DEPLOY_WS:-$HOME/atom01_deploy}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "$RUN_DIR"

start_bg() {
  local name="$1"
  local cmd="$2"
  local log="$RUN_DIR/$name.log"
  local pid_file="$RUN_DIR/$name.pid"

  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "$name already running, pid=$(cat "$pid_file")"
    return
  fi

  echo "Starting $name ..."
  bash -lc "$cmd" >"$log" 2>&1 &
  echo "$!" >"$pid_file"
  echo "  pid=$(cat "$pid_file"), log=$log"
}

COMMON_ENV="set +u; source /opt/ros/humble/setup.bash; set -u; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

cd "$DEPLOY_WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

echo "Running CAN setup ..."
./scripts/can.sh

start_bg robot "$COMMON_ENV; cd '$DEPLOY_WS'; set +u; source install/setup.bash; set -u; exec ./tools/start_robot.sh"

sleep 4

start_bg cmd_vel_monitor "$COMMON_ENV; cd '$DEPLOY_WS'; set +u; source install/setup.bash; set -u; exec ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --qos-reliability best_effort"

echo
echo "Lower stack started."
echo "Logs: tail -f $RUN_DIR/*.log"
echo "Check /cmd_vel: ros2 topic info -v /cmd_vel"
echo "Important: use the controller to press B to start inference, then Y to switch to /cmd_vel control."
