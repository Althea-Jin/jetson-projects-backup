#!/usr/bin/env bash
set -euo pipefail

# Start Livox, FAST-LIO, SCAN-Planner, and cmd_vel follower on the upper board.
# Logs and PIDs are stored in /tmp/scan_upper_stack.
#
# Usage:
#   bash tools/start_upper_stack.sh
#   bash tools/start_upper_stack.sh --configure-link

RUN_DIR="${SCAN_RUN_DIR:-/tmp/scan_upper_stack}"
LIVOX_WS="${LIVOX_WS:-$HOME/livox_ws}"
LIO_WS="${LIO_WS:-$HOME/lio_ws}"
SCAN_WS="${SCAN_WS:-$HOME/scan_ws}"
LIDAR_NIC="${LIDAR_NIC:-enx6c1ff7ecace4}"
LIDAR_HOST_CIDR="${LIDAR_HOST_CIDR:-192.168.1.5/24}"
# FAST-LIO's bundled RViz can consume most of the Jetson GPU/CPU and make
# the estimator publish stale scans. Keep it disabled in the operational
# startup path. Start exactly one lightweight SCAN RViz instance below.
FAST_LIO_RVIZ=false
SCAN_RVIZ="${SCAN_RVIZ:-true}"
# The MID360 publishes at 10 Hz. Keeping one out of five raw points is enough
# for the current 0.08 m local-obstacle voxel grid, while preventing the
# estimator from accumulating seconds of unprocessed scans on this board.
FAST_LIO_POINT_FILTER_NUM="${FAST_LIO_POINT_FILTER_NUM:-5}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "$RUN_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$SCAN_WS/install/setup.bash"
set -u

if [[ "${1:-}" == "--configure-link" ]]; then
  echo "Configuring $LIDAR_NIC as $LIDAR_HOST_CIDR ..."
  sudo ip addr flush dev "$LIDAR_NIC"
  sudo ip addr add "$LIDAR_HOST_CIDR" dev "$LIDAR_NIC"
  sudo ip link set "$LIDAR_NIC" up
fi

# ROS 2 permits duplicate node names. PID files alone are not sufficient:
# an interrupted ros2 launch can leave child nodes alive after its recorded
# parent PID has exited. Refuse to start another local stack in that state.
assert_no_residual_processes() {
  local found=0
  local label pattern matches
  while IFS='|' read -r label pattern; do
    matches="$(pgrep -af "$pattern" 2>/dev/null || true)"
    if [[ -n "$matches" ]]; then
      echo "Residual $label process detected:"
      echo "$matches"
      found=1
    fi
  done <<'EOF'
livox|[l]ivox_ros_driver2_node
fast_lio|[f]astlio_mapping
frame_corrector|[f]astlio_frame_corrector
global_planner|[g]lobal_2d_planner
scan_planner|/scan_planner/[m]otion_plan
follower|/scan_cmd_vel_follower/[s]can_cmd_vel_follower
rviz|[r]viz2
EOF

  if (( found )); then
    echo "Refusing to start a duplicate upper stack."
    echo "Run: bash $SCAN_WS/tools/stop_upper_stack.sh"
    exit 1
  fi
}

assert_no_residual_processes

start_bg() {
  local name="$1"
  local cmd="$2"
  local log="$RUN_DIR/$name.log"
  local pid_file="$RUN_DIR/$name.pid"

  if [[ -f "$pid_file" ]]; then
    local old_pid
    old_pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
      echo "$name already running, pid=$old_pid"
      return
    fi
    rm -f "$pid_file"
  fi

  echo "Starting $name ..."
  # Create one session/process group per component. The stop script can then
  # terminate ros2 launch and all of its child nodes as one unit.
  setsid bash -lc "$cmd" >"$log" 2>&1 &
  echo "$!" >"$pid_file"
  echo "  pid=$(cat "$pid_file"), log=$log"
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-30}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "Detected $topic"
      return 0
    fi
    sleep 1
  done
  echo "Warning: timeout waiting for $topic; continuing."
  return 1
}

COMMON_ENV="set +u; source /opt/ros/humble/setup.bash; set -u; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

start_bg livox "$COMMON_ENV; cd '$LIVOX_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch livox_ros_driver2 msg_MID360_launch.py"
wait_for_topic "/livox/lidar" 30 || true

start_bg fast_lio "$COMMON_ENV; cd '$LIO_WS'; set +u; source '$LIVOX_WS/install/setup.bash'; source install/setup.bash; set -u; exec ros2 run fast_lio fastlio_mapping --ros-args --params-file '$LIO_WS/install/fast_lio/share/fast_lio/config/mid360.yaml' -p point_filter_num:=$FAST_LIO_POINT_FILTER_NUM -p publish.dense_publish_en:=true -p publish.map_en:=false -p publish.effect_map_en:=false -p publish.scan_bodyframe_pub_en:=false"
wait_for_topic "/Odometry" 45 || true
wait_for_topic "/cloud_registered" 45 || true

start_bg frame_corrector "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 run scan_planner fastlio_frame_corrector --ros-args -p input_cloud_topic:=/cloud_registered -p output_cloud_topic:=/scan/cloud_registered -p input_odom_topic:=/Odometry -p output_odom_topic:=/scan/Odometry -p input_path_topic:=/path -p output_path_topic:=/scan/path -p output_frame:=scan_map -p output_child_frame:=base_link -p correction_matrix:='[1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]'"
wait_for_topic "/scan/Odometry" 20 || true
wait_for_topic "/scan/cloud_registered" 20 || true

start_bg global_planner "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 run scan_planner global_2d_planner --ros-args -p map_topic:=/scan/cloud_registered -p odom_topic:=/scan/Odometry -p planning_frame:=scan_map -p resolution:=0.20 -p obstacle_height:=0.25 -p inflation_radius:=0.20 -p free_space_fill_radius:=0.30 -p endpoint_search_radius:=1.0 -p cell_timeout:=8.0 -p map_update_period:=1.0"
wait_for_topic "/global_costmap" 30 || true

start_bg scan_planner "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch scan_planner scan_planner.launch.py use_rviz:=false cloud_topic:=/scan/cloud_registered odom_topic:=/scan/Odometry global_path_topic:=/scan_global_path planning_frame:=scan_map"
sleep 3

start_bg follower "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch scan_cmd_vel_follower scan_cmd_vel_follower.launch.py path_topic:=/visual_local_trajectory odom_topic:=/scan/Odometry cmd_topic:=/cmd_vel planning_frame:=scan_map max_linear_x:=0.4 max_angular_z:=0.4 odom_timeout:=2.0 path_timeout:=2.0"

if [[ "$SCAN_RVIZ" == "true" ]]; then
  start_bg rviz "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 run rviz2 rviz2 -d '$SCAN_WS/config/fastlio_scan.rviz'"
fi

echo
echo "Upper stack started."
echo "Logs: tail -f $RUN_DIR/*.log"
echo "Trigger planning: bash $SCAN_WS/tools/trigger_scan_plan.sh start"
echo "Stop planning:   bash $SCAN_WS/tools/trigger_scan_plan.sh stop"
echo "Stop all:         bash $SCAN_WS/tools/stop_upper_stack.sh
"
echo
echo "Important: do not immediately send trigger false during real test."
