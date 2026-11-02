#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
set -u

case "${1:-start}" in
  start|true)
    ros2 topic pub --once /trigger_plan std_msgs/msg/Bool "{data: true}"
    ;;
  stop|false)
    ros2 topic pub --once /trigger_plan std_msgs/msg/Bool "{data: false}"
    ;;
  *)
    echo "Usage: $0 {start|stop}"
    exit 2
    ;;
esac
