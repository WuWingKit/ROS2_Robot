#!/usr/bin/env bash

# One-command navigation smoke test for the course demo.
# Run from the workspace root after sourcing ROS2:
#   bash src/robot_navigation/scripts/navigation_smoke_test.sh

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TIMEOUT_SEC="${1:-150}"

source /opt/ros/humble/setup.bash
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

set -u

cleanup_ros_processes() {
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f spawn_entity 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f amcl 2>/dev/null || true
  pkill -9 -f map_server 2>/dev/null || true
  pkill -9 -f controller_server 2>/dev/null || true
  pkill -9 -f planner_server 2>/dev/null || true
  pkill -9 -f bt_navigator 2>/dev/null || true
  pkill -9 -f lifecycle_manager 2>/dev/null || true
  pkill -9 -f behavior_server 2>/dev/null || true
  for _ in $(seq 1 8); do
    ss -tlnp 2>/dev/null | grep -q 11345 || break
    sleep 1
  done
}

cd "${WORKSPACE_DIR}"

echo "=== Navigation smoke test ==="
echo "workspace: ${WORKSPACE_DIR}"
echo "timeout: ${TIMEOUT_SEC}s"

cleanup_ros_processes
sleep 1

echo "=== Build navigation packages ==="
colcon build --symlink-install --packages-select robot_navigation robot_bringup robot_gazebo robot_description 2>&1 | tail -12
source install/setup.bash

echo "=== Start full system in nav mode ==="
nohup ros2 launch robot_bringup full_system.launch.py mode:=nav use_rviz:=false > /tmp/robot_nav_smoke_full_system.log 2>&1 &
sleep 30

echo "=== Node check ==="
ros2 node list 2>/dev/null | grep -E "gazebo|diff_drive|map_server|amcl|planner_server|controller_server|bt_navigator" | sort

echo "=== Topic check ==="
ros2 topic list 2>/dev/null | grep -E "^/scan$|^/map$|^/odom$|^/cmd_vel$|^/tf$" | sort

echo "=== Run autonomous navigation ==="
python3 src/robot_navigation/scripts/nav_to_exit.py --timeout "${TIMEOUT_SEC}" 2>&1 | tee /tmp/robot_nav_smoke_result.log

RESULT="$(grep -E "RESULT: (PASS|FAIL)" /tmp/robot_nav_smoke_result.log | tail -1 || true)"
echo "=== Smoke test result ==="
echo "${RESULT:-RESULT: UNKNOWN}"

cleanup_ros_processes

if echo "${RESULT}" | grep -q "PASS"; then
  exit 0
fi
exit 1
