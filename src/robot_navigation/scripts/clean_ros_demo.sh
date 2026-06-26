#!/usr/bin/env bash
set +e

killall -q -9 \
  gzserver gzclient rviz2 \
  robot_state_publisher spawn_entity.py \
  cartographer_node cartographer_occupancy_grid_node \
  amcl map_server planner_server controller_server bt_navigator \
  lifecycle_manager behavior_server smoother_server velocity_smoother waypoint_follower nav2_container \
  nav_visualizer.py planner_compare.py clear_bonus_visuals.py auto_initial_pose.py nav_to_exit.py \
  component_container component_container_isolated

pkill -9 -f 'ros2 launch robot_bringup' 2>/dev/null || true
pkill -9 -f 'ros2 run robot_navigation planner_compare.py' 2>/dev/null || true
pkill -9 -f 'nav_visualizer.py' 2>/dev/null || true
pkill -9 -f 'planner_compare.py' 2>/dev/null || true
pkill -9 -f 'clear_bonus_visuals.py' 2>/dev/null || true
pkill -9 -f 'auto_initial_pose.py' 2>/dev/null || true
pkill -9 -f 'nav_to_exit.py' 2>/dev/null || true

echo "ROS2 demo processes cleaned."
