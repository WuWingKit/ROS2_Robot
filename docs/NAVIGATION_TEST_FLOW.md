# 自主导航测试与录屏流程

本文档用于演示和验收大项目的导航部分：Gazebo 场景中机器人从入口出发，加载已保存地图，使用 AMCL 定位、Nav2 规划与 DWB 控制器跟踪路径，最终绕开 5 个障碍物到达出口。

## 1. 环境准备

在 Ubuntu 22.04 + ROS2 Humble 虚拟机中打开终端：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果 Gazebo 报端口占用或以前的节点没有退出，先清理：

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f spawn_entity
pkill -9 -f robot_state_publisher
pkill -9 -f amcl
pkill -9 -f map_server
pkill -9 -f planner_server
pkill -9 -f controller_server
pkill -9 -f bt_navigator
```

## 2. 录屏展示流程

建议录屏时打开 3 个终端，并把 Gazebo、RViz2 和终端都摆在屏幕上。

### 终端 1：启动完整导航系统

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup full_system.launch.py mode:=nav use_rviz:=true
```

启动后应看到：

- Gazebo 中出现 8m x 5m 场景、入口、出口和 5 个障碍物
- RViz2 中显示地图、机器人模型、激光雷达数据、全局/局部代价地图和 TF
- Nav2 节点自动进入 active 状态

### 终端 2：检查关键话题

```bash
source /opt/ros/humble/setup.bash
source ~/Hrj_ws/ROS2_Robot/install/setup.bash
ros2 topic list | grep -E "/scan|/map|/odom|/cmd_vel|/tf"
ros2 topic hz /scan
```

期望结果：

- `/scan` 存在，频率约 10Hz
- `/map`、`/odom`、`/tf`、`/cmd_vel` 存在

### 终端 3：自动发送入口到出口目标

```bash
source /opt/ros/humble/setup.bash
source ~/Hrj_ws/ROS2_Robot/install/setup.bash
python3 ~/Hrj_ws/ROS2_Robot/src/robot_navigation/scripts/nav_to_exit.py --timeout 150
```

也可以使用安装后的 ROS2 命令：

```bash
ros2 run robot_navigation nav_to_exit.py --timeout 150
```

成功时终端会输出类似：

```text
localized start=(-0.36,-0.06)  ->  goal=(6.50,-0.10)
=== NAV RESULT ===
final_pose=(6.11,-0.07)  dist_to_goal=0.39  min_dist=0.39  collided=False  dur=26.2s
RESULT: PASS
```

录屏时重点展示：

- RViz2 中出现规划路径
- Gazebo 中机器人绕过障碍物
- 机器人接近右侧出口
- 终端输出 `RESULT: PASS`

## 3. 一键自动验收流程

如果只是检查系统是否还能完整跑通，可直接运行：

```bash
cd ~/Hrj_ws/ROS2_Robot
bash src/robot_navigation/scripts/navigation_smoke_test.sh 150
```

该脚本会自动执行：

- 清理残留 Gazebo / Nav2 进程
- 构建导航相关包
- 以无 RViz 模式启动 `full_system.launch.py mode:=nav`
- 检查核心节点和话题
- 执行 `nav_to_exit.py`
- 根据输出给出 `RESULT: PASS` 或 `RESULT: FAIL`

## 4. 当前导航参数说明

关键参数位于：

```text
src/robot_navigation/config/nav2_params.yaml
```

已经针对本项目场景做过以下优化：

- `scan_topic: /scan`：与 Gazebo 激光雷达真实话题一致
- DWB 控制器补全 `vx_samples`、`vtheta_samples`、`acc_lim_*`：避免 `No valid trajectories`
- `robot_radius: 0.18`：适配 0.6m 障碍间隙
- `inflation_radius: 0.22`：保留避障余量，同时不堵死窄通道
- `track_unknown_space: False`：允许机器人走到出口附近的少量未知区域
- `use_sim_time: True`：保证 AMCL、Nav2、仿真 TF 时间一致

自动导航脚本中的 map 坐标：

```text
INIT_GUESS = (-0.45, -0.1)
GOAL_ABS   = (6.5, -0.1)
```

这是根据 Cartographer 生成地图后的实际 map 坐标解析得到的。Gazebo 世界坐标中的入口/出口仍然是作业要求的 `(0.5, 2.5)` 与 `(7.5, 2.5)`。

## 5. 常见问题

### 5.1 RViz2 中没有地图或机器人不动

检查 Nav2 生命周期是否 active：

```bash
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState "{}"
ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState "{}"
```

应看到 `label='active'`。

### 5.2 AMCL 报时间外推错误

说明有节点没有使用仿真时间。确认启动时使用：

```bash
use_sim_time:=true
```

并确认 `/clock` 存在：

```bash
ros2 topic echo /clock --once
```

### 5.3 Gazebo 启动失败并提示 Address already in use

说明上一次仿真没有完全退出，执行：

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
```

### 5.4 规划失败或机器人停在出口前

优先检查：

- `global_costmap.track_unknown_space` 是否为 `False`
- 地图文件是否为 `src/robot_navigation/maps/map.yaml`
- 目标点是否使用 map 坐标 `(6.5, -0.1)`

## 6. 提交材料建议截图

建议至少准备以下截图或视频片段：

- Gazebo 场景：入口、出口、5 个障碍物、机器人
- RViz2：地图、TF、机器人模型、LaserScan、规划路径
- 终端：`/scan` 约 10Hz
- 终端：`nav_to_exit.py` 输出 `RESULT: PASS`
- 机器人在 Gazebo 中到达右侧出口
