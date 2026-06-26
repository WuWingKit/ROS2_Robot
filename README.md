# 基于 ROS2 的轮式机器人自主建图、路径规划与安全控制系统

本项目为《智能传感与检测系统》课程第二次大作业，基于 Ubuntu 22.04、ROS2 Humble 和 Gazebo Classic 11 实现轮式机器人在仿真场景中的自主建图、路径规划、路径跟踪与安全控制。系统包含 Gazebo 场景搭建、URDF/xacro 机器人建模、二维激光雷达仿真、Cartographer SLAM、Nav2 自主导航、RViz 可视化增强、Dijkstra 与 A* 路径对比，以及 AEB 自动紧急制动演示。

## 1. 项目目标

作业主线目标：

- 在 Gazebo 中搭建 8 m x 5 m 的入口、障碍物、出口仿真场景。
- 使用 URDF/xacro 建立差速轮式机器人模型，包含 2 个驱动轮、1 个随动轮和 1 个二维激光雷达。
- 使用 Cartographer 完成二维 SLAM 建图，并保存 `map.pgm` 与 `map.yaml`。
- 使用 Nav2 加载保存地图，完成从入口区域到出口区域的路径规划与路径跟踪。
- 在 RViz 中显示地图、小车模型、TF、激光点云、全局路径和局部路径。

扩展加分功能：

- RViz 导航增强显示：起点、终点、本次运行轨迹、路径 Marker。
- Dijkstra 与 A* 路径规划对比：展示两种算法的路径、搜索节点数与耗时差异。
- AEB 自动紧急制动：手动控制小车接近障碍物时，根据 `/scan` 最近距离自动停车并弹出提示。

## 2. 运行环境

推荐环境：

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| 仿真器 | Gazebo Classic 11 |
| 建图 | Cartographer ROS |
| 导航 | Navigation2 / Nav2 |
| 可视化 | RViz2 |

安装依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard
```

## 3. 工程结构

```text
ROS2_Robot/
├── src/
│   ├── robot_description/      # URDF/xacro 机器人模型、Gazebo 插件、RViz 配置
│   ├── robot_gazebo/           # Gazebo world 和仿真启动文件
│   ├── robot_slam/             # Cartographer 参数、SLAM 启动、自动建图脚本
│   ├── robot_navigation/       # Nav2 参数、地图、导航脚本、扩展功能脚本
│   ├── robot_bringup/          # 一键启动入口：建图、导航、AEB 控制
│   └── ros2_exercises/         # 课程小练习：小海龟、TF、rqt plot 等
├── docs/                       # 测试流程、扩展功能说明、报告辅助材料
├── maps/                       # 交付用地图副本
├── config/                     # 交付用配置副本
├── launch/                     # 交付用 launch 副本
├── urdf/                       # 交付用 URDF/xacro 副本
├── README.md                   # GitHub 项目说明
└── README.txt                  # 原始课程交付说明
```

核心包说明：

| 包名 | 作用 |
| --- | --- |
| `robot_description` | 定义机器人几何、碰撞、惯性、轮子、雷达、Gazebo 插件和 RViz 显示配置。 |
| `robot_gazebo` | 加载 8 m x 5 m 作业场景，生成机器人，提供 Gazebo 仿真环境。 |
| `robot_slam` | 使用 Cartographer 订阅 `/scan` 和 `/odom` 进行二维建图。 |
| `robot_navigation` | 配置 Nav2、AMCL、costmap、路径规划、导航验证和扩展功能。 |
| `robot_bringup` | 提供统一启动入口，支持 `slam`、`nav` 和 AEB 独立控制模式。 |

## 4. 编译

进入工作空间根目录：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果是首次克隆到新机器，建议先安装依赖：

```bash
cd ~/Hrj_ws/ROS2_Robot
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 5. 一键启动命令

### 5.1 建图模式

启动 Gazebo、Cartographer 和建图 RViz：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_bringup full_system.launch.py mode:=slam use_rviz:=true
```

建图时 RViz 应显示：

- `RobotModel`：小车模型。
- `TF`：`map -> odom -> base_link -> laser_link` 坐标关系。
- `LaserScan /scan`：激光雷达点云。
- `SLAM Map /map`：实时生成的地图。

手动控制小车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

自动巡游建图：

```bash
ros2 run robot_slam map_drive.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f src/robot_navigation/maps/map
```

保存后应生成：

```text
src/robot_navigation/maps/map.pgm
src/robot_navigation/maps/map.yaml
```

### 5.2 导航模式

启动 Gazebo、Nav2、AMCL 和导航 RViz：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_bringup full_system.launch.py mode:=nav use_rviz:=true bonus_visuals:=true
```

RViz 操作步骤：

1. 使用 `2D Pose Estimate` 设置小车初始位姿。
2. 等待 AMCL 粒子和地图对齐。
3. 使用 `2D Goal Pose` 设置出口目标点。
4. 观察紫色全局路径、局部路径、激光点云和小车运动。

自动导航到出口：

```bash
ros2 run robot_navigation nav_to_exit.py
```

无头导航验收：

```bash
bash src/robot_navigation/scripts/navigation_smoke_test.sh 150
```

### 5.3 仅显示 RobotModel 与 TF

用于截图 `RobotModel + TF`：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_description display.launch.py
```

RViz 中建议开启：

- `RobotModel`
- `TF`
- `Grid`

截图时应包含 `base_link`、`laser_link`、左右轮 link 和随动轮 link。

## 6. 扩展功能运行方法

### 6.1 Dijkstra 与 A* 路径对比

启动导航模式后，在新终端运行：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation planner_compare.py --publish
```

RViz 中显示：

| 算法 | 中文读法 | RViz 颜色 | 特点 |
| --- | --- | --- | --- |
| Dijkstra | 迪杰斯特拉算法 | 橙黄色 | 无启发式，搜索完整，访问节点较多。 |
| A* | A 星算法 | 青蓝色 | 使用启发式距离，访问节点更少，效率更高。 |

本项目地图中测试结果约为：

| 算法 | 路径长度 | 搜索节点数 | 耗时 |
| --- | --- | --- | --- |
| Dijkstra | 6.57 m | 9791 | 50-70 ms |
| A* | 6.57 m | 2345 | 14-17 ms |

### 6.2 AEB 自动紧急制动

AEB 演示不使用建图或导航 RViz，只启动 Gazebo 和手动控制节点。

终端 1：启动 AEB 控制场景。

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_bringup aeb_control.launch.py
```

终端 2：启动键盘控制与 AEB。

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation aeb_keyboard_control.py
```

可调参数示例：

```bash
ros2 run robot_navigation aeb_keyboard_control.py --ros-args \
  -p aeb_distance:=0.55 \
  -p release_distance:=0.70
```

参数含义：

- `aeb_distance`：AEB 触发距离。最近障碍物距离小于该值时，小车自动停车。
- `release_distance`：AEB 解除距离。最近障碍物距离大于该值后，允许继续控制。

键盘控制：

```text
w : 前进
x : 后退
a : 左转
d : 右转
s / Space : 停止
q : 退出
```

## 7. 关键话题与坐标系

关键话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 二维激光雷达数据，供 SLAM、AMCL、costmap 和 AEB 使用。 |
| `/odom` | `nav_msgs/Odometry` | Gazebo 差速驱动插件发布的里程计。 |
| `/cmd_vel` | `geometry_msgs/Twist` | 控制小车线速度和角速度。 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 或 map_server 发布的栅格地图。 |
| `/tf` | `tf2_msgs/TFMessage` | 动态坐标变换。 |
| `/tf_static` | `tf2_msgs/TFMessage` | URDF 固定关节坐标变换。 |
| `/plan` | `nav_msgs/Path` | Nav2 全局路径。 |
| `/local_plan` | `nav_msgs/Path` | Nav2 局部路径。 |

主要 TF 链：

```text
map -> odom -> base_link -> laser_link
                         -> left_wheel_link
                         -> right_wheel_link
                         -> caster_wheel_link
```

说明：

- 建图模式下，Cartographer 负责发布 `map -> odom`。
- 导航模式下，AMCL 负责发布 `map -> odom`。
- Gazebo 差速驱动插件负责发布 `odom -> base_link`。
- URDF 固定关节负责发布 `base_link -> laser_link` 等静态关系。

## 8. 截图与报告建议

建议提交报告时包含以下截图：

| 图号建议 | 截图内容 |
| --- | --- |
| 图 1-1 | Gazebo 场景总览，显示入口、出口、墙体和 5 个障碍物。 |
| 图 3-1 | Gazebo 中障碍物布置截图。 |
| 图 4-1 | RViz 中 `RobotModel + TF` 坐标系截图。 |
| 图 4-2 | Gazebo 中机器人模型截图。 |
| 图 5-2 | RViz 中 `/scan` 激光点云截图。 |
| 图 6-1 | SLAM 实时建图过程截图。 |
| 图 6-2 | 最终保存地图截图。 |
| 图 7-2 | 导航 RViz 截图，显示地图、小车、激光点云和路径。 |
| 图 7-3 | 小车到达出口附近截图。 |
| 图 9-2 | Dijkstra 与 A* 路径对比截图。 |
| 图 9-3 | AEB 触发弹窗截图。 |

## 9. 常见问题

### 9.1 RViz 中 Map 显示 `No map received`

检查当前模式：

- 建图模式下，Cartographer 启动后才会逐步发布 `/map`。
- 导航模式下，`map_server` 必须成功加载 `map.yaml`。

检查命令：

```bash
ros2 topic list | grep /map
ros2 topic echo /map --once
```

### 9.2 点云与地图偏移

常见原因：

- 导航模式未用 `2D Pose Estimate` 设置 AMCL 初始位姿。
- 小车转弯速度过快，激光帧与位姿估计短时不同步。
- `use_sim_time` 未统一设置为 `true`。

建议：

- 导航前先设置初始位姿。
- 建图时慢速移动和慢速旋转。
- 检查 `/tf` 是否连续。

### 9.3 小车转向不灵敏

本项目已通过调整车体碰撞体离地间隙解决底盘拖地问题。若再次出现转向困难，检查：

- `robot.xacro` 中车体 collision 是否与地面接触。
- 左右轮 joint axis 是否方向正确。
- Gazebo 中地面高度是否使车体或障碍物发生异常接触。

### 9.4 重新启动后 RViz 仍显示旧路径

可运行清理脚本：

```bash
bash src/robot_navigation/scripts/clean_ros_demo.sh
```

或在导航模式中重新设置初始位姿和目标点，`nav_visualizer.py` 会只记录当次路径 history。

### 9.5 Gazebo 端口占用

```bash
pkill -9 gzserver
pkill -9 gzclient
```

然后重新启动 launch。

## 10. 开发与提交说明

查看工作区状态：

```bash
git status
```

提交修改：

```bash
git add <files>
git commit -m "docs: update project readme"
git push origin main
```

注意：

- 不提交 `build/`、`install/`、`log/` 等 ROS2 构建产物。
- 不提交个人授权二维码、临时缓存、录屏大文件。
- 推荐每完成一个功能或测试门禁后进行一次 Git 提交。

## 11. 快速复现流程

从零复现主线演示：

```bash
cd ~/Hrj_ws/ROS2_Robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 1. 建图
ros2 launch robot_bringup full_system.launch.py mode:=slam use_rviz:=true

# 2. 保存地图
ros2 run nav2_map_server map_saver_cli -f src/robot_navigation/maps/map

# 3. 导航
ros2 launch robot_bringup full_system.launch.py mode:=nav use_rviz:=true bonus_visuals:=true

# 4. 自动导航验证
ros2 run robot_navigation nav_to_exit.py
```

项目完成状态：

- Gazebo 场景：已完成。
- URDF 差速小车：已完成。
- 单线激光雷达 `/scan`：已完成。
- Cartographer SLAM：已完成。
- Nav2 自主导航：已完成。
- RViz 可视化：已完成。
- Dijkstra 与 A* 对比：已完成。
- AEB 自动紧急制动：已完成。
