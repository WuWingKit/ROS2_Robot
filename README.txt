============================================================
 智能传感与检测系统 第二次大作业
 轮式机器人 Gazebo 自主建图 → 路径规划 → 路径跟踪
============================================================

一、运行环境
  - Ubuntu 22.04
  - ROS2 Humble
  - Gazebo 11 (Classic, gazebo_ros_pkgs)
  - 依赖功能包(apt 安装)：
      ros-humble-gazebo-ros-pkgs
      ros-humble-cartographer-ros
      ros-humble-nav2-bringup ros-humble-navigation2
      ros-humble-robot-state-publisher ros-humble-xacro
      ros-humble-teleop-twist-keyboard

二、工作空间结构
  ros2_ws/
    src/
      robot_description/   URDF/xacro 机器人模型、RViz 配置、display 启动
      robot_gazebo/        Gazebo 世界(8m×5m 入口-障碍-出口) 与仿真启动
      robot_slam/          Cartographer 建图配置、巡游脚本、slam 启动
      robot_navigation/    Nav2 参数、地图、导航启动、导航脚本
      robot_bringup/       一键整合启动(full_system: slam/nav 两模式)
    maps/                  保存的地图 map.pgm / map.yaml
    urdf/  launch/  config/  单独提取的关键文件副本(便于查阅)

三、编译
  cd ~/ros2_ws        (本机为 ~/Hrj_ws/ROS2_Robot)
  colcon build --symlink-install
  source install/setup.bash

四、启动命令
  1) 仅显示模型(RViz 校验 URDF)：
       ros2 launch robot_description display.launch.py
  2) 启动 Gazebo 场景+机器人：
       ros2 launch robot_gazebo gazebo.launch.py
  3) SLAM 建图(Gazebo+Cartographer+RViz)：
       ros2 launch robot_bringup full_system.launch.py mode:=slam
     键盘控制机器人移动(另开终端)：
       ros2 run teleop_twist_keyboard teleop_twist_keyboard
     或自动巡游建图：
       python3 src/robot_slam/scripts/map_drive.py
     保存地图：
       ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/map
  4) 自主导航(加载地图+Nav2+RViz)：
       ros2 launch robot_bringup full_system.launch.py mode:=nav
     自动从入口(0.5,2.5)导航到出口(7.5,2.5)：
       python3 src/robot_navigation/scripts/nav_to_exit.py
     或在 RViz 用 "2D Pose Estimate" 设初始位姿、"Nav2 Goal" 设目标点。

五、关键话题
  /scan        单线激光雷达数据(10Hz, 360°, 0.1~5m)
  /odom        里程计
  /cmd_vel     差速速度指令
  /map         SLAM/导航地图
  /tf /tf_static  坐标变换(map->odom->base_link->laser_link/轮)

六、注意事项
  - 无头(无显示)运行 Gazebo 加 gui:=false。
  - 若 Gazebo 端口占用报错(Address already in use)，先 pkill -9 gzserver。
  - 雷达话题统一为 /scan(由 ray_sensor 插件 ~/out 重映射而来)。
  - 机器人车身碰撞体离地 0.05m 以保证转向灵活(避免底盘拖地)。
============================================================
