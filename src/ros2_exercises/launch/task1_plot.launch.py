"""
任务1: rqt_plot 显示小海龟位姿
- 启动 turtlesim 仿真器节点
- 启动键盘控制器 (turtle_teleop_key)
- 启动 rqt_plot 绘制 turtle1 的 pose (x, y, theta)

用法:
  ros2 launch ros2_exercises task1_plot.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # 1. 小海龟仿真器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),

        # 2. 键盘控制器 (在独立终端运行, 控制 turtle1 运动)
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlesim', 'turtle_teleop_key'],
            output='screen',
            prefix='x-terminal-emulator -e',  # 在新终端中运行以接收键盘输入
        ),

        # 3. rqt_plot 绘制 turtle1 的位姿数据
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rqt_plot', 'rqt_plot',
                '/turtle1/pose/x',
                '/turtle1/pose/y',
                '/turtle1/pose/theta',
            ],
            output='screen',
        ),
    ])
