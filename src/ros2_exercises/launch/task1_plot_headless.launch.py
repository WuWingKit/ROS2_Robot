"""
任务1 无头测试: turtlesim + 自动运动 + pose数据终端显示
(替代 rqt_plot, 适合SSH远程测试)

用法:
  ros2 launch ros2_exercises task1_plot_headless.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. 小海龟仿真器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),

        # 2. 自动发布速度指令让海龟运动(圆形轨迹)
        Node(
            package='ros2_exercises',
            executable='turtle_pose_monitor.py',
            name='pose_monitor',
            output='screen',
        ),
    ])
