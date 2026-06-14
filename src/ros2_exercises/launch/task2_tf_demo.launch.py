"""
任务2: 小海龟跟随 + TF坐标变换 + RViz2可视化
- 启动 turtlesim 仿真器 (turtle1 + turtle2)
- turtle1 的 TF broadcaster (广播 turtle1 的位姿)
- turtle2 的 TF broadcaster (广播 turtle2 的位姿)
- turtle2 跟随 turtle1 的 listener 节点
- 键盘控制器 (控制 turtle1, turtle2 自动跟随)
- RViz2 显示 TF 坐标转换关系
- 终端显示两个小海龟之间的坐标变换矩阵 (tf2_echo)

用法:
  ros2 launch ros2_exercises task2_tf_demo.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_exercises')
    rviz_config = os.path.join(pkg_share, 'rviz', 'tf_display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='turtle2 要跟随的目标坐标系'
        ),

        # 1. 小海龟仿真器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen',
        ),

        # 2. turtle1 的 TF 广播器
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}],
            output='screen',
        ),

        # 3. turtle2 的 TF 广播器
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[{'turtlename': 'turtle2'}],
            output='screen',
        ),

        # 4. turtle2 跟随 turtle1 的 listener
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[{'target_frame': LaunchConfiguration('target_frame')}],
            output='screen',
        ),

        # 5. 键盘控制器 (控制 turtle1 运动, turtle2 自动跟随)
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlesim', 'turtle_teleop_key'],
            output='screen',
            prefix='x-terminal-emulator -e',
        ),

        # 6. RViz2 可视化 TF 坐标转换
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # 7. tf2_echo 在终端输出两个小海龟的坐标变换矩阵
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'turtle1', 'turtle2'],
            output='screen',
            prefix='x-terminal-emulator -e',
        ),
    ])
