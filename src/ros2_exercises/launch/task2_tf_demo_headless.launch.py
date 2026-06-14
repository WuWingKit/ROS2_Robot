"""
任务2 无头测试: turtle follow + TF + tf2_echo 终端矩阵输出
(替代RViz2, 适合SSH远程测试)

用法:
  ros2 launch ros2_exercises task2_tf_demo_headless.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

        # 2. turtle1 TF 广播器
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}],
            output='screen',
        ),

        # 3. turtle2 TF 广播器
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

        # 5. 自动驱动 turtle1 做圆周运动 (并打印位姿 & 跟随状态)
        Node(
            package='ros2_exercises',
            executable='turtle_drive_and_echo.py',
            name='turtle_driver',
            output='screen',
        ),

        # 6. tf2_echo 输出变换矩阵 (本地终端)
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'turtle1', 'turtle2'],
            output='screen',
        ),
    ])
