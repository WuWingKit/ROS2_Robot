import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================================================================
    # 获取包路径
    # ================================================================
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_gazebo_pkg = get_package_share_directory('robot_gazebo')
    robot_slam_pkg = get_package_share_directory('robot_slam')
    robot_navigation_pkg = get_package_share_directory('robot_navigation')

    # ================================================================
    # Launch 参数
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operation mode: slam (建图) or nav (导航)',
        choices=['slam', 'nav']
    )

    # ================================================================
    # Gazebo 仿真场景 (始终启动)
    # ================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ================================================================
    # SLAM 建图模式 (mode=slam)
    # ================================================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_slam_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration('mode', default='slam').equals('slam')
        )
    )

    # ================================================================
    # 导航模式 (mode=nav)
    # ================================================================
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_pkg, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration('mode', default='slam').equals('nav')
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        gazebo_launch,
        slam_launch,
        nav_launch,
    ])
