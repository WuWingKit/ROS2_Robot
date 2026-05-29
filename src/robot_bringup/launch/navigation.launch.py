import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ================================================================
    # 获取包路径
    # ================================================================
    robot_gazebo_pkg = get_package_share_directory('robot_gazebo')
    robot_navigation_pkg = get_package_share_directory('robot_navigation')

    # ================================================================
    # Launch 参数
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    map_yaml_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            robot_navigation_pkg, 'config', 'map.yaml'
        ),
        description='Full path to map yaml file to load'
    )

    # ================================================================
    # 启动 Gazebo 仿真场景
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
    # 启动 Nav2 导航
    # ================================================================
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_pkg, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_file_arg,
        gazebo_launch,
        nav_launch,
    ])
