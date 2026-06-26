import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    robot_gazebo_pkg = get_package_share_directory('robot_gazebo')
    robot_slam_pkg = get_package_share_directory('robot_slam')
    robot_navigation_pkg = get_package_share_directory('robot_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operation mode: slam or nav',
        choices=['slam', 'nav']
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    bonus_visuals_arg = DeclareLaunchArgument(
        'bonus_visuals',
        default_value='true',
        description='Enable bonus visualization overlays in navigation mode'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_slam_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rsp': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'slam'"])
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_pkg, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'bonus_visuals': LaunchConfiguration('bonus_visuals'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        use_rviz_arg,
        bonus_visuals_arg,
        gazebo_launch,
        slam_launch,
        nav_launch,
    ])
