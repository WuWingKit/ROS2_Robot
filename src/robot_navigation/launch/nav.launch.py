import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    rd = get_package_share_directory('robot_description')
    rn = get_package_share_directory('robot_navigation')
    map_yaml = os.path.join(rn, 'maps', 'map.yaml')
    rviz_cfg = os.path.join(rd, 'config', 'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        LogInfo(msg='Starting Nav2 nodes directly...'),

        # map_server
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]),

        # AMCL
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[os.path.join(rn, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]),

        # planner
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[os.path.join(rn, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]),

        # controller
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[os.path.join(rn, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]),

        # behaviors
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[os.path.join(rn, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]),

        # bt_navigator
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[os.path.join(rn, 'config', 'nav2_params.yaml'), {'use_sim_time': True}]),

        # lifecycle_manager
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}]),

        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
