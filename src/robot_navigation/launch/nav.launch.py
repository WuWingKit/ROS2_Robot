import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ================================================================
    # 获取包路径
    # ================================================================
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_navigation_pkg = get_package_share_directory('robot_navigation')

    # ================================================================
    # 文件路径
    # ================================================================
    xacro_file = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')
    nav2_params_file = os.path.join(robot_navigation_pkg, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(robot_navigation_pkg, 'config', 'map.yaml')
    nav2_bt_xml_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    )
    rviz_config_file = os.path.join(robot_description_pkg, 'config', 'display.rviz')

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
        default_value=map_yaml_file,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup'
    )

    # ================================================================
    # Robot State Publisher 节点
    # 发布机器人 TF 坐标变换 (base_link -> laser_link 等)
    # ================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ================================================================
    # Nav2 Bringup 启动
    # 包括: map_server, amcl, planner_server, controller_server,
    #       recoveries_server, bt_navigator, waypoint_follower
    # ================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_rviz': 'false',  # 我们自己启动 rviz
        }.items()
    )

    # ================================================================
    # RViz2 节点 (可选)
    # ================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_file_arg,
        params_file_arg,
        use_rviz_arg,
        autostart_arg,
        use_composition_arg,
        robot_state_publisher_node,
        nav2_bringup,
        rviz_node,
    ])
