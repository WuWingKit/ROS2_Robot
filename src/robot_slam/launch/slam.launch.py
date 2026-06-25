import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    robot_slam_pkg = get_package_share_directory('robot_slam')

    # ================================================================
    # 文件路径
    # ================================================================
    xacro_file = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')
    cartographer_config_dir = os.path.join(robot_slam_pkg, 'config')
    cartographer_config_basename = 'cartographer.lua'
    rviz_config_file = os.path.join(robot_description_pkg, 'config', 'navigation.rviz')

    # ================================================================
    # Launch 参数
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'
    )

    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'
    )

    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory',
        default_value=cartographer_config_dir,
        description='Full path to config directory'
    )

    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=cartographer_config_basename,
        description='Name of lua file for cartographer'
    )

    # 是否启动 robot_state_publisher (经 full_system 调用时由 gazebo.launch 提供, 置 false 避免重复)
    use_rsp_arg = DeclareLaunchArgument('use_rsp', default_value='true')
    # 是否启动 RViz (无头测试置 false)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # ================================================================
    # Robot State Publisher 节点
    # 发布机器人 TF 坐标变换 (base_link -> laser_link 等)
    # ================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rsp')),
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ================================================================
    # Cartographer 节点
    # 运行 Cartographer SLAM 算法
    # ================================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # ================================================================
    # OccupancyGrid 发布节点
    # 将 Cartographer 的子图转换为 ROS OccupancyGrid 消息
    # ================================================================
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': LaunchConfiguration('resolution'),
            'publish_period_sec': LaunchConfiguration('publish_period_sec'),
        }],
    )

    # ================================================================
    # RViz2 节点 (可选)
    # 可视化 SLAM 结果
    # ================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        publish_period_sec_arg,
        configuration_directory_arg,
        configuration_basename_arg,
        use_rsp_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
