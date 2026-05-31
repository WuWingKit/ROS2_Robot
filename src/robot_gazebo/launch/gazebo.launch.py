import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ================================================================
    # 获取包路径
    # ================================================================
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_gazebo_pkg = get_package_share_directory('robot_gazebo')

    # ================================================================
    # 文件路径
    # ================================================================
    xacro_file = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')
    world_file = os.path.join(robot_gazebo_pkg, 'worlds', 'robot_world.world')

    # ================================================================
    # Launch 参数
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ================================================================
    # 处理 xacro 文件生成 robot_description
    # ================================================================
    robot_description_content = Command([
        'xacro ', xacro_file
    ])

    # ================================================================
    # Robot State Publisher 节点
    # 发布机器人 TF 坐标变换
    # ================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ================================================================
    # 启动 Gazebo 仿真器
    # 使用 Gazebo Classic (gazebo_ros_pkgs)
    # ================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false',
        }.items()
    )

    # ================================================================
    # 在 Gazebo 中生成机器人模型
    # 使用 spawn_entity 将 URDF 模型放入仿真世界
    # 初始位置: 场景入口附近 (x=1.0, y=2.5, z=0.0)
    # ================================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'articubot_robot',
            '-x', '1.0',
            '-y', '2.5',
            '-z', '0.0',
            '-Y', '3.14159',   # 朝向: 沿 +x 方向
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
    ])
