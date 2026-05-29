import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # 获取包路径
    robot_description_pkg = get_package_share_directory('robot_description')

    # URDF 文件路径 (使用 xacro 处理)
    xacro_file = os.path.join(robot_description_pkg, 'urdf', 'robot.xacro')

    # RViz 配置文件路径
    rviz_config_file = os.path.join(robot_description_pkg, 'config', 'display.rviz')

    # 声明 launch 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher 节点
    # 使用 Command 处理 xacro 文件,生成 robot_description
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

    # Joint State Publisher GUI 节点 (用于手动调试关节)
    # 在 RViz 预览模式下启动,方便手动控制轮子
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
