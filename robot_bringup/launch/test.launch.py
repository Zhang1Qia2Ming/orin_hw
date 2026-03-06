from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
import os

def generate_launch_description():
    # 参数声明
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value='""', description="Joint name prefix"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Use mock hardware"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false", description="Mock sensors"),
        DeclareLaunchArgument("slowdown", default_value="50.0", description="Slowdown factor"),
    ]

    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")

    pkg_share = FindPackageShare('robot_bringup').find('robot_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])

    # xacro 动态生成 robot_description
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        TextSubstitution(text=' '),  # 分隔符
        xacro_file,
        TextSubstitution(text=' '),
        "prefix:=", prefix,
        TextSubstitution(text=' '),
        "use_mock_hardware:=", use_mock_hardware,
        TextSubstitution(text=' '),
        "mock_sensor_commands:=", mock_sensor_commands,
        TextSubstitution(text=' '),
        "slowdown:=", slowdown
    ])

    robot_description = {"robot_description": robot_description_content}

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("robot_bringup"),
    #         "config",
    #         "test_mock_camera_controller.yaml",
    #     ]
    # )
    
    # ros2_control_node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen',
        arguments=['--ros-args', '--log-level', 'controller_manager:=debug'],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # perception_system_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["perception_system_controller", "--controller-manager", "/controller_manager"],
    # )


    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    ld = LaunchDescription(declared_arguments + [control_node, 
    robot_state_pub_node, 
    # perception_system_controller_spawner,
    # rviz_node,
    ])
    return ld

