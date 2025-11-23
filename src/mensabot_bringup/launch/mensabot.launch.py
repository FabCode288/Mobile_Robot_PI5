from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # --- Launch Argument für optionalen Joypad-Start ---
    declare_arg = DeclareLaunchArgument(
        'start_joy',
        default_value='false',
        description='Whether to launch the nodes for controlling the robot with the joypad.'
    )

    # --- Xacro expandieren ---
    xacro_file = os.path.join(
        get_package_share_directory('mensabot_description'),
        'urdf',
        'mensabot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # --- Parameterdateien ---
    controller_manager_yaml = os.path.join(
        get_package_share_directory("mensabot_bringup"),
        "config",
        "controller_manager.yaml"
    )
    mensabot_base_controller_yaml = os.path.join(
        get_package_share_directory("mensabot_bringup"),
        "config",
        "mensabot_base_controller.yaml"
    )

    # --- ROS2 Control Node ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config},
            controller_manager_yaml,
        ],
        output="both",
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config}],
        output="both",
    )

    # --- Controller Spawner ---
    joint_state_broadcaster_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
    )

    robot_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mensabot_base_controller", "--param-file", mensabot_base_controller_yaml],
                output="screen",
    )
    
    # --- Optional: Joypad-Steuerung ---
    teleop_twist_joy_launch_file_path = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )
    teleop_twist_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_twist_joy_launch_file_path),
        launch_arguments={
            'joy_config': 'xbox',
            'joy_vel': '/mensabot_base_controller/cmd_vel',
            'publish_stamped_twist': 'True'
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_joy'))
    )

    camera_pub_node = Node(
        package='camera_publisher',          # Name deines Packages
        executable='camera_publisher',          # Name des Entry-Points aus setup.py
        name='camera_publisher',
        output='screen'
    )

    return LaunchDescription([
        declare_arg,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        teleop_twist_joy_launch,
        #camera_pub_node
    ])
