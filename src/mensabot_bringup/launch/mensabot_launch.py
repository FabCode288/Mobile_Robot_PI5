"""
Mensabot bringup launch file.

This launch file starts the core components required to bring up the Mensabot
robot, including:

- Robot description generation from Xacro
- ROS 2 Control node
- Robot State Publisher
- Controller spawners

The launch file is intended to be used during robot startup to initialize
the robot model, controllers, and state publishing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    """
    Generate the launch description for the Mensabot system.

    This function:
    - processes the robot Xacro file into a URDF
    - loads controller configuration files
    - creates ROS 2 nodes for control and state publishing
    - spawns required controllers

    Returns:
        LaunchDescription: ROS 2 launch description containing all nodes
    """

    # --------------------------------------------------
    # Process Xacro to generate robot description (URDF)
    # --------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory('mensabot_description'),
        'urdf',
        'mensabot.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        xacro_file
    ).toxml()

    # --------------------------------------------------
    # Controller configuration files
    # --------------------------------------------------
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

    # --------------------------------------------------
    # ROS 2 Control node
    # --------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config},
            controller_manager_yaml
        ],
        output="both",
    )

    # --------------------------------------------------
    # Robot State Publisher node
    # --------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_config}
        ],
        output="both",
    )

    # --------------------------------------------------
    # Joint State Broadcaster spawner
    # --------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # --------------------------------------------------
    # Mensabot base controller spawner
    # --------------------------------------------------
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mensabot_base_controller",
            "--param-file",
            mensabot_base_controller_yaml
        ],
        output="screen",
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
