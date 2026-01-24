"""
RViz visualization launch file for the Mensabot robot.

This launch file starts RViz in different configurations depending
on the selected mode:

- Standalone mode:
  Displays only the robot model generated from the description package.
- Full robot mode:
  Displays the robot together with additional visualization nodes.

Additionally, it launches helper nodes for odometry logging and
marker visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for RViz-based visualization.

    This function:
    - declares a launch argument to switch between standalone and full mode
    - selects the appropriate RViz configuration file
    - conditionally launches RViz based on the selected mode
    - starts additional visualization and logging nodes

    Returns:
        LaunchDescription: ROS 2 launch description containing all actions
    """

    # --------------------------------------------------
    # Launch argument: standalone visualization mode
    # --------------------------------------------------
    standalone = LaunchConfiguration("standalone")

    standalone_argument = DeclareLaunchArgument(
        "standalone",
        default_value="True",
        description=(
            "If true, only the robot model is displayed in RViz. "
            "If false, the full robot visualization is started."
        ),
    )

    # --------------------------------------------------
    # RViz configuration files
    # --------------------------------------------------
    rviz_config_robot = PathJoinSubstitution(
        [FindPackageShare("mensabot_description"), "rviz", "robot.rviz"]
    )

    rviz_config_model = PathJoinSubstitution(
        [FindPackageShare("mensabot_description"), "rviz", "model.rviz"]
    )

    # --------------------------------------------------
    # RViz node: model-only visualization
    # --------------------------------------------------
    rviz_model_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_model],
        condition=IfCondition(standalone),
    )

    # --------------------------------------------------
    # RViz node: full robot visualization
    # --------------------------------------------------
    rviz_robot_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_robot],
        condition=IfCondition(
            PythonExpression(["not ", standalone])
        ),
    )

    # --------------------------------------------------
    # Odometry logger node
    # --------------------------------------------------
    odom_logger_node = Node(
        package="odom_logger",
        executable="odom_logger",
        name="odom_logger",
        output="screen",
    )

    # --------------------------------------------------
    # Marker visualization node
    # --------------------------------------------------
    marker_visualization_node = Node(
        package="marker_visualization",
        executable="marker_visualization",
        name="marker_visualization",
        output="screen",
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([
        standalone_argument,
        rviz_model_node,
        rviz_robot_node,
        odom_logger_node,
        marker_visualization_node,
    ])
