from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, PythonExpression, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    standalone = LaunchConfiguration("standalone")
    standalone_argument = DeclareLaunchArgument(
        "standalone",
        default_value="True",
        description="Just view the robot model, generated from the description.",
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("mensabot_description"), "urdf", "mensabot.urdf.xacro"]),
            " model_only:=True"
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    rviz_config_robot = PathJoinSubstitution([FindPackageShare("mensabot_description"), "rviz", "robot.rviz"])
    rviz_config_model = PathJoinSubstitution([FindPackageShare("mensabot_description"), "rviz", "model.rviz"])


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        condition=IfCondition(standalone)
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(standalone)
    )

    rviz_model_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_model],
        condition=IfCondition(standalone)
    )
    
    rviz_robot_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_robot],
        condition=IfCondition(PythonExpression(['not ', standalone]))
    )

    odom_logger_node = Node(
        package='odom_logger',          # Name deines Packages
        executable='odom_logger',          # Name des Entry-Points aus setup.py
        name='odom_logger',
        output='screen'
    )

    marker_visualization_node = Node(
        package='marker_visualization',          # Name deines Packages
        executable='marker_visualization',          # Name des Entry-Points aus setup.py
        name='marker_visualization',
        output='screen'
    )

    return LaunchDescription([
        standalone_argument,
        robot_state_publisher_node,
        #joint_state_publisher_gui_node,
        rviz_model_node,
        rviz_robot_node,
        odom_logger_node,
        marker_visualization_node
    ])