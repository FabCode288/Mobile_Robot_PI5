from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

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
        parameters=[{"robot_description": robot_description_config}, controller_manager_yaml],
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

    # --- Mensabot Base Controller Spawner ---
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mensabot_base_controller", "--param-file", mensabot_base_controller_yaml],
        output="screen",
    )

    # --- Kamera Publisher Node ---
    camera_pub_node = Node(
        package='camera_publisher',          # Name deines Packages
        executable='camera_publisher',          # Name des Entry-Points aus setup.py
        name='camera_publisher',
        output='screen'
    )

    shm_writer_test_node = Node(
        package='camera_publisher',          # Name deines Packages
        executable='shm_writer',          # Name des Entry-Points aus setup.py
        name='shm_writer_test',
        output='screen'
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        camera_pub_node,
        shm_writer_test_node
    ])
