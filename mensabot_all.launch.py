from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Terminal 1: view_mensabot.launch.py
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/ros2_seb/install/setup.bash && '
                'cd ~/ros2_seb/src/mensabot_description/launch && '
                'ros2 launch view_mensabot.launch.py; exec bash'
            ],
            output='screen'
        ),

        # Terminal 2: mensabot.launch.py (mit TimerDelay 8s, damit Terminal 1 gestartet ist)
        TimerAction(
            period=0.0,  # X Sekunden warten
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal', '--', 'bash', '-c',
                        'source ~/ros2_seb/install/setup.bash && '
                        'cd ~/ros2_seb/src/mensabot_bringup/launch && '
                        'ros2 launch mensabot.launch.py; exec bash'
                    ],
                    output='screen'
                )
            ]
        ),

        # Terminal 3: ros2 topic pub (mit TimerDelay 14s, damit mensabot.launch läuft)
        TimerAction(
            period=5.0,  # 5 Sekunden warten, damit alle Nodes bereit sind
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal', '--', 'bash', '-c',
                        'source ~/ros2_seb/install/setup.bash && '
                        'cd ~/ros2_seb/src/mensabot_bringup/launch && '
                        'ros2 topic pub -r 5 /mensabot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped '
                        '"header:\n  stamp:\n    sec: 0\n    nanosec: 0\n'
                        'twist:\n  linear:\n    x: 0.1\n    y: 0.0\n    z: 0.0\n'
                        '  angular:\n    x: 0.0\n    y: 0.0\n    z: 0.05"; exec bash'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
