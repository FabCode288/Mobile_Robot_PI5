#!/usr/bin/env python3
"""
Odometry Logger Node.

This ROS2 node subscribes to odometry information and builds a continuous
`nav_msgs/Path` message from the received poses.

The generated path can be visualized in RViz to show the trajectory
of the robot over time.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomLogger(Node):
    """
    ROS2 node for logging odometry data and publishing a path.

    The node:
    - subscribes to an odometry topic
    - extracts the robot pose from each message
    - appends the pose to a `nav_msgs/Path`
    - publishes the accumulated path for visualization
    """

    def __init__(self):
        """
        Initialize the OdomLogger node.

        Sets up:
        - a subscriber for odometry messages
        - a publisher for the generated path
        """
        super().__init__('odom_logger')

        self.sub = self.create_subscription(
            Odometry,
            '/mensabot_base_controller/odom',
            self.odom_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10,
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.get_logger().info(
            "OdomLogger started (publishing robot path)."
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback for received odometry messages.

        Extracts the robot pose from the odometry message,
        converts it into a `PoseStamped`, appends it to the
        internal path, and publishes the updated path.

        Args:
            msg (Odometry): Odometry message containing the robot pose
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Create PoseStamped from odometry pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        # Append pose to path and publish
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    """
    Entry point for the OdomLogger node.

    Initializes rclpy, spins the node, and performs
    a clean shutdown when the node stops.
    """
    rclpy.init(args=args)
    node = OdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
