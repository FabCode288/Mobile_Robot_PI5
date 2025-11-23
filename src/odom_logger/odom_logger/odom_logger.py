#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.sub = self.create_subscription(Odometry, '/mensabot_base_controller/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.get_logger().info("OdomLogger + Path publisher started!")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        #self.get_logger().info(f"Robot position: x={x:.2f}, y={y:.2f}")

        # PoseStamped erzeugen
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        # Pose zum Path hinzufügen
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
