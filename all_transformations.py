import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class CameraToRobot(Node):
    def __init__(self):
        super().__init__('camera_to_robot')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def transform_point(self, point_camera):
        # Punkt vorbereiten
        p = PointStamped()
        p.header.frame_id = 'camera_link'
        p.point.x, p.point.y, p.point.z = point_camera

        try:
            # Transformation abrufen: camera_link → base_link
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()
            )
            point_robot = tf2_geometry_msgs.do_transform_point(p, t)
            return point_robot.point
        except Exception as e:
            self.get_logger().warn(f"Transformation von Camera zu Robot nicht verfügbar: {e}")
            return None

class RobotToWorld(Node):
    def __init__(self):
        super().__init__('robot_to_world')

        # Subscriber auf Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',          # Standard Odom-Topic
            self.odom_callback,
            10
        )

        # Letzte bekannte Roboterpose
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation_quat = np.array([0.0, 0.0, 0.0, 1.0])

    def odom_callback(self, msg: Odometry):
        # Update der Roboterpose
        self.robot_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.robot_orientation_quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def transform_point(self, point_robot):
        """
        Transformiert einen Punkt vom Roboterframe in relative Weltkoordinaten.
        Holt die aktuelle Roboterpose automatisch von /odom.
        """
        r = R.from_quat(self.robot_orientation_quat)
        R_world_robot = r.as_matrix()
        t_world_robot = self.robot_position

        point_world = R_world_robot @ point_robot + t_world_robot
        return point_world
