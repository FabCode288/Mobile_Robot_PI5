#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs


class FixedPointTransformer(Node):
    def __init__(self):
        super().__init__('fixed_point_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, '/camera_point_marker', 10)
        self.source_frame = 'front_camera_link'
        self.target_frame = 'odom'

        # Liste von festen Punkten
        self.points = [
            (0.4, 0.2, 1.0),
            (0.6, -0.3, 0.5),
            (0.6, -0.3, 1.5),
            (0.8, 0, 0.1),
            (0.8, -0.1, 0.2),
            (0.2, 0.5, 0.8)
        ]

        self.timer = self.create_timer(1.0, self.publish_points)
        self.get_logger().info('FixedPointTransformer ready (publishing multiple points)')

    def publish_points(self):
        for idx, (x, y, z) in enumerate(self.points):
            point = PointStamped()
            point.header.frame_id = self.source_frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            self._transform_and_publish(point, idx)

    def _transform_and_publish(self, point: PointStamped, marker_id: int):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                rclpy.time.Time()
            )
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "points"
            marker.id = marker_id  # eindeutige ID für jeden Punkt
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = transformed_point.point.x
            marker.pose.position.y = transformed_point.point.y
            marker.pose.position.z = transformed_point.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform point: {e}')


class CameraPointHandler(Node):
    def __init__(self):
        super().__init__('camera_point_handler')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, '/camera_point_marker', 10)

        self.source_frame = 'front_camera_link'
        self.target_frame = 'odom'

        self.subscription = self.create_subscription(
            PointStamped,
            '/camera_point',
            self.point_callback,
            10
        )

        # Counter für Marker IDs
        self.marker_id_counter = 0

        self.get_logger().info('CameraPointHandler ready (listening for camera points)')

    def point_callback(self, msg: PointStamped):
        self._transform_and_publish(msg)

    def _transform_and_publish(self, point: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                rclpy.time.Time()
            )

            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "points"

            # eindeutige ID für jeden Marker
            marker.id = self.marker_id_counter
            self.marker_id_counter += 1

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = transformed_point.point.x
            marker.pose.position.y = transformed_point.point.y
            marker.pose.position.z = transformed_point.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Lifetime, damit Marker erhalten bleiben
            marker.lifetime.sec = 5
            marker.lifetime.nanosec = 0

            self.marker_pub.publish(marker)

            #self.get_logger().info(f'Transformed camera point: ({marker.pose.position.x:.2f}, 'f'{marker.pose.position.y:.2f}, {marker.pose.position.z:.2f}), ID={marker.id}')

        except Exception as e:
            self.get_logger().warn(f'Could not transform camera point: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Hier kann einfach zwischen den beiden Klassen gewechselt werden:
    #node = FixedPointTransformer()
    node = CameraPointHandler()  # <-- später aktivieren, wenn Camera-Publisher vorhanden

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
