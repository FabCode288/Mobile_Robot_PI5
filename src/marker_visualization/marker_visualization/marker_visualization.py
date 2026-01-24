#!/usr/bin/env python3
"""
Camera Point Handler Node.

This ROS2 node receives detected persons from a camera topic,
transforms their positions into the odometry coordinate frame,
and visualizes them as RViz markers.

Features:
- Deterministic color assignment per person ID
- First and last trajectory points visualized as spheres
- Movement direction visualized as an arrow
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

from robot_msgs.msg import Persons, PersonMarker

import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Duration


class CameraPointHandler(Node):
    """
    ROS2 node for processing and visualizing camera-based person detections.

    The node:
    - subscribes to a `Persons` topic published by a camera pipeline
    - transforms points from the camera frame to the odometry frame
    - publishes RViz markers for visualization
    """

    def __init__(self):
        """
        Initialize the CameraPointHandler node.

        Sets up:
        - TF buffer and listener
        - subscriber for detected persons
        - publisher for RViz markers
        """
        super().__init__('camera_point_handler')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_sub = self.create_subscription(
            Persons,
            '/camera_persons',
            self.persons_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/camera_point_marker',
            10
        )

        self.source_frame = 'front_camera_link'
        self.target_frame = 'odom'

        self.get_logger().info(
            'CameraPointHandler ready (listening for camera points)'
        )

    def get_color_for_person(self, person_id):
        """
        Generate a deterministic RGB color for a person.

        The color is derived from the person ID so that the same
        person is always visualized with the same color.

        Args:
            person_id (int): Unique identifier of the person

        Returns:
            tuple[float, float, float]: RGB color values in range [0.0, 1.0]
        """
        r = ((person_id * 97) % 255) / 255.0
        g = ((person_id * 53) % 255) / 255.0
        b = ((person_id * 199) % 255) / 255.0
        return r, g, b

    def persons_callback(self, msg: Persons):
        """
        Callback for received person detection messages.

        For each person:
        - the first and last trajectory points are visualized as spheres
        - an arrow marker is published to indicate movement direction

        Args:
            msg (Persons): Message containing detected persons
        """
        for person in msg.people:

            if len(person.positions) < 2:
                continue

            person_id = person.id

            first_pos = person.positions[0]
            last_pos = person.positions[-1]

            marker_id_first = person_id * 10 + 1
            marker_id_last = person_id * 10 + 2
            marker_id_arrow = person_id * 10 + 3

            self.publish_person_point(
                first_pos, marker_id_first, person_id
            )
            self.publish_person_point(
                last_pos, marker_id_last, person_id
            )

            self.publish_arrow(
                first_pos, last_pos, marker_id_arrow, person_id
            )

    def publish_person_point(self, p: Point, marker_id, person_id):
        """
        Prepare and publish a sphere marker for a person point.

        The point is wrapped in a `PointStamped`, transformed into the
        target frame, and then published as a sphere marker.

        Args:
            p (Point): Point in the camera coordinate frame
            marker_id (int): Unique marker identifier
            person_id (int): Person ID used for color assignment
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.source_frame
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = p

        self._transform_and_publish_sphere(
            point_stamped, marker_id, person_id
        )

    def _transform_and_publish_sphere(
        self, point: PointStamped, marker_id, person_id
    ):
        """
        Transform a point into the target frame and publish it as a sphere marker.

        Args:
            point (PointStamped): Point in the source coordinate frame
            marker_id (int): Marker identifier
            person_id (int): Person ID used for color assignment
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                rclpy.time.Time(seconds=0)
            )

            tp = tf2_geometry_msgs.do_transform_point(
                point, transform
            )

            r, g, b = self.get_color_for_person(person_id)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = rclpy.time.Time(
                seconds=0
            ).to_msg()
            marker.ns = f"person_{person_id}"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = tp.point.x
            marker.pose.position.y = tp.point.y
            marker.pose.position.z = tp.point.z
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.24
            marker.scale.y = 0.24
            marker.scale.z = 0.24

            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            marker.lifetime = Duration(sec=1, nanosec=0)

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(
                f'Could not transform sphere point: {e}'
            )

    def publish_arrow(
        self, first: Point, last: Point, marker_id, person_id
    ):
        """
        Create and publish an arrow marker between two points.

        The arrow represents the movement direction of a person.

        Args:
            first (Point): Start point in the camera frame
            last (Point): End point in the camera frame
            marker_id (int): Marker identifier
            person_id (int): Person ID used for color assignment
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(seconds=0)
            )

            p1 = PointStamped()
            p1.header.frame_id = self.source_frame
            p1.header.stamp = self.get_clock().now().to_msg()
            p1.point = first
            t1 = tf2_geometry_msgs.do_transform_point(
                p1, transform
            )

            p2 = PointStamped()
            p2.header.frame_id = self.source_frame
            p2.header.stamp = self.get_clock().now().to_msg()
            p2.point = last
            t2 = tf2_geometry_msgs.do_transform_point(
                p2, transform
            )

            r, g, b = self.get_color_for_person(person_id)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = rclpy.time.Time(
                seconds=0
            ).to_msg()
            marker.ns = f"person_arrow_{person_id}"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.scale.x = 0.1
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            marker.lifetime = Duration(sec=1, nanosec=0)

            marker.points = [
                Point(
                    x=t1.point.x,
                    y=t1.point.y,
                    z=t1.point.z
                ),
                Point(
                    x=t2.point.x,
                    y=t2.point.y,
                    z=t2.point.z
                )
            ]

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(
                f'Could not publish arrow: {e}'
            )


def main(args=None):
    """
    Entry point for the CameraPointHandler node.

    Initializes rclpy, spins the node, and ensures
    a clean shutdown on exit.
    """
    rclpy.init(args=args)
    node = CameraPointHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
